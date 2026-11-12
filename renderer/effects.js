import * as THREE from 'three';
import {mat4, vec3} from './camera.js';
import {vertexShader} from './splats.js';

// B · 렌더 패스 구성. Three.js는 렌더 타깃(FBO)·깊이 텍스처 생성과 드로 호출만 맡고,
// 패스 순서(그림자 깊이 → 최전면 깊이 → 가중 누적 → 합성)와 모든 셰이더는 여기서 직접 정한다.

// Convert hardware depth to positive camera-space distance in metres.
const depthFunction = `
  uniform vec2 clipPlanes;
  float distanceAt(float d) {
    return clipPlanes.x * clipPlanes.y / mix(clipPlanes.y, clipPlanes.x, d);
  }`;

export function createEffects(renderer, scene, camera, mesh, {shadowSize = 2048} = {}) {
  const base = mesh.material;
  const nearest = new THREE.WebGLRenderTarget(1, 1, {
    depthTexture: new THREE.DepthTexture(1, 1, THREE.UnsignedIntType),
    minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter
  });
  // 디버그 뷰의 법선 버퍼. mode=1로 한 번 더 그린다.
  const normals = new THREE.WebGLRenderTarget(1, 1, {minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter});
  const canBlend = renderer.extensions.has('EXT_color_buffer_float');
  const sum = canBlend ? new THREE.WebGLRenderTarget(1, 1, {
    type: THREE.HalfFloatType, depthBuffer: false,
    minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter
  }) : null;
  const clipPlanes = {value: new THREE.Vector2(camera.near, camera.far)};
  const nearestDepth = {value: nearest.depthTexture};

  // ---- 그림자 맵: 광원 시점 직교 투영으로 깊이만 기록한다. ----
  const shadow = new THREE.WebGLRenderTarget(shadowSize, shadowSize, {
    depthTexture: new THREE.DepthTexture(shadowSize, shadowSize, THREE.UnsignedIntType),
    minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter
  });
  base.uniforms.shadowMap.value = shadow.depthTexture;
  base.uniforms.shadowSize.value.set(shadowSize, shadowSize);
  const lightView = new Float32Array(16), lightProjection = new Float32Array(16);
  const depthOnly = new THREE.RawShaderMaterial({
    side: THREE.DoubleSide,
    uniforms: {...base.uniforms,
      uProjection: {value: lightProjection}, uView: {value: lightView},
      viewport: {value: new THREE.Vector2(shadowSize, shadowSize)},
      // 점군은 틈이 많아 원판을 그대로 쓰면 그림자가 체처럼 샌다. 캐스터만 3배로 키워 빈틈을 메운다.
      radiusScale: {value: 3}, spacingStrength: {value: 0}},
    vertexShader,
    fragmentShader: `
      precision highp float;
      varying vec2 disc;
      void main() {
        if (dot(disc,disc) > 1.0) discard;
        gl_FragColor = vec4(1.0);
      }`
  });
  const {min, max} = mesh.userData.bounds;
  const center = [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2];
  const radius = vec3.length(vec3.sub(max, center)) || 1;
  function updateLight(lightDir) {
    const L = vec3.normalize(lightDir);
    const eye = vec3.add(center, vec3.scale(L, 2 * radius));
    const up = Math.abs(L[1]) > 0.9 ? [1, 0, 0] : [0, 1, 0];
    lightView.set(mat4.lookAt(eye, center, up));
    lightProjection.set(mat4.orthographic(-radius, radius, -radius, radius, radius * 0.5, radius * 3.5));
    mat4.multiply(lightProjection, lightView, base.uniforms.uLightVP.value);
  }

  // ---- 가중 누적 패스: 최전면 표면에서 허용 오차 안의 조각만 가우시안 가중으로 더한다. ----
  const accumulation = base.clone();
  accumulation.uniforms = {...base.uniforms, clipPlanes, nearestDepth};
  accumulation.fragmentShader = base.fragmentShader.replace('void main()', `
    uniform sampler2D nearestDepth;
    uniform vec2 viewport;
    ${depthFunction}
    void main()`).replace('gl_FragColor = vec4(color,1);', `
      float front = distanceAt(texture2D(nearestDepth, gl_FragCoord.xy / viewport).r);
      float separation = distanceAt(gl_FragCoord.z) - front;
      if (separation > max(0.015, front * 0.003)) discard;
      float weight = exp(-2.5 * dot(disc,disc));
      gl_FragColor = vec4(pow(color,vec3(2.2))*weight,weight);`);
  Object.assign(accumulation, {
    transparent: true, forceSinglePass: true, depthTest: false, depthWrite: false,
    blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
    blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor
  });

  // ---- 합성 패스: 정규화, EDL, 디버그 2×2 ----
  const composite = new THREE.RawShaderMaterial({
    uniforms: {
      clipPlanes, nearestDepth, baseColor: {value: nearest.texture},
      normalColor: {value: normals.texture},
      weightedColor: {value: sum?.texture ?? nearest.texture},
      viewport: base.uniforms.viewport, blend: {value: false}, edl: {value: false},
      debug: {value: false}, depthRange: {value: 40},
      shadowDepth: {value: shadow.depthTexture}, showShadowMap: {value: false}
    },
    vertexShader: `
      precision highp float;
      attribute vec3 position;
      attribute vec2 uv;
      varying vec2 tex;
      void main(){tex = uv; gl_Position = vec4(position.xy,0,1);}`,
    fragmentShader: `
      precision highp float;
      varying vec2 tex;
      uniform sampler2D nearestDepth, baseColor, normalColor, weightedColor, shadowDepth;
      uniform vec2 viewport;
      uniform bool blend, edl, debug, showShadowMap;
      uniform float depthRange;
      ${depthFunction}
      float response(vec2 t, vec2 offset, float center) {
        float d = texture2D(nearestDepth, t + offset / viewport).r;
        if (d >= 1.0) return 0.0;
        return max(0.0,log(center / distanceAt(d)));
      }
      vec3 finalColor(vec2 t) {
        vec3 color = texture2D(baseColor, t).rgb;
        float d = texture2D(nearestDepth, t).r;
        if (d < 1.0) {
          if (blend) {
            vec4 total = texture2D(weightedColor, t);
            if (total.a > 0.0) color = pow(total.rgb / total.a, vec3(1.0/2.2));
          }
          if (edl) {
            float z = distanceAt(d);
            float shadow = response(t, vec2(2,0), z) + response(t, vec2(-2,0), z)
                         + response(t, vec2(0,2), z) + response(t, vec2(0,-2), z);
            color *= max(0.45, exp(-2.0*shadow));
          }
        }
        return color;
      }
      void main(){
        if (!debug) { gl_FragColor = vec4(finalColor(tex),1); return; }
        // 왼쪽 위 깊이(그림자 맵이 켜져 있으면 광원 시점 깊이), 오른쪽 위 법선, 왼쪽 아래 누적 가중치, 오른쪽 아래 최종
        vec2 cell = floor(tex * 2.0), t = fract(tex * 2.0);
        vec3 color;
        float d = texture2D(nearestDepth, t).r;
        if (cell.y > 0.5 && cell.x < 0.5) {
          if (showShadowMap) {
            float s = texture2D(shadowDepth, t).r;
            color = s < 1.0 ? vec3(1.0 - s) * 1.6 : vec3(0.0);
          } else color = d < 1.0 ? vec3(1.0 - clamp(distanceAt(d) / depthRange, 0.0, 1.0)) : vec3(0.0);
        } else if (cell.y > 0.5) {
          color = texture2D(normalColor, t).rgb;
        } else if (cell.x < 0.5) {
          float w = texture2D(weightedColor, t).a;
          color = d < 1.0 ? mix(vec3(0.08,0.12,0.2), vec3(1.0,0.85,0.3), clamp(w / 6.0, 0.0, 1.0)) : vec3(0.0);
        } else {
          color = finalColor(t);
        }
        if (abs(tex.x - 0.5) < 1.0 / viewport.x || abs(tex.y - 0.5) < 1.0 / viewport.y) color = vec3(0.55);
        gl_FragColor = vec4(color,1);
      }`,
    depthTest: false, depthWrite: false
  });
  const screen = new THREE.Scene();
  screen.add(new THREE.Mesh(new THREE.PlaneGeometry(2,2),composite));
  const screenCamera = new THREE.Camera();
  const clearColor = renderer.getClearColor(new THREE.Color());
  const clearAlpha = renderer.getClearAlpha();
  function renderWith(material, target) {
    mesh.material = material;
    renderer.setRenderTarget(target);
    renderer.render(scene, camera);
    mesh.material = base;
  }
  return {
    canBlend,
    updateLight,
    resize(size) {
      nearest.setSize(size.x,size.y);
      normals.setSize(size.x,size.y);
      sum?.setSize(size.x,size.y);
    },
    // 패스 순서: [그림자 깊이] → 최전면 색·깊이 → [법선] → [가중 누적] → 합성
    draw({blend = false, edl = false, debug = false, shadowMap = false} = {}) {
      base.uniforms.shadowMapOn.value = shadowMap;
      if (shadowMap) renderWith(depthOnly, shadow);
      if (!blend && !edl && !debug) {
        renderer.setRenderTarget(null);
        renderer.render(scene,camera);
        return;
      }
      renderWith(base, nearest);
      if (debug) {
        const mode = base.uniforms.mode.value;
        base.uniforms.mode.value = 1;
        renderWith(base, normals);
        base.uniforms.mode.value = mode;
      }
      if ((blend || debug) && sum) {
        renderer.setClearColor(0,0);
        renderWith(accumulation, sum);
        renderer.setClearColor(clearColor,clearAlpha);
      }
      renderer.setRenderTarget(null);
      composite.uniforms.blend.value = blend && canBlend;
      composite.uniforms.edl.value = edl;
      composite.uniforms.debug.value = debug;
      composite.uniforms.showShadowMap.value = shadowMap;
      renderer.render(screen,screenCamera);
    }
  };
}
