import * as THREE from 'three';
import {cameraToViewer} from '../stereo_reconstruction/stream_client/read_points.js';

export const photoSplit = 0.53;
export const photoPointShader = `
  vec3 photoPaint(bool fromLeft) {
    return fromLeft ? vec3(0.15, 0.47, 0.92) : vec3(0.88, 0.23, 0.53);
  }
  float dropVariation(vec2 imageUV) {
    return fract(sin(dot(imageUV, vec2(127.1, 311.7))) * 43758.5453);
  }
  float pointFlight(float progress, float depthOrder) {
    return (progress - (0.065 + 0.31 * depthOrder)) / 0.36;
  }
`;

export const photoSplit = 0.53;
export const photoPointShader = `
  // 왼쪽 사진에는 파란색, 오른쪽 사진에는 분홍색을 준다.
  vec3 photoPaint(bool fromLeft) {
    return fromLeft ? vec3(0.15, 0.47, 0.92) : vec3(0.88, 0.23, 0.53);
  }
  // 사진 속 위치마다 다른 값을 만들어 입자 움직임에 차이를 준다.
  float dropVariation(vec2 imageUV) {
    return fract(sin(dot(imageUV, vec2(127.1, 311.7))) * 43758.5453);
  }
  // 카메라에 가까운 점부터 출발하도록 각 입자의 진행률을 계산한다.
  float pointFlight(float progress, float depthOrder) {
    // 출발 시간: 깊이 값이 클수록 늦게 출발한다. 결과가 1이면 착지다.
    return (progress - (0.065 + 0.31 * depthOrder)) / 0.36;
  }
`;

const shotPath = `
  // 출발점과 도착점 사이에서 포물선을 따라 움직이는 위치를 계산한다.
  vec3 projectile(vec3 origin, vec3 targetPoint, float t) {
    // 포물선 식: 두 점 사이의 직선 위치에 y축으로 1.4t(1-t)를 더한다.
    return mix(origin,targetPoint,t)+vec3(0.0,1.4*t*(1.0-t),0.0);
  }`;

<<<<<<< Updated upstream
=======
// 그림자 맵 샘플링. 광원 시점 깊이와 비교해 0(그늘)~1(빛) 가시성을 돌려준다. 3×3 PCF로 경계를 부드럽게 한다.
const shadowLookup = `
  uniform sampler2D shadowMap;
  uniform mat4 uLightVP;
  uniform bool shadowMapOn;
  uniform vec2 shadowSize;
  uniform float shadowBias;
  float shadowVisibility(vec3 worldPosition, float NdotL) {
    if (!shadowMapOn) return 1.0;
    vec4 lightClip = uLightVP * vec4(worldPosition, 1.0);
    vec3 s = lightClip.xyz / lightClip.w * 0.5 + 0.5;
    if (s.x < 0.0 || s.x > 1.0 || s.y < 0.0 || s.y > 1.0 || s.z > 1.0) return 1.0;
    float bias = shadowBias * (1.5 - NdotL);
    float lit = 0.0;
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        float stored = texture2D(shadowMap, s.xy + vec2(float(dx), float(dy)) / shadowSize).r;
        lit += step(s.z - bias, stored);
      }
    }
    return lit / 9.0;
  }`;

export const vertexShader = `
  precision highp float;
  precision highp int;
  uniform mat4 uProjection, uView;
  uniform vec3 uCameraPosition;
  attribute vec3 position;
  attribute vec3 stroke;
  attribute float hero, lot;
  attribute vec2 leftPhotoUV;
  attribute float depthOrder;
  uniform vec2 paintBuild;
  ${photoPointShader}
  ${shotPath}
  uniform bool paint, gun;
  uniform vec3 muzzle;
  uniform float seconds, keep;
  attribute vec3 center, surfaceNormal, pigment;
  attribute float radius, landedAt;
  uniform int mode;
  uniform vec2 viewport;
  uniform float radiusScale, spacingStrength;
  attribute float spacingRadius;
  varying vec2 disc;
  varying vec3 rgb, n, eye;
  varying float valid, arrival, ink, wetPhoto;
  void main() {
    // 점 개수 비교용 다운샘플: point_id 해시가 keep보다 크면 화면 밖으로 보낸다.
    if (lot > keep) { gl_Position = vec4(2,2,2,1); return; }
    bool fromLeft = leftPhotoUV.x < ${photoSplit};
    float flight = pointFlight(fromLeft ? paintBuild.x : paintBuild.y, depthOrder);
    // 표면 드러내기: 입자가 도착하기 전에는 해당 3D 점을 숨긴다.
    float coverage = smoothstep(1.0, 1.12, flight);
    if (coverage <= 0.0) { gl_Position = vec4(2,2,2,1); return; }
    disc = position.xy;
    // 물감 건조: 착지 후 파란색이나 분홍색의 비중을 줄여 원래 RGB로 돌아간다.
    wetPhoto = 1.0 - smoothstep(1.18, 1.72, flight);
    rgb = mix(pigment, photoPaint(fromLeft), 0.88 * wetPhoto);
    arrival = landedAt;
    valid = float(radius > 0.0);
    vec3 p = center;
    ink = float(paint && stroke.y > 0.0);
    bool flying = false;
    float spread = 1.0 + 0.65 * wetPhoto * smoothstep(1.0, 1.13, flight)
                 * (1.0 - smoothstep(1.14, 1.47, flight));
    if (ink > 0.5) {
      arrival = stroke.x;
      float age = seconds-arrival;
      float t = 1.0+age/stroke.y;
      if (t < 0.0) { gl_Position = vec4(2,2,2,1); return; }
      flying = age < 0.0;
      if (flying) {
        if (gun) {
          p = projectile(muzzle, center, t);
          rgb = mix(rgb,vec3(1.0,0.78,0.42),0.45);
        } else p += vec3(-stroke.z*(1.0-t), (1.0-t)*0.08+0.24*t*(1.0-t),0.0);
        valid = 0.0;
      } else if (gun) {
        spread = (hero>0.5 ? 2.0 : 1.0)*(0.45 + 2.15*smoothstep(0.0,0.10,age)
                      - 1.60*smoothstep(0.10,0.48,age));
      } else {
        spread = 0.30 + 0.98*smoothstep(0.0,0.18,age)
                      - 0.28*smoothstep(0.18,0.48,age);
      }
    }
    if (!flying && mode != 0 && radius > 0.0) {
      // 법선에 수직인 접평면 기저(tangent, bitangent)를 만들고 미터 단위 원판을 펼친다.
      vec3 normalDir = normalize(surfaceNormal);
      vec3 axis = abs(normalDir.y) < 0.9 ? vec3(0,1,0) : vec3(1,0,0);
      vec3 tangent = normalize(cross(axis,normalDir));
      p += coverage * spread * mix(radiusScale * radius, spacingRadius, spacingStrength) * (disc.x*tangent + disc.y*cross(normalDir,tangent));
    }
    eye = uCameraPosition - p;
    n = surfaceNormal;
    gl_Position = uProjection * uView * vec4(p,1);
    if (flying || mode == 0 || radius == 0.0)
      gl_Position.xy += coverage * disc * (flying ? (gun ? (hero>0.5 ? 17.0 : 6.0) : 2.0) : (gun && ink > 0.5 ? 2.5*spread : 2.5)) / viewport * gl_Position.w;
  }`;

export const fragmentShader = `
  precision highp float;
  precision highp int;
  uniform vec3 uCameraPosition;
  ${shotPath}
  ${shadowLookup}
  uniform bool paint, gun, shadowOn;
  uniform vec3 muzzle,target,planeNormal,lightDir;
  uniform int mode;
  uniform float seconds;
  varying vec2 disc;
  varying vec3 rgb, n, eye;
  varying float valid, arrival, ink, wetPhoto;
  void main() {
    if (dot(disc,disc) > 1.0) discard;
    vec3 color = rgb;
    if (valid > 0.5 && mode == 1) color = normalize(n)*0.5+0.5;
    if (valid > 0.5 && mode == 3) {
      // Blinn-Phong. 젖은 페인트(wet)는 광택 지수와 정반사 강도가 높고 시간이 지나면 무광으로 마른다.
      vec3 N = normalize(n);
      vec3 L = lightDir;
      float diffuse = max(dot(N,L),0.0);
      float visibility = shadowVisibility(uCameraPosition - eye, diffuse);
      diffuse *= visibility;
      float wet = step(arrival,seconds)*(1.0-smoothstep(0.0,3.0,seconds-arrival));
      vec3 H = normalize(L+normalize(eye));
      float shine = diffuse > 0.0 ? pow(max(dot(N,H),0.0),mix(8.0,64.0,wet)) : 0.0;
      // 그늘에서는 주변광도 절반으로 줄여 그림자가 분명히 보이게 한다.
      float ambient = mix(0.34, 0.68, wetPhoto) * mix(0.45, 1.0, visibility);
      vec3 linearRGB = pow(rgb,vec3(2.2))*(ambient + mix(0.8, 0.30, wetPhoto)*diffuse) + shine*mix(0.03,0.7,wet)*visibility*(1.0-wetPhoto);
      if (ink > 0.5) {
        float grazing = pow(1.0-clamp(dot(N,normalize(eye)),0.0,1.0),3.0);
        linearRGB += vec3(0.45,0.58,0.65)*wet*grazing;
      }
      color = pow(linearRGB,vec3(1.0/2.2));
    }
    if (shadowOn && paint && gun && valid>0.5 && seconds>=8.1 && seconds<=9.5) {
      float incidence=dot(planeNormal,lightDir);
      if (incidence>0.05 && dot(n,planeNormal)>0.75) {
        float t=clamp((seconds-8.1)/1.4,0.0,1.0);
        vec3 ball=projectile(muzzle,target,t);
        float travel=dot(ball-target,planeNormal)/incidence;
        vec3 hit=ball-travel*lightDir;
        vec3 delta=(uCameraPosition-eye)-hit;
        float height=dot(delta,planeNormal);
        if (travel>=0.0 && abs(height)<0.18) {
          float distanceOnPlane=length(delta-height*planeNormal);
          float softness=0.045+min(travel*0.085,0.18);
          color*=mix(0.25,1.0,smoothstep(0.13,0.13+softness,distanceOnPlane));
        }
      }
    }
    gl_FragColor = vec4(color,1);
  }`;

// 점별 균일 난수 [0,1). point_id 기반이라 재생마다 같은 부분집합이 선택된다.
function lots(pointIds) {
  return Float32Array.from(pointIds, id => ((Math.imul(id + 1, 0x9E3779B1) >>> 0) / 4294967296));
}

>>>>>>> Stashed changes
// Geometry is a metric disc in the tangent plane. Invalid normals use a screen-space dot.
export function createSplats(points, surface, landingTimes, spacing = surface.radii, stroke = new Float32Array(points.count*3)) {
  if (points.count !== surface.count || landingTimes.length !== points.count ||
      points.pointIds.some((id, i) => id !== surface.pointIds[i])) {
    throw Error('점과 표면 데이터의 개수 또는 point_id가 일치하지 않습니다.');
  }
  const geometry = new THREE.InstancedBufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute([-1,-1,0, 1,-1,0, -1,1,0, 1,1,0], 3));
  geometry.setIndex([0,1,2, 2,1,3]);
  geometry.instanceCount = points.count;
  function attribute(name, data, size, normalized = false) {
    geometry.setAttribute(name, new THREE.InstancedBufferAttribute(data, size, normalized));
  }
  attribute('stroke',stroke,3);
  attribute('center', cameraToViewer(points.positions), 3);
  attribute('surfaceNormal', cameraToViewer(surface.normals), 3);
  attribute('radius', surface.radii, 1);
  attribute('spacingRadius', spacing, 1);
  attribute('pigment', points.rgb, 3, true);
  attribute('landedAt', landingTimes, 1);
<<<<<<< Updated upstream
  const material = new THREE.ShaderMaterial({
    side: THREE.DoubleSide,
    uniforms: {target: {value:new THREE.Vector3()}, planeNormal:{value:new THREE.Vector3()},
      shadowOn:{value:true}, lightDir:{value:new THREE.Vector3()}, gun: {value: false}, muzzle: {value: new THREE.Vector3()}, paint: {value: false}, spacingStrength: {value: 0}, radiusScale: {value: 1}, paintBuild: {value: new THREE.Vector2(1, 1)}, mode: {value: 3}, seconds: {value: 0}, viewport: {value: new THREE.Vector2()}},
    vertexShader: `
      attribute vec3 stroke;
      attribute float hero;
      ${shotPath}
      ${photoPointShader}
      uniform bool paint, gun;
      uniform vec3 muzzle;
      uniform float seconds;
      attribute vec3 center, surfaceNormal, pigment;
      attribute float radius, landedAt;
      uniform int mode;
      uniform vec2 viewport;
      uniform float radiusScale, spacingStrength;
      uniform vec2 paintBuild;
      attribute float spacingRadius, depthOrder;
      attribute vec2 leftPhotoUV;
      varying vec2 disc;
      varying vec3 rgb, n, eye;
      varying float valid, arrival, ink, wetPhoto;
      void main() {
        bool fromLeft = leftPhotoUV.x < ${photoSplit};
        float flight = pointFlight(fromLeft ? paintBuild.x : paintBuild.y, depthOrder);
        float coverage = smoothstep(1.0, 1.12, flight);
        if (coverage <= 0.0) { gl_Position = vec4(2,2,2,1); return; }
        disc = position.xy;
        wetPhoto = 1.0 - smoothstep(1.18, 1.72, flight);
        rgb = mix(pigment, photoPaint(fromLeft), 0.88 * wetPhoto);
        arrival = landedAt;
        valid = float(radius > 0.0);
        vec3 p = center;
        ink = float(paint && stroke.y > 0.0);
        bool flying = false;
        float spread = 1.0 + 0.65 * wetPhoto * smoothstep(1.0, 1.13, flight)
                     * (1.0 - smoothstep(1.14, 1.47, flight));
        if (ink > 0.5) {
          arrival = stroke.x;
          float age = seconds-arrival;
          float t = 1.0+age/stroke.y;
          if (t < 0.0) { gl_Position = vec4(2,2,2,1); return; }
          flying = age < 0.0;
          if (flying) {
            if (gun) {
              p = projectile(muzzle, center, t);
              rgb = mix(rgb,vec3(1.0,0.78,0.42),0.45);
            } else p += vec3(-stroke.z*(1.0-t), (1.0-t)*0.08+0.24*t*(1.0-t),0.0);
            valid = 0.0;
          } else if (gun) {
            spread = (hero>0.5 ? 2.0 : 1.0)*(0.45 + 2.15*smoothstep(0.0,0.10,age)
                          - 1.60*smoothstep(0.10,0.48,age));
          } else {
            spread = 0.30 + 0.98*smoothstep(0.0,0.18,age)
                          - 0.28*smoothstep(0.18,0.48,age);
          }
        }
        if (!flying && mode != 0 && radius > 0.0) {
          vec3 normalDir = normalize(surfaceNormal);
          vec3 axis = abs(normalDir.y) < 0.9 ? vec3(0,1,0) : vec3(1,0,0);
          vec3 tangent = normalize(cross(axis,normalDir));
          p += coverage * spread * mix(radiusScale * radius, spacingRadius, spacingStrength) * (disc.x*tangent + disc.y*cross(normalDir,tangent));
        }
        eye = cameraPosition - p;
        n = surfaceNormal;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(p,1);
        if (flying || mode == 0 || radius == 0.0)
          gl_Position.xy += coverage * disc * (flying ? (gun ? (hero>0.5 ? 17.0 : 6.0) : 2.0) : (gun && ink > 0.5 ? 2.5*spread : 2.5)) / viewport * gl_Position.w;
      }`,
    fragmentShader: `
      ${shotPath}
      uniform bool paint, gun, shadowOn;
      uniform vec3 muzzle,target,planeNormal,lightDir;
      uniform int mode;
      uniform float seconds;
      varying vec2 disc;
      varying vec3 rgb, n, eye;
      varying float valid, arrival, ink, wetPhoto;
      void main() {
        if (dot(disc,disc) > 1.0) discard;
        vec3 color = rgb;
        if (valid > 0.5 && mode == 1) color = normalize(n)*0.5+0.5;
        if (valid > 0.5 && mode == 3) {
          vec3 N = normalize(n);
          vec3 L = lightDir;
          float diffuse = max(dot(N,L),0.0);
          float wet = step(arrival,seconds)*(1.0-smoothstep(0.0,3.0,seconds-arrival));
          vec3 H = normalize(L+normalize(eye));
          float shine = diffuse > 0.0 ? pow(max(dot(N,H),0.0),mix(8.0,64.0,wet)) : 0.0;
          float light = mix(0.34 + 0.8 * diffuse, 0.68 + 0.30 * diffuse, wetPhoto);
          vec3 linearRGB = pow(rgb,vec3(2.2)) * light + shine * mix(0.03, 0.7, wet) * (1.0 - wetPhoto);
          if (ink > 0.5) {
            float grazing = pow(1.0-clamp(dot(N,normalize(eye)),0.0,1.0),3.0);
            linearRGB += vec3(0.45,0.58,0.65)*wet*grazing;
          }
          color = pow(linearRGB,vec3(1.0/2.2));
        }
        if (shadowOn && paint && gun && valid>0.5 && seconds>=8.1 && seconds<=9.5) {
          float incidence=dot(planeNormal,lightDir);
          if (incidence>0.05 && dot(n,planeNormal)>0.75) {
            float t=clamp((seconds-8.1)/1.4,0.0,1.0);
            vec3 ball=projectile(muzzle,target,t);
            float travel=dot(ball-target,planeNormal)/incidence;
            vec3 hit=ball-travel*lightDir;
            vec3 delta=(cameraPosition-eye)-hit;
            float height=dot(delta,planeNormal);
            if (travel>=0.0 && abs(height)<0.18) {
              float distanceOnPlane=length(delta-height*planeNormal);
              float softness=0.045+min(travel*0.085,0.18);
              color*=mix(0.25,1.0,smoothstep(0.13,0.13+softness,distanceOnPlane));
            }
          }
        }
        gl_FragColor = vec4(color,1);
      }`
=======
  attribute('leftPhotoUV', new Float32Array(points.count * 2), 2);
  attribute('depthOrder', new Float32Array(points.count), 1);
  attribute('lot', lots(points.pointIds), 1);
  attribute('hero', new Float32Array(points.count), 1);
  const material = new THREE.RawShaderMaterial({
    side: THREE.DoubleSide,
    uniforms: {
      uProjection: {value: new Float32Array(16)}, uView: {value: new Float32Array(16)}, uCameraPosition: {value: new Float32Array(3)},
      uLightVP: {value: new Float32Array(16)}, shadowMap: {value: null}, shadowMapOn: {value: false},
      shadowSize: {value: new THREE.Vector2(2048, 2048)}, shadowBias: {value: 0.0004},
      keep: {value: 1}, paintBuild: {value: new THREE.Vector2(1, 1)},
      target: {value:new THREE.Vector3()}, planeNormal:{value:new THREE.Vector3()},
      shadowOn:{value:true}, lightDir:{value:new THREE.Vector3()}, gun: {value: false}, muzzle: {value: new THREE.Vector3()}, paint: {value: false}, spacingStrength: {value: 0}, radiusScale: {value: 1}, mode: {value: 3}, seconds: {value: 0}, viewport: {value: new THREE.Vector2()}},
    vertexShader,
    fragmentShader
>>>>>>> Stashed changes
  });
  material.defaultAttributeValues.leftPhotoUV = [0.5, 0.5];
  material.defaultAttributeValues.depthOrder = [0];
  const mesh = new THREE.Mesh(geometry, material);
  mesh.frustumCulled = false; // The base quad bounds exclude instance centers.
  return mesh;
}
