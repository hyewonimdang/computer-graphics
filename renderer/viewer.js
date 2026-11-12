import * as THREE from 'three';
import {readPointBuffer} from '../stereo_reconstruction/stream_client/read_points.mjs';
import {readSurfaceBuffer} from '../stereo_reconstruction/stream_client/read_surface.mjs';
import {createSplats} from './splats.js';
import {createEffects} from './effects.js';
import {spacingRadii} from './spacing.js';
import {brushStroke, gunStroke} from './paint.js';
import {createOrbitCamera, mat4, poseToViewer} from './camera.js';

const element = id => document.getElementById(id);
async function load(name, type) {
  const result = await fetch('./data/' + name);
  if (!result.ok) throw Error(name + ' 로드 실패 (' + result.status + ')');
  return result[type]();
}
async function start() {
  const [frame, pointBytes, surfaceBytes] = await Promise.all([
    load('frame.json','json'), load('new_points.bin','arrayBuffer'), load('surface.bin','arrayBuffer')
  ]);
  const points = readPointBuffer(pointBytes,frame.point_count);
  const surface = readSurfaceBuffer(surfaceBytes);
  const [width,height] = frame.image_size;
  const times = Float32Array.from(points.pointIds, (_,i) => 5*points.uv[2*i]/width);
  const brush = brushStroke(points,frame);
  const shots = gunStroke(points);
  const mesh = createSplats(points,surface,times,spacingRadii(points,surface,frame),brush);
  // Reserve a single last shot on bright, nearly horizontal foreground road.
  let hero = -1, best = Infinity;
  for (let i=0;i<points.count;i++) {
    if (surface.normals[3*i+1] > -0.93 || points.rgb[3*i]<105) continue;
    const score = Math.hypot(points.uv[2*i]/width-0.54, points.uv[2*i+1]/height-0.84);
    if (score<best) { best=score; hero=i; }
  }
  const heroFlags = new Float32Array(points.count);
  if (hero>=0) {
    heroFlags[hero]=1;
    shots.set([9.5,1.4,0],3*hero);
  }
  mesh.geometry.setAttribute('hero',new THREE.InstancedBufferAttribute(heroFlags,1));
  const scene = new THREE.Scene();
  scene.add(mesh);
  const canvas = document.querySelector('canvas');
  const renderer = new THREE.WebGLRenderer({canvas,antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setClearColor(0x0b1014);

  // B · 카메라. 투영 행렬은 frame.json의 K_rect에서, 초기 포즈는 카메라 포즈에서 직접 계산한다 (camera.js).
  // Three.js에는 행렬을 그대로 복사한 Camera 껍데기만 넘겨 드로 호출에 쓴다.
  const pose = poseToViewer(frame.T_world_from_observation_camera_column_major);
  const orbit = createOrbitCamera(canvas, {K: frame.K_rect, width, height, near: 0.05, far: 2000,
    pose: frame.T_world_from_observation_camera_column_major, lookDistance: 12});
  const camera = new THREE.Camera();
  camera.matrixAutoUpdate = false;
  camera.matrixWorldAutoUpdate = false;
  camera.near = orbit.near; camera.far = orbit.far;
  camera.projectionMatrix.fromArray(orbit.projection);
  camera.projectionMatrixInverse.fromArray(mat4.invert(orbit.projection));
  const uniforms = mesh.material.uniforms;
  uniforms.uProjection.value.set(orbit.projection);
  function applyCamera() {
    const {view, position} = orbit.update();
    uniforms.uView.value.set(view);
    uniforms.uCameraPosition.value.set(position);
    camera.matrixWorldInverse.fromArray(view);
    camera.matrixWorld.fromArray(mat4.invert(view));
  }
  element('reset').onclick = () => orbit.reset();
  if (hero>=0) {
    uniforms.target.value.fromBufferAttribute(mesh.geometry.attributes.center,hero);
    uniforms.planeNormal.value.fromBufferAttribute(mesh.geometry.attributes.surfaceNormal,hero);
  }
  element('shadow').onchange = e => { uniforms.shadowOn.value=e.target.checked; };
  element('shadowDemo').disabled = hero<0;
  element('shadowDemo').onclick = () => {
    element('style').value='gun';
    element('style').dispatchEvent(new Event('change'));
    seek(9.27);
  };
  uniforms.muzzle.value.fromArray(mat4.transformPoint(pose, [0.64,-0.20,-1.0]));
  element('style').onchange = e => {
    uniforms.gun.value = e.target.value === 'gun';
    const stroke = mesh.geometry.getAttribute('stroke');
    stroke.array = uniforms.gun.value ? shots : brush;
    stroke.needsUpdate = true;
    element('paint').checked = true;
    element('paint').onchange({target: element('paint')});
  };
  element('mode').onchange = e => { uniforms.mode.value = Number(e.target.value); };
  element('density').onchange = e => { uniforms.keep.value = Number(e.target.value); };
  const effects = createEffects(renderer, scene, camera, mesh);
  const expand = element('expand'), scale = element('scale');
  function updateScale() {
    const adaptive = element('adaptive').checked;
    expand.disabled = adaptive;
    scale.disabled = adaptive || !expand.checked;
    element('strength').disabled = !adaptive;
    const strength = Number(element('strength').value);
    uniforms.spacingStrength.value = adaptive ? strength/100 : 0;
    element('amount').value = strength+'%';
    uniforms.radiusScale.value = expand.checked && !adaptive ? Number(scale.value) : 1;
    element('factor').value = Number(scale.value).toFixed(1)+'×';
  }
  expand.onchange = scale.oninput = element('adaptive').onchange = element('strength').oninput = updateScale;
  updateScale();
  if (!effects.canBlend) {
    element('blend').disabled = true;
    element('blend').parentElement.title = '이 장치에서는 부동소수점 렌더 타깃을 지원하지 않습니다.';
  }
  const resize = () => {
    const w = canvas.clientWidth;
    renderer.setSize(w,Math.round(w*height/width),false);
    canvas.style.height = Math.round(w*height/width)+'px';
    renderer.getDrawingBufferSize(uniforms.viewport.value);
    effects.resize(uniforms.viewport.value);
  };
  new ResizeObserver(resize).observe(canvas);
  resize();
  let playing = true;
  element('play').onclick = () => {
    if (!playing && uniforms.paint.value && uniforms.seconds.value >= 10) uniforms.seconds.value = 0;
    playing = !playing;
    element('play').textContent = playing ? '일시정지' : '재생';
  };
  function seek(t) {
    playing = false;
    element('play').textContent = '재생';
    uniforms.seconds.value = t;
  }
  element('time').oninput = e => seek(Number(e.target.value));
  element('paint').onchange = e => {
    uniforms.paint.value = e.target.checked;
    if (e.target.checked) {
      uniforms.seconds.value = 0;
      element('mode').value = '3';
      uniforms.mode.value = 3;
      playing = true;
      element('play').textContent = '일시정지';
    }
  };
  for (const button of document.querySelectorAll('[data-time]')) {
    button.onclick = () => {
      element('paint').checked = uniforms.paint.value = true;
      element('mode').value = '3';
      uniforms.mode.value = 3;
      seek(Number(button.dataset.time));
    };
  }
  // URL 쿼리로 컨트롤을 미리 설정한다. 예: ?mode=3&shadowMap=1&debug=1&blend=1&time=4.3 (시연 링크·자동 검증용)
  for (const [key, value] of new URLSearchParams(location.search)) {
    const control = element(key);
    if (!control) continue;
    if (control.type === 'checkbox') control.checked = value !== '0';
    else control.value = value;
    control.dispatchEvent(new Event(control.type === 'range' ? 'input' : 'change'));
    if (key === 'time') seek(Number(value));
  }
  // FPS: 0.5초마다 그 사이의 프레임 수로 갱신
  let frames = 0, fpsSince = performance.now();
  let last = performance.now();
  renderer.setAnimationLoop(now => {
    if (playing) {
      const t = uniforms.seconds.value+Math.max(0,now-last)/1000;
      if (uniforms.paint.value && t >= 10) seek(10);
      else uniforms.seconds.value = t%10;
    }
    last = now;
    element('time').value = uniforms.seconds.value;
    element('clock').value = uniforms.seconds.value.toFixed(2)+'초';
    const phase=uniforms.seconds.value*0.6283185+Number(element('lightOffset').value)*Math.PI/180;
    uniforms.lightDir.value.set(1.6*Math.sin(phase),Number(element('lightHeight').value),1).normalize();
    applyCamera();
    effects.updateLight(uniforms.lightDir.value.toArray());
    effects.draw({
      blend: element('blend').checked, edl: element('edl').checked,
      debug: element('debug').checked, shadowMap: element('shadowMap').checked
    });
    frames++;
    if (now - fpsSince >= 500) {
      element('fps').value = (1000*frames/(now-fpsSince)).toFixed(0)+' FPS';
      frames = 0; fpsSince = now;
    }
  });
  element('status').textContent = '점 '+points.count.toLocaleString()+'개 · 유효 법선 '+surface.radii.reduce((sum,r)=>sum+Number(r>0),0).toLocaleString()+'개';
}
start().catch(error => { element('status').textContent = error.message; console.error(error); });
