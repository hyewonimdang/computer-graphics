import * as THREE from 'three';
import {OrbitControls} from './node_modules/three/examples/jsm/controls/OrbitControls.js';
import {readPointBuffer} from '../stereo_reconstruction/stream_client/read_points.js';
import {readSurfaceBuffer} from '../stereo_reconstruction/stream_client/read_surface.js';
import {createSplats} from './splats.js?v=paint-arc';

import {createEffects} from './effects.js';

import {spacingRadii} from './spacing.js';

import {createStereoTransition, loadStereoPhotos} from './stereo-transition.js?v=paint-arc';

const stageText = {
  ready: '같은 장면을 찍은 좌우 카메라 사진입니다. 왼쪽부터 점을 펼쳐 보세요.',
  left: '파란 물감이 포물선을 그리며 날아가 맵에 닿고 주변에 튑니다.',
  rightReady: '왼쪽 표면이 만들어졌습니다. 오른쪽 사진의 점을 펼쳐 나머지를 채우세요.',
  right: '분홍 물감이 날아가 남은 표면에 튀고, 마르면서 원래 색이 드러납니다.',
  reveal: '두 사진에서 나온 점들이 하나의 3D 표면으로 이어집니다.',
  complete: '3D 맵 완성 · 드래그로 회전 · 휠로 확대'
};

const element = id => document.getElementById(id);
const wait = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds));

async function playIntro(photos) {
  const status = element('capture-status');
  const enter = element('enter-viewer');
  enter.onclick = () => {
    element('intro').hidden = true;
    const viewer = element('viewer');
    viewer.hidden = false;
    viewer.querySelector('h1').focus();
  };

  if (!photos) {
    status.textContent = '카메라 사진을 찾을 수 없습니다. 복원 화면을 열 수 있습니다.';
    enter.disabled = false;
    return;
  }

  const captures = [element('capture-left'), element('capture-right')];
  const previews = captures.map((capture, i) => {
    const image = capture.querySelector('img');
    image.src = photos.textures[i].image.src;
    return image;
  });
  await Promise.all(previews.map(image => image.decode()));

  const sensorPhoto = element('sensor-photo');
  const cameraReadout = element('sensor-camera');
  const count = element('sensor-count');
  const flash = element('sensor-flash');
  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
  for (let i = 0; i < previews.length; i++) {
    sensorPhoto.src = previews[i].src;
    sensorPhoto.hidden = false;
    cameraReadout.textContent = i === 0 ? 'CAM L · LIVE' : 'CAM R · LIVE';
    status.textContent = i === 0 ? '왼쪽 카메라 조준 중' : '오른쪽 카메라 조준 중';
    await wait(i === 0 ? 920 : 780);
    const shutter = flash.animate([{opacity: reduceMotion ? 0.38 : 1}, {opacity: 0}], {
      duration: reduceMotion ? 130 : 230, easing: 'ease-out'
    });
    captures[i].hidden = false;
    count.textContent = i === 0 ? '01 / 02' : '02 / 02';
    cameraReadout.textContent = i === 0 ? 'CAM L · CAPTURED' : 'CAM R · CAPTURED';
    await shutter.finished;
    if (i === 0) await wait(420);
  }
  status.textContent = '촬영 완료 · 복원할 사진 2장';
  enter.disabled = false;
}

async function load(name, type) {
  const result = await fetch('./data/' + name);
  if (!result.ok) throw Error(name + ' 로드 실패 (' + result.status + ')');
  return result[type]();
}
async function start() {
  const [frame, pointBytes, surfaceBytes, photos] = await Promise.all([
    load('frame.json','json'), load('new_points.bin','arrayBuffer'), load('surface.bin','arrayBuffer'),
    loadStereoPhotos()
  ]);
  const points = readPointBuffer(pointBytes,frame.point_count);
  const surface = readSurfaceBuffer(surfaceBytes);
  const [width,height] = frame.image_size;
  const times = new Float32Array(points.count);
  const mesh = createSplats(points,surface,times,spacingRadii(points,surface,frame));
  const scene = new THREE.Scene();
  scene.add(mesh);
  const canvas = document.querySelector('canvas');
  const renderer = new THREE.WebGLRenderer({canvas,antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setClearColor(0x0b1014);
  const K = frame.K_rect;
  const camera = new THREE.PerspectiveCamera(THREE.MathUtils.radToDeg(2*Math.atan(height/(2*K[1][1]))),width/height,0.05,2000);
  const projection = camera.projectionMatrix.elements;
  projection[0] = 2*K[0][0]/width;
  projection[8] = 1-2*(K[0][2]+0.5)/width;
  projection[9] = 2*(K[1][2]+0.5)/height-1;
  camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
  const flip = new THREE.Matrix4().makeScale(1,-1,-1);
  const pose = new THREE.Matrix4().fromArray(frame.T_world_from_observation_camera_column_major);
  pose.premultiply(flip).multiply(flip).decompose(camera.position,camera.quaternion,camera.scale);
  camera.up.applyQuaternion(camera.quaternion);
  const initialTarget = camera.getWorldDirection(new THREE.Vector3()).multiplyScalar(12).add(camera.position);
  const controls = new OrbitControls(camera,canvas);
  controls.target.copy(initialTarget);
  const uniforms = mesh.material.uniforms;
  const transition = photos && points.count > 0
    ? createStereoTransition(photos, points, frame, mesh, pose, renderer.getPixelRatio())
    : null;
  if (transition) {
    camera.position.copy(transition.overviewPosition);
    controls.target.copy(transition.overviewTarget);
    scene.add(transition.group);
    transition.restart();
    element('left').onclick = () => { transition.startLeft(); updateStage(); };
    element('right').onclick = () => { transition.startRight(); updateStage(); };
    element('restart').onclick = () => { transition.restart(); updateStage(); };
    element('skip').onclick = () => { transition.finish(); updateStage(); };
  } else {
    photos?.textures.forEach(texture => texture.dispose());
  }
  controls.update();
  const overviewPosition = camera.position.clone();
  const overviewTarget = controls.target.clone();
  element('reset').onclick = () => {
    camera.position.copy(overviewPosition);
    controls.target.copy(overviewTarget);
    controls.update();
  };
  element('reset').disabled = false;

  let displayedStage;
  let surfaceAge = 4;
  function updateStage() {
    const stage = transition?.stage ?? 'complete';
    if (stage === displayedStage) return;
    displayedStage = stage;
    const complete = stage === 'complete';
    if (stage === 'reveal') {
      element('mode').value = uniforms.mode.value = 3;
    }
    element('left').disabled = stage !== 'ready';
    element('right').disabled = stage !== 'rightReady';
    element('skip').disabled = complete;
    element('restart').disabled = !transition || stage === 'ready';
    element('stage').textContent = transition ? stageText[stage] :
      points.count === 0 ? '이 프레임에 복원된 점이 없습니다.' :
      '사진 한 쌍을 찾지 못해 3D 맵을 바로 표시합니다.';
  }
  updateStage();
  element('mode').onchange = e => { uniforms.mode.value = Number(e.target.value); };
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
    if (!w) return;
    renderer.setSize(w,Math.round(w*height/width),false);
    canvas.style.height = Math.round(w*height/width)+'px';
    renderer.getDrawingBufferSize(uniforms.viewport.value);
    effects.resize(uniforms.viewport.value);
  };
  canvas.style.maxWidth = 'calc(max(240px, 100svh - 245px) * ' + (width / height) + ')';
  new ResizeObserver(resize).observe(canvas);
  resize();
  let last = performance.now();
  const viewer = element('viewer');
  renderer.setAnimationLoop(now => {
    const delta = THREE.MathUtils.clamp((now - last) / 1000, 0, 0.1);
    last = now;
    if (viewer.hidden) return;
    transition?.tick(delta);
    updateStage();
    surfaceAge += delta;
    uniforms.seconds.value = surfaceAge;
    const phase = now * 0.0006283185 + Number(element('lightOffset').value) * Math.PI / 180;
    uniforms.lightDir.value.set(1.6*Math.sin(phase),0.7,1).normalize();
    if (displayedStage === 'complete') effects.draw(element('blend').checked, element('edl').checked);
    else renderer.render(scene, camera);
  });
  const photoType = transition ? (photos.rectified ? ' · 정류 사진' : ' · 원본 사진') : '';
  element('status').textContent = '점 '+points.count.toLocaleString()+'개 · 유효 법선 '+surface.radii.reduce((sum,r)=>sum+Number(r>0),0).toLocaleString()+'개'+photoType;
  await playIntro(photos);
}
start().catch(error => {
  element('capture-status').textContent = '뷰어를 시작하지 못했습니다. ' + error.message;
  console.error(error);
});
