import * as THREE from 'three';
<<<<<<< Updated upstream
import {photoPointShader, photoSplit} from './splats.js?v=paint-arc';
=======
import {photoPointShader, photoSplit} from './splats.js';
>>>>>>> Stashed changes

const stageDuration = {left: 5.3, right: 5.3, reveal: 1.25};
const nextStage = {left: 'rightReady', right: 'reveal', reveal: 'complete'};

<<<<<<< Updated upstream
=======
// 보정된 좌우 사진을 먼저 불러오고, 없으면 원본 사진을 불러온다.
>>>>>>> Stashed changes
export async function loadStereoPhotos() {
  const loader = new THREE.TextureLoader();
  for (const suffix of ['_rectified', '']) {
    const results = await Promise.allSettled(
      ['left', 'right'].map(side => loader.loadAsync('./data/' + side + suffix + '.png'))
    );
    if (results.every(result => result.status === 'fulfilled')) {
      return {textures: results.map(result => result.value), rectified: Boolean(suffix)};
    }
    for (const result of results) if (result.status === 'fulfilled') result.value.dispose();
  }
  return null;
}

<<<<<<< Updated upstream
=======
// 촬영 위치에 사진을 놓고 물감이 출발한 자리를 지운다.
>>>>>>> Stashed changes
function makePhotoPanel(photo, side, cameraX, paintBuild) {
  const aspect = photo.image.width / photo.image.height;
  const width = Math.min(5.2, 1.65 * aspect);
  const height = width / aspect;
  const material = new THREE.ShaderMaterial({
    uniforms: {photo: {value: photo}, paintBuild, fromLeft: {value: side < 0}},
    side: THREE.DoubleSide,
    transparent: true,
    depthWrite: false,
    vertexShader: `
      varying vec2 imageUV;
      void main() {
        imageUV = uv;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }`,
    fragmentShader: `
      ${photoPointShader}
      uniform sampler2D photo;
      uniform vec2 paintBuild;
      uniform bool fromLeft;
      varying vec2 imageUV;
      void main() {
        float progress = fromLeft ? paintBuild.x : paintBuild.y;
<<<<<<< Updated upstream
=======
        // 사진 녹이기: 픽셀 높이마다 사라지기 시작하는 시간을 다르게 준다.
>>>>>>> Stashed changes
        float release = 0.065 + 0.31 * imageUV.y;
        float opacity = 1.0 - smoothstep(release + 0.04, release + 0.27, progress);
        if (opacity <= 0.0) discard;
        vec3 source = texture2D(photo, imageUV).rgb;
        vec3 paint = photoPaint(fromLeft) * (0.76 + 0.24 * dot(source, vec3(0.2126, 0.7152, 0.0722)));
        float stain = smoothstep(0.0, 0.075, progress);
        gl_FragColor = vec4(mix(source, paint, stain), opacity);
      }`
  });
  const panel = new THREE.Mesh(new THREE.PlaneGeometry(width, height), material);
  const angle = 0.12;
  // Each inner edge sits above its optical center. The photos extend outwards so they remain readable.
  panel.position.set(cameraX + side * Math.cos(angle) * width / 2,
                     1.15 + height / 2, Math.sin(angle) * width / 2);
  panel.rotation.y = -side * angle;
  panel.updateMatrix();
  return panel;
}

<<<<<<< Updated upstream
=======
// 사진의 픽셀에서 맵의 3D 점까지 날아가는 물감 입자를 만든다.
>>>>>>> Stashed changes
function makePhotoPoints(panel, indices, localCenters, depthOrder, leftUV, rightUV, pixelRatio) {
  const {width, height} = panel.geometry.parameters;
  const {photo, fromLeft, paintBuild} = panel.material.uniforms;
  const starts = new Float32Array(indices.length * 3);
  const ends = new Float32Array(indices.length * 3);
  const sourceUV = new Float32Array(indices.length * 2);
  const dropOrder = new Float32Array(indices.length);
  const point = new THREE.Vector3();
  indices.forEach((index, i) => {
<<<<<<< Updated upstream
=======
    // 입자 출발점: 해당 사진의 픽셀 좌표를 사진 판 위 3D 위치로 바꾼다.
>>>>>>> Stashed changes
    const u = fromLeft.value ? leftUV[2*index] : rightUV[2*index];
    const v = 1 - leftUV[2*index+1];
    point.set((u - 0.5) * width, (v - 0.5) * height, 0.025)
      .applyMatrix4(panel.matrix).toArray(starts, 3*i);
<<<<<<< Updated upstream
=======
    // 입자 도착점: Python에서 미리 복원한 3D 점의 위치를 넣는다.
>>>>>>> Stashed changes
    ends.set(localCenters.subarray(3*index, 3*index+3), 3*i);
    sourceUV.set([u, v], 2*i);
    dropOrder[i] = depthOrder[index];
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(starts, 3));
  geometry.setAttribute('destination', new THREE.BufferAttribute(ends, 3));
  geometry.setAttribute('sourceUV', new THREE.BufferAttribute(sourceUV, 2));
  geometry.setAttribute('depthOrder', new THREE.BufferAttribute(dropOrder, 1));
  const material = new THREE.ShaderMaterial({
    uniforms: {photo, fromLeft, paintBuild, pixelRatio: {value: pixelRatio}},
    transparent: true,
    depthWrite: false,
    vertexShader: `
      ${photoPointShader}
      uniform sampler2D photo;
      uniform vec2 paintBuild;
      uniform bool fromLeft;
      uniform float pixelRatio;
      attribute vec3 destination;
      attribute vec2 sourceUV;
      attribute float depthOrder;
      varying vec3 color;
      varying float opacity, stretch;
      void main() {
        float flight = pointFlight(fromLeft ? paintBuild.x : paintBuild.y, depthOrder);
        if (flight <= 0.0 || flight >= 1.14) {
          gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
          return;
        }
        float variation = dropVariation(sourceUV);
        float t = clamp(flight, 0.0, 1.0);
<<<<<<< Updated upstream
        float arc = 4.0 * t * (1.0 - t);
        float height = mix(1.0, 3.4, variation);
=======
        // 포물선 식: 4t(1-t)는 출발과 도착에서 0이고, 중간에서 1이다.
        float arc = 4.0 * t * (1.0 - t);
        float height = mix(1.0, 3.4, variation);
        // 비행 위치: 직선 이동에 포물선 높이와 약간의 좌우 흔들림을 더한다.
>>>>>>> Stashed changes
        vec3 p = mix(position, destination, t);
        p += arc * vec3((variation - 0.5) * 1.35, height, 0.0);
        vec4 view = modelViewMatrix * vec4(p, 1.0);
        gl_Position = projectionMatrix * view;
        float largeDrop = pow(variation, 5.0);
        float size = mix(38.0, 117.0, largeDrop);
        gl_PointSize = pixelRatio * clamp(size / max(1.0, -view.z), 1.55, 6.4);
        vec3 source = texture2D(photo, sourceUV).rgb;
        vec3 paint = photoPaint(fromLeft);
        color = mix(source, paint, 0.83);
        stretch = smoothstep(0.42, 0.90, t) * (1.0 - smoothstep(1.0, 1.12, flight));
        opacity = mix(0.55, 0.93, largeDrop) * smoothstep(0.0, 0.11, flight)
                  * (1.0 - smoothstep(1.00, 1.14, flight));
      }`,
    fragmentShader: `
      varying vec3 color;
      varying float opacity, stretch;
      void main() {
        vec2 disc = gl_PointCoord * 2.0 - 1.0;
<<<<<<< Updated upstream
=======
        // 물방울 모양: 비행 후반에는 원형 점의 한쪽을 좁혀 꼬리를 만든다.
>>>>>>> Stashed changes
        float tail = 1.0 - smoothstep(-0.80, 0.45, disc.y);
        float width = mix(0.72, mix(0.23, 0.71, 1.0 - tail), stretch);
        vec2 drop = vec2(disc.x / width, (disc.y - 0.12 * stretch) / mix(0.72, 0.88, stretch));
        float edge = 1.0 - smoothstep(0.61, 1.0, dot(drop, drop));
        if (edge <= 0.0) discard;
        gl_FragColor = vec4(color * (0.88 + 0.12 * tail), edge * opacity);
      }`
  });
  const cloud = new THREE.Points(geometry, material);
  cloud.frustumCulled = false;
  cloud.renderOrder = 1;
  return cloud;
}

<<<<<<< Updated upstream
function makeImpactSplashes(panel, indices, localCenters, depthOrder, normals, toCamera, pixelRatio) {
=======
// 물감이 맵에 닿았을 때 주변으로 튀는 작은 방울을 만든다.
function makeImpactSplashes(panel, indices, localCenters, depthOrder, normals, toCamera, pixelRatio) {
  // 착지 효과: 일부 점만 골라 작은 방울을 5개씩 만든다.
>>>>>>> Stashed changes
  const stride = Math.max(13, Math.ceil(indices.length / 2400));
  const flecksPerImpact = 5;
  const count = Math.ceil(indices.length / stride) * flecksPerImpact;
  const positions = new Float32Array(count * 3);
  const directions = new Float32Array(count * 3);
  const timing = new Float32Array(count * 2);
  const normal = new THREE.Vector3();
  const point = new THREE.Vector3();
  let j = 0;
  for (let i = 0; i < indices.length; i += stride) {
    const index = indices[i];
    point.fromArray(localCenters, index * 3);
    normal.fromBufferAttribute(normals, index);
    if (normal.lengthSq() < 0.01) normal.set(0, 1, 0);
    else normal.transformDirection(toCamera);
    if (normal.dot(point) > 0) normal.negate();
    for (let fleck = 0; fleck < flecksPerImpact; fleck++, j++) {
      point.toArray(positions, j * 3);
      normal.toArray(directions, j * 3);
      timing[j * 2] = depthOrder[index];
      timing[j * 2 + 1] = ((index * 37 + fleck * 211) % 997) / 997;
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('impactNormal', new THREE.BufferAttribute(directions, 3));
  geometry.setAttribute('timing', new THREE.BufferAttribute(timing, 2));
  const {fromLeft, paintBuild} = panel.material.uniforms;
  const material = new THREE.ShaderMaterial({
    uniforms: {fromLeft, paintBuild, pixelRatio: {value: pixelRatio}},
    transparent: true,
    depthWrite: false,
    vertexShader: `
      ${photoPointShader}
      uniform vec2 paintBuild;
      uniform bool fromLeft;
      uniform float pixelRatio;
      attribute vec3 impactNormal;
      attribute vec2 timing;
      varying vec3 color;
      varying float opacity;
      void main() {
<<<<<<< Updated upstream
=======
        // 튀는 시점: 비행 진행률에서 1을 빼서 착지 후에만 방울을 그린다.
>>>>>>> Stashed changes
        float impactAge = pointFlight(fromLeft ? paintBuild.x : paintBuild.y, timing.x) - 1.0;
        float seed = timing.y;
        float t = impactAge / mix(0.19, 0.34, seed);
        if (t <= 0.0 || t >= 1.0) {
          gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
          return;
        }
        vec3 axis = abs(impactNormal.y) < 0.90 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
<<<<<<< Updated upstream
=======
        // 튀는 방향: 표면과 나란한 방향을 만들고 방울마다 다른 각도로 퍼뜨린다.
>>>>>>> Stashed changes
        vec3 tangent = normalize(cross(axis, impactNormal));
        float angle = seed * 6.283185;
        vec3 radial = cos(angle) * tangent + sin(angle) * cross(impactNormal, tangent);
        vec3 p = position + impactNormal * (0.04 + 4.0 * t * (1.0 - t) * mix(0.12, 0.56, seed));
        p += radial * mix(0.32, 1.05, seed) * t * (1.55 - 0.55 * t);
        vec4 view = modelViewMatrix * vec4(p, 1.0);
        gl_Position = projectionMatrix * view;
        gl_PointSize = pixelRatio * clamp(mix(36.0, 110.0, seed) / max(1.0, -view.z), 1.6, 6.2);
        color = photoPaint(fromLeft) * mix(0.83, 1.1, seed);
        opacity = 0.87 * smoothstep(0.0, 0.13, t) * (1.0 - smoothstep(0.48, 1.0, t));
      }`,
    fragmentShader: `
      varying vec3 color;
      varying float opacity;
      void main() {
        vec2 point = gl_PointCoord * 2.0 - 1.0;
        float edge = 1.0 - smoothstep(0.48, 1.0, dot(point, point));
        if (edge <= 0.0) discard;
        gl_FragColor = vec4(color, opacity * edge);
      }`
  });
  const splash = new THREE.Points(geometry, material);
  splash.frustumCulled = false;
  splash.renderOrder = 2;
  return splash;
}

<<<<<<< Updated upstream
=======
// 사진과 입자를 준비하고 좌우 연출의 재생 순서를 정한다.
>>>>>>> Stashed changes
export function createStereoTransition(photos, points, frame, mesh, pose, pixelRatio) {
  const group = new THREE.Group();
  group.applyMatrix4(pose);
  const {paintBuild} = mesh.material.uniforms;

  const [imageWidth, imageHeight] = frame.image_size;
  const leftUV = new Float32Array(points.count * 2);
  const rightUV = new Float32Array(points.count * 2);
  const photoIndices = [[], []];
  for (let i = 0; i < points.count; i++) {
    const u = points.uv[2*i] + 0.5;
    const row = (points.uv[2*i+1] + 0.5) / imageHeight;
    leftUV[2*i] = THREE.MathUtils.clamp(u / imageWidth, 0, 1);
    leftUV[2*i+1] = row;
<<<<<<< Updated upstream
=======
    // 오른쪽 사진 좌표: 왼쪽의 가로 픽셀 위치에서 시차를 뺀다.
>>>>>>> Stashed changes
    rightUV[2*i] = THREE.MathUtils.clamp((u - points.disparity[i]) / imageWidth, 0, 1);
    rightUV[2*i+1] = 1 - row;
    photoIndices[leftUV[2*i] < Math.fround(photoSplit) ? 0 : 1].push(i);
  }
  mesh.geometry.setAttribute('leftPhotoUV', new THREE.InstancedBufferAttribute(leftUV, 2));

  const toCamera = pose.clone().invert();
  const centers = mesh.geometry.getAttribute('center');
  const normals = mesh.geometry.getAttribute('surfaceNormal');
  const localCenters = new Float32Array(points.count * 3);
  const depthOrder = new Float32Array(points.count);
  const point = new THREE.Vector3();
  const depths = [];
  const stride = Math.max(1, Math.ceil(points.count / 2048));
  let near = Infinity;
  let far = -Infinity;
  for (let i = 0; i < points.count; i++) {
    point.fromBufferAttribute(centers, i).applyMatrix4(toCamera);
    point.toArray(localCenters, 3*i);
    const cameraDepth = -localCenters[3*i+2];
    near = Math.min(near, cameraDepth);
    far = Math.max(far, cameraDepth);
    if (i % stride === 0 && cameraDepth > 0) depths.push(cameraDepth);
  }
  const depthRange = far - near || 1;
  for (let i = 0; i < points.count; i++) {
<<<<<<< Updated upstream
=======
    // 깊이 순서: 카메라 기준으로 가장 가까운 점은 0, 가장 먼 점은 1이 된다.
>>>>>>> Stashed changes
    depthOrder[i] = (-localCenters[3*i+2] - near) / depthRange;
  }
  mesh.geometry.setAttribute('depthOrder', new THREE.InstancedBufferAttribute(depthOrder, 1));
  depths.sort((a, b) => a - b);
  const depth = depths[Math.floor(depths.length * 0.40)] ?? 12;
  const scale = THREE.MathUtils.clamp(depth / 18, 0.45, 2.5);
  const baseline = Math.max(0, frame.baseline_m ?? 0);
  photos.textures.forEach((photo, i) => {
    const panel = makePhotoPanel(photo, i === 0 ? -1 : 1, i * baseline, paintBuild);
    const indices = photoIndices[i];
    group.add(panel, makePhotoPoints(panel, indices, localCenters, depthOrder, leftUV, rightUV, pixelRatio),
              makeImpactSplashes(panel, indices, localCenters, depthOrder, normals, toCamera, pixelRatio));
  });
  const overviewPosition = new THREE.Vector3(baseline / 2, 2.15, 10.2).applyMatrix4(pose);
  const overviewTarget = new THREE.Vector3(baseline / 2, 0.95 * scale, -depth).applyMatrix4(pose);
  let stage = 'ready';
  let elapsed = 0;

<<<<<<< Updated upstream
=======
  // 현재 재생 단계에 맞춰 사진과 맵의 표시 상태를 바꾼다.
>>>>>>> Stashed changes
  function update() {
    const progress = elapsed / (stageDuration[stage] ?? 1);
    const left = stage === 'ready' ? 0 : stage === 'left' ? progress : 1;
    const right = stage === 'right' ? progress : stage === 'reveal' || stage === 'complete' ? 1 : 0;
    const complete = stage === 'complete';
    paintBuild.value.set(left, right);
    group.visible = !complete;
    mesh.visible = left > 0 || right > 0;
  }

  return {
    group,
    overviewPosition,
    overviewTarget,
<<<<<<< Updated upstream
    get stage() { return stage; },
    startLeft() {
      if (stage === 'ready') { stage = 'left'; elapsed = 0; }
    },
    startRight() {
      if (stage === 'rightReady') { stage = 'right'; elapsed = 0; }
    },
=======
    // 현재 연출 단계를 알려 준다.
    get stage() { return stage; },
    // 준비 상태에서 왼쪽 사진 연출을 시작한다.
    startLeft() {
      if (stage === 'ready') { stage = 'left'; elapsed = 0; }
    },
    // 왼쪽 연출이 끝나면 오른쪽 사진 연출을 시작한다.
    startRight() {
      if (stage === 'rightReady') { stage = 'right'; elapsed = 0; }
    },
    // 연출을 사진 두 장이 보이는 처음 상태로 되돌린다.
>>>>>>> Stashed changes
    restart() {
      stage = 'ready';
      elapsed = 0;
      update();
    },
<<<<<<< Updated upstream
=======
    // 남은 연출을 건너뛰고 완성된 맵을 보여 준다.
>>>>>>> Stashed changes
    finish() {
      stage = 'complete';
      elapsed = 0;
      update();
    },
<<<<<<< Updated upstream
=======
    // 지난 시간을 더하고, 현재 단계가 끝나면 다음 단계로 넘어간다.
>>>>>>> Stashed changes
    tick(delta) {
      if (!stageDuration[stage]) return;
      elapsed += delta;
      if (elapsed >= stageDuration[stage]) {
        stage = nextStage[stage];
        elapsed = 0;
      }
      update();
    }
  };
}

