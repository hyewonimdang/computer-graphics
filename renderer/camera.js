// B · 직접 구현한 변환 행렬과 오빗 카메라.
// Three.js의 Camera/Matrix4/OrbitControls를 쓰지 않는다. 행렬은 WebGL 규약대로 열 우선(column-major) Float32Array(16)이다.
// 뷰어 좌표계: +X 오른쪽, +Y 위, 카메라는 -Z를 바라본다. (A의 카메라 좌표 (X,Y,Z)→(X,−Y,−Z) 변환 결과)

export const mat4 = {
  identity() {
    const m = new Float32Array(16);
    m[0] = m[5] = m[10] = m[15] = 1;
    return m;
  },
  // out = a × b (열 우선 저장. 점 p에 대해 (a×b)p = a(bp))
  multiply(a, b, out = new Float32Array(16)) {
    for (let col = 0; col < 4; col++) {
      for (let row = 0; row < 4; row++) {
        let sum = 0;
        for (let k = 0; k < 4; k++) sum += a[row + 4 * k] * b[k + 4 * col];
        out[row + 4 * col] = sum;
      }
    }
    return out;
  },
  // 여인수 전개로 일반 역행렬. 특이 행렬이면 null.
  invert(m, out = new Float32Array(16)) {
    const [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = m;
    const b00 = a00 * a11 - a01 * a10, b01 = a00 * a12 - a02 * a10, b02 = a00 * a13 - a03 * a10;
    const b03 = a01 * a12 - a02 * a11, b04 = a01 * a13 - a03 * a11, b05 = a02 * a13 - a03 * a12;
    const b06 = a20 * a31 - a21 * a30, b07 = a20 * a32 - a22 * a30, b08 = a20 * a33 - a23 * a30;
    const b09 = a21 * a32 - a22 * a31, b10 = a21 * a33 - a23 * a31, b11 = a22 * a33 - a23 * a32;
    let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if (!det) return null;
    det = 1 / det;
    out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
    out[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
    out[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
    out[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
    out[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
    out[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
    out[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
    out[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
    out[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
    out[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
    out[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
    out[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
    out[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
    out[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
    out[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
    out[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;
    return out;
  },
  // 정류 카메라 내부 행렬 K(fx, fy, cx, cy)에서 OpenGL 투영 행렬을 만든다.
  // 픽셀 (u,v)는 셀 중심이 +0.5인 규약, v는 아래로 증가하므로 y를 뒤집어 NDC +y가 위가 되게 한다.
  // 깊이는 표준 OpenGL 규약(-(f+n)/(f-n), -2fn/(f-n))이라 하드웨어 depth → 거리 복원이 effects.js와 맞는다.
  perspectiveFromIntrinsics(K, width, height, near, far) {
    const fx = K[0][0], fy = K[1][1], cx = K[0][2], cy = K[1][2];
    const m = new Float32Array(16);
    m[0] = 2 * fx / width;
    m[5] = 2 * fy / height;
    m[8] = 1 - 2 * (cx + 0.5) / width;
    m[9] = 2 * (cy + 0.5) / height - 1;
    m[10] = -(far + near) / (far - near);
    m[11] = -1;
    m[14] = -2 * far * near / (far - near);
    return m;
  },
  // 직교 투영. 그림자 맵의 광원 카메라에 쓴다.
  orthographic(left, right, bottom, top, near, far) {
    const m = new Float32Array(16);
    m[0] = 2 / (right - left);
    m[5] = 2 / (top - bottom);
    m[10] = -2 / (far - near);
    m[12] = -(right + left) / (right - left);
    m[13] = -(top + bottom) / (top - bottom);
    m[14] = -(far + near) / (far - near);
    m[15] = 1;
    return m;
  },
  // eye에서 target을 바라보는 뷰 행렬(월드 → 카메라). 카메라는 -Z를 본다.
  lookAt(eye, target, up) {
    const z = normalize(sub(eye, target));          // 카메라 +Z = 시선의 반대
    const x = normalize(cross(up, z));              // 카메라 +X
    const y = cross(z, x);                          // 카메라 +Y
    const m = new Float32Array(16);
    m[0] = x[0]; m[4] = x[1]; m[8] = x[2];  m[12] = -dot(x, eye);
    m[1] = y[0]; m[5] = y[1]; m[9] = y[2];  m[13] = -dot(y, eye);
    m[2] = z[0]; m[6] = z[1]; m[10] = z[2]; m[14] = -dot(z, eye);
    m[15] = 1;
    return m;
  },
  // 동차 좌표 변환 후 w로 나눈 3차원 점
  transformPoint(m, p) {
    const x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
    const y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
    const z = m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14];
    const w = m[3] * p[0] + m[7] * p[1] + m[11] * p[2] + m[15] || 1;
    return [x / w, y / w, z / w];
  },
  // 이동 성분을 뺀 방향 변환
  transformDirection(m, d) {
    return [
      m[0] * d[0] + m[4] * d[1] + m[8] * d[2],
      m[1] * d[0] + m[5] * d[1] + m[9] * d[2],
      m[2] * d[0] + m[6] * d[1] + m[10] * d[2]
    ];
  }
};

export const vec3 = {sub, add, scale, dot, cross, normalize, length};
function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function length(a) { return Math.hypot(a[0], a[1], a[2]); }
function normalize(a) { const l = length(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }

// A의 카메라 포즈(열 우선, CV 규약 X 오른쪽/Y 아래/Z 앞)를 뷰어 좌표의 카메라→월드 행렬로 바꾼다.
// 양쪽 좌표계가 모두 (X,−Y,−Z)로 뒤집히므로 F·T·F (F = diag(1,−1,−1,1)).
export function poseToViewer(columnMajorPose) {
  const F = mat4.identity();
  F[5] = -1; F[10] = -1;
  return mat4.multiply(mat4.multiply(F, Float32Array.from(columnMajorPose)), F);
}

/**
 * 오빗 카메라. target 주위를 yaw·pitch·distance로 공전하며, 드래그로 회전, 휠로 거리, 오른쪽 버튼(또는 Shift)으로 이동.
 * update()가 돌려주는 view·projection·position을 셰이더 uniform으로 넘긴다.
 */
export function createOrbitCamera(canvas, {K, width, height, near = 0.05, far = 2000, pose, lookDistance = 12}) {
  const projection = mat4.perspectiveFromIntrinsics(K, width, height, near, far);
  const cameraToWorld = poseToViewer(pose);
  const home = {
    eye: mat4.transformPoint(cameraToWorld, [0, 0, 0]),
    up: normalize(mat4.transformDirection(cameraToWorld, [0, 1, 0])),
    forward: normalize(mat4.transformDirection(cameraToWorld, [0, 0, -1]))
  };
  // 공전 기준 좌표계: up을 축으로 하고, 초기 시선으로 yaw=0 방향을 정한다.
  const axisY = home.up;
  const axisZ = normalize(sub(home.forward, scale(axisY, dot(home.forward, axisY))));
  const axisX = cross(axisY, axisZ);
  const state = {target: [0, 0, 0], distance: lookDistance, yaw: 0, pitch: 0};
  const view = new Float32Array(16);
  let position = [0, 0, 0];

  function reset() {
    state.target = add(home.eye, scale(home.forward, lookDistance));
    state.distance = lookDistance;
    // 초기 시선: target에서 eye로 향하는 오프셋 = -forward·distance → yaw는 π(뒤쪽), pitch는 forward의 기울기.
    const offset = normalize(sub(home.eye, state.target));
    state.pitch = Math.asin(Math.max(-1, Math.min(1, dot(offset, axisY))));
    state.yaw = Math.atan2(dot(offset, axisX), dot(offset, axisZ));
  }
  reset();

  function update() {
    const cp = Math.cos(state.pitch), sp = Math.sin(state.pitch);
    const offset = add(add(scale(axisX, state.distance * cp * Math.sin(state.yaw)),
                           scale(axisY, state.distance * sp)),
                       scale(axisZ, state.distance * cp * Math.cos(state.yaw)));
    position = add(state.target, offset);
    view.set(mat4.lookAt(position, state.target, axisY));
    return {view, projection, position, near, far};
  }

  // 마우스·터치 입력. 브라우저 포인터 이벤트만 쓰고 라이브러리 컨트롤은 쓰지 않는다.
  let dragging = null, lastX = 0, lastY = 0;
  canvas.addEventListener('pointerdown', e => {
    dragging = (e.button === 2 || e.shiftKey) ? 'pan' : 'rotate';
    lastX = e.clientX; lastY = e.clientY;
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', e => {
    if (!dragging) return;
    const dx = e.clientX - lastX, dy = e.clientY - lastY;
    lastX = e.clientX; lastY = e.clientY;
    if (dragging === 'rotate') {
      state.yaw -= dx * 2 * Math.PI / canvas.clientWidth;
      state.pitch = Math.max(-1.5, Math.min(1.5, state.pitch + dy * Math.PI / canvas.clientHeight));
    } else {
      // 화면의 한 픽셀이 target 깊이에서 차지하는 길이만큼 target을 옮긴다.
      const perPixel = 2 * state.distance / (projection[5] * canvas.clientHeight);
      const right = [view[0], view[4], view[8]], up = [view[1], view[5], view[9]];
      state.target = add(state.target, add(scale(right, -dx * perPixel), scale(up, dy * perPixel)));
    }
  });
  const stop = e => { dragging = null; if (e.pointerId !== undefined) canvas.releasePointerCapture?.(e.pointerId); };
  canvas.addEventListener('pointerup', stop);
  canvas.addEventListener('pointercancel', stop);
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    state.distance = Math.max(0.2, Math.min(500, state.distance * Math.exp(e.deltaY * 0.001)));
  }, {passive: false});

  return {update, reset, state, projection, near, far};
}
