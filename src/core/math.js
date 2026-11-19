// Small fixed-size linear algebra: 3-vectors, 3x3 rotation matrices and
// rigid transforms. Matrices are row-major arrays. Everything is plain
// numbers so results are identical in Node and in the browser.

/** @typedef {[number, number, number]} Vec3 */
/** @typedef {[number, number, number, number, number, number, number, number, number]} Mat3 */
/**
 * A rigid transform: rotation R (row-major 3x3) and translation p.
 * Applied to a point x as R x + p.
 * @typedef {{ R: Mat3, p: Vec3 }} Transform
 */

/** @returns {Mat3} */
export const identity3 = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];

/** @returns {Transform} */
export const identityTransform = () => ({ R: identity3(), p: [0, 0, 0] });

/** @param {Vec3} a @param {Vec3} b @returns {Vec3} */
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
/** @param {Vec3} a @param {Vec3} b @returns {Vec3} */
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
/** @param {Vec3} a @param {number} s @returns {Vec3} */
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
/** @param {Vec3} a @param {Vec3} b */
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
/** @param {Vec3} a @param {Vec3} b @returns {Vec3} */
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
/** @param {Vec3} a */
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
/** @param {Vec3} a @returns {Vec3} */
export const normalize = (a) => {
  const l = norm(a);
  return l > 0 ? scale(a, 1 / l) : [0, 0, 0];
};

/** @param {Mat3} A @param {Mat3} B @returns {Mat3} */
export function mul3(A, B) {
  /** @type {number[]} */
  const C = new Array(9).fill(0);
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++)
      C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
  return /** @type {Mat3} */ (C);
}

/** @param {Mat3} A @param {Vec3} v @returns {Vec3} */
export const mulVec = (A, v) => [
  A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
  A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
  A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
];

/** @param {Mat3} A @returns {Mat3} */
export const transpose3 = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];

/** Column k of a matrix. @param {Mat3} A @param {number} k @returns {Vec3} */
export const column = (A, k) => [A[k], A[3 + k], A[6 + k]];

/**
 * URDF roll-pitch-yaw: fixed-axis rotations about X (roll), then Y (pitch),
 * then Z (yaw), so R = Rz(yaw) * Ry(pitch) * Rx(roll).
 * @param {Vec3} rpy
 * @returns {Mat3}
 */
export function rpyToMatrix(rpy) {
  const [r, p, y] = rpy;
  const cr = Math.cos(r);
  const sr = Math.sin(r);
  const cp = Math.cos(p);
  const sp = Math.sin(p);
  const cy = Math.cos(y);
  const sy = Math.sin(y);
  return [
    cy * cp,
    cy * sp * sr - sy * cr,
    cy * sp * cr + sy * sr,
    sy * cp,
    sy * sp * sr + cy * cr,
    sy * sp * cr - cy * sr,
    -sp,
    cp * sr,
    cp * cr,
  ];
}

/**
 * Inverse of rpyToMatrix for a proper rotation matrix. At pitch = +-90 deg
 * roll is set to 0 and yaw absorbs the remaining rotation.
 * @param {Mat3} R
 * @returns {Vec3}
 */
export function matrixToRpy(R) {
  const sp = Math.max(-1, Math.min(1, -R[6]));
  const pitch = Math.asin(sp);
  if (Math.abs(sp) > 1 - 1e-12) {
    return [0, pitch, Math.atan2(-R[1], R[4])];
  }
  return [Math.atan2(R[7], R[8]), pitch, Math.atan2(R[3], R[0])];
}

/**
 * Rotation by `angle` about the unit vector `u` (Rodrigues' formula).
 * @param {Vec3} u
 * @param {number} angle
 * @returns {Mat3}
 */
export function axisAngle(u, angle) {
  const [x, y, z] = u;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return [
    t * x * x + c,
    t * x * y - s * z,
    t * x * z + s * y,
    t * x * y + s * z,
    t * y * y + c,
    t * y * z - s * x,
    t * x * z - s * y,
    t * y * z + s * x,
    t * z * z + c,
  ];
}

/** a then b: the transform of frame b expressed through frame a. @param {Transform} a @param {Transform} b @returns {Transform} */
export const compose = (a, b) => ({ R: mul3(a.R, b.R), p: add(mulVec(a.R, b.p), a.p) });

/** @param {Transform} t @param {Vec3} x @returns {Vec3} */
export const apply = (t, x) => add(mulVec(t.R, x), t.p);

/** @param {Transform} t @returns {Transform} */
export function invert(t) {
  const Rt = transpose3(t.R);
  return { R: Rt, p: scale(mulVec(Rt, t.p), -1) };
}

/** @param {Vec3} xyz @param {Vec3} rpy @returns {Transform} */
export const fromXyzRpy = (xyz, rpy) => ({ R: rpyToMatrix(rpy), p: [xyz[0], xyz[1], xyz[2]] });

/**
 * Eigenvalues of a symmetric 3x3 matrix by cyclic Jacobi rotations, sorted
 * ascending. Converges to machine precision in a handful of sweeps.
 * @param {Mat3} S
 * @returns {Vec3}
 */
export function symEigenvalues(S) {
  const a = [
    [S[0], S[1], S[2]],
    [S[3], S[4], S[5]],
    [S[6], S[7], S[8]],
  ];
  for (let sweep = 0; sweep < 50; sweep++) {
    const off = Math.abs(a[0][1]) + Math.abs(a[0][2]) + Math.abs(a[1][2]);
    const scaleRef = Math.abs(a[0][0]) + Math.abs(a[1][1]) + Math.abs(a[2][2]);
    if (off <= 1e-15 * scaleRef || off === 0) break;
    for (const [p, q] of [
      [0, 1],
      [0, 2],
      [1, 2],
    ]) {
      if (a[p][q] === 0) continue;
      const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1);
      const s = t * c;
      for (let k = 0; k < 3; k++) {
        const akp = a[k][p];
        const akq = a[k][q];
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k++) {
        const apk = a[p][k];
        const aqk = a[q][k];
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
      }
    }
  }
  const ev = [a[0][0], a[1][1], a[2][2]].sort((x, y) => x - y);
  return [ev[0], ev[1], ev[2]];
}

/** Round to `digits` significant digits for stable printing. @param {number} v @param {number} [digits] */
export const sig = (v, digits = 6) => (v === 0 ? 0 : Number(v.toPrecision(digits)));
