// Inertia tensors: validity (positive definite, triangle inequality on the
// principal moments) and reference values for solid primitives.
//
// Solid-body moments about the centroid (shape axes as in URDF: a cylinder
// is along its local z axis, a box has full side lengths size = (a, b, c)):
//   box       Ixx = m (b^2 + c^2) / 12, Iyy = m (a^2 + c^2) / 12, Izz = m (a^2 + b^2) / 12
//   cylinder  Ixx = Iyy = m (3 r^2 + h^2) / 12, Izz = m r^2 / 2
//   sphere    Ixx = Iyy = Izz = 2 m r^2 / 5

import { mul3, rpyToMatrix, symEigenvalues, transpose3 } from "./math.js";

/** Relative tolerance of the triangle test: values rounded to six significant digits pass. */
export const TRIANGLE_TOL = 1e-5;

/** @typedef {import("./math.js").Mat3} Mat3 */
/** @typedef {import("./math.js").Vec3} Vec3 */
/** @typedef {import("./urdf.js").InertiaTensor} InertiaTensor */
/** @typedef {import("./urdf.js").Geometry} Geometry */

/** @param {InertiaTensor} t @returns {Mat3} */
export const tensorToMatrix = (t) => [
  t.ixx,
  t.ixy,
  t.ixz,
  t.ixy,
  t.iyy,
  t.iyz,
  t.ixz,
  t.iyz,
  t.izz,
];

/** @param {Mat3} m @returns {InertiaTensor} */
export const matrixToTensor = (m) => ({
  ixx: m[0],
  ixy: m[1],
  ixz: m[2],
  iyy: m[4],
  iyz: m[5],
  izz: m[8],
});

/**
 * Principal moments (ascending) and the two physical-validity tests.
 * Tolerances are relative to the largest moment so that values printed
 * with a few significant digits (e.g. a thin rod with I1 = 0) still pass
 * the triangle test at equality.
 * @param {InertiaTensor} t
 * @returns {{ principal: Vec3, positiveDefinite: boolean, triangle: boolean, slack: number }}
 *   slack = I1 + I2 - I3, negative when the triangle inequality fails
 */
export function inertiaValidity(t) {
  const principal = symEigenvalues(tensorToMatrix(t));
  const [a, b, c] = principal;
  const tol = 1e-9 * Math.max(Math.abs(c), 1e-300);
  const slack = a + b - c;
  return {
    principal,
    positiveDefinite: a > tol,
    triangle: slack >= -TRIANGLE_TOL * Math.abs(c),
    slack,
  };
}

/**
 * Volume of a primitive in m^3, or null for meshes and unknown shapes.
 * @param {Geometry} g
 * @returns {number | null}
 */
export function volume(g) {
  if (g.type === "box") return g.size[0] * g.size[1] * g.size[2];
  if (g.type === "cylinder") return Math.PI * g.radius * g.radius * g.length;
  if (g.type === "sphere") return (4 / 3) * Math.PI * g.radius ** 3;
  return null;
}

/**
 * Solid-body inertia of a primitive about its own centroid, in the shape's
 * frame, or null for meshes and unknown shapes.
 * @param {Geometry} g
 * @param {number} m
 * @returns {InertiaTensor | null}
 */
export function primitiveInertia(g, m) {
  /** @param {number} x @param {number} y @param {number} z */
  const diag = (x, y, z) => ({ ixx: x, ixy: 0, ixz: 0, iyy: y, iyz: 0, izz: z });
  if (g.type === "box") {
    const [a, b, c] = g.size;
    return diag((m * (b * b + c * c)) / 12, (m * (a * a + c * c)) / 12, (m * (a * a + b * b)) / 12);
  }
  if (g.type === "cylinder") {
    const r = g.radius;
    const h = g.length;
    const side = (m * (3 * r * r + h * h)) / 12;
    return diag(side, side, (m * r * r) / 2);
  }
  if (g.type === "sphere") {
    const s = (2 / 5) * m * g.radius * g.radius;
    return diag(s, s, s);
  }
  return null;
}

/**
 * Express a tensor given in frame S in frame F, where rpyF and rpyS are the
 * orientations of F and S in a common (link) frame: I_F = R I_S R^T with
 * R = R_F^T R_S.
 * @param {InertiaTensor} t
 * @param {Vec3} rpyS
 * @param {Vec3} rpyF
 * @returns {InertiaTensor}
 */
export function rotateTensor(t, rpyS, rpyF) {
  const R = mul3(transpose3(rpyToMatrix(rpyF)), rpyToMatrix(rpyS));
  const m = mul3(mul3(R, tensorToMatrix(t)), transpose3(R));
  // Clean tiny asymmetric round-off and values that are zero to machine precision.
  const scaleRef = Math.max(Math.abs(m[0]), Math.abs(m[4]), Math.abs(m[8]));
  /** @param {number} v */
  const clean = (v) => (Math.abs(v) < 1e-12 * scaleRef ? 0 : v);
  const sym = matrixToTensor(m);
  return {
    ixx: clean(sym.ixx),
    ixy: clean((m[1] + m[3]) / 2),
    ixz: clean((m[2] + m[6]) / 2),
    iyy: clean(sym.iyy),
    iyz: clean((m[5] + m[7]) / 2),
    izz: clean(sym.izz),
  };
}
