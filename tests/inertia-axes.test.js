// The inertia orientation check: a tensor whose principal moments match the
// link's primitive shape but whose principal axes do not (the classic case
// is a wheel cylinder rotated in <collision> while <inertial> keeps the
// unrotated tensor). v0.1.0 compared sorted moments only and missed it.

import assert from "node:assert/strict";
import { test } from "node:test";
import { inspectText } from "../src/core/inspect.js";
import {
  AXIS_ANGLE_TOL_DEG,
  AXIS_GAP,
  principalAxisMismatch,
  primitiveInertia,
  rotateTensor,
} from "../src/core/inertia.js";
import { symEigen } from "../src/core/math.js";
import { ids, robot } from "./helpers.js";

const HALF_PI = "1.5707963267948966";
const BASE =
  '<link name="base_link"><inertial><mass value="5"/><inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/></inertial>' +
  '<collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision></link>';

/**
 * One link 'part' on a continuous joint below base_link.
 * @param {{ inertia: string, geometry: string, geomRpy?: string, inertialRpy?: string, mass?: number }} o
 */
const part = (o) =>
  robot(
    BASE +
      `<link name="part"><inertial><origin xyz="0 0 0" rpy="${o.inertialRpy ?? "0 0 0"}"/><mass value="${o.mass ?? 1}"/>` +
      `<inertia ${o.inertia}/></inertial>` +
      `<collision><origin rpy="${o.geomRpy ?? "0 0 0"}"/><geometry>${o.geometry}</geometry></collision></link>` +
      '<joint name="j" type="continuous"><parent link="base_link"/><child link="part"/><axis xyz="0 1 0"/></joint>',
  );

/** @param {number} x @param {number} y @param {number} z */
const diag = (x, y, z) => `ixx="${x}" ixy="0" ixz="0" iyy="${y}" iyz="0" izz="${z}"`;

// Solid cylinder r = 0.04, h = 0.03, m = 1:
// side = (3 r^2 + h^2) / 12 = 0.000475, axis = r^2 / 2 = 0.0008.
const WHEEL = '<cylinder radius="0.04" length="0.03"/>';
const SIDE = 0.000475;
const AXIS = 0.0008;

/** @param {string} text */
const axesFinding = (text) => inspectText(text).findings.find((f) => f.id === "inertia-axes");

test("an unrotated wheel tensor under a rotated cylinder is reported", () => {
  // The audit reproduction: the cylinder is rolled 90 deg so its axis is
  // along y, but the inertia still has the axis moment on z.
  const text = part({
    inertia: diag(0.00045, 0.00045, AXIS),
    geometry: WHEEL,
    geomRpy: "1.5707963 0 0",
  });
  const f = axesFinding(text);
  assert.ok(f, "inertia-axes expected");
  assert.equal(f.severity, "warning");
  assert.match(f.message, /not rotated with its cylinder geometry/);
  assert.match(f.message, /90 deg apart/);
  const data = /** @type {Record<string, any>} */ (f.data);
  assert.ok(Math.abs(data.angleDeg - 90) < 1e-3);
  // The reference in the inertial frame has the axis moment on y.
  assert.ok(Math.abs(data.reference.iyy - AXIS) < 1e-9);
  assert.ok(Math.abs(data.reference.izz - SIDE) < 1e-9);
  // Moments are within the 10x band, so the magnitude check stays quiet.
  assert.ok(!ids(text).includes("inertia-implausible"));
});

test("a correctly rotated wheel tensor passes", () => {
  // Tensor rotated into the link frame by hand.
  const byHand = part({
    inertia: diag(SIDE, AXIS, SIDE),
    geometry: WHEEL,
    geomRpy: `${HALF_PI} 0 0`,
  });
  assert.equal(axesFinding(byHand), undefined);
  // Tensor left in the shape frame, but the <inertial> origin carries the same rpy.
  const viaOrigin = part({
    inertia: diag(SIDE, SIDE, AXIS),
    geometry: WHEEL,
    geomRpy: `${HALF_PI} 0 0`,
    inertialRpy: `${HALF_PI} 0 0`,
  });
  assert.equal(axesFinding(viaOrigin), undefined);
  // Rolled by -90 deg instead of +90: the axis is the same line.
  const negative = part({
    inertia: diag(SIDE, AXIS, SIDE),
    geometry: WHEEL,
    geomRpy: `-${HALF_PI} 0 0`,
  });
  assert.equal(axesFinding(negative), undefined);
});

test("an inertial origin rpy that rotates an already rotated tensor is reported", () => {
  const f = axesFinding(
    part({
      inertia: diag(SIDE, AXIS, SIDE),
      geometry: WHEEL,
      geomRpy: `${HALF_PI} 0 0`,
      inertialRpy: `${HALF_PI} 0 0`,
    }),
  );
  assert.ok(f);
});

test("degenerate axes never false-alarm", () => {
  // Sphere: every axis is principal, whatever the rotations.
  const s = (2 / 5) * 0.05 * 0.05;
  const sphere = part({
    inertia: diag(s, s, s),
    geometry: '<sphere radius="0.05"/>',
    geomRpy: "0.3 1.1 -0.7",
    inertialRpy: "1.2 0 0.4",
  });
  assert.equal(axesFinding(sphere), undefined);
  // Cylinder spun about its own axis: the side pair is degenerate.
  const spun = part({ inertia: diag(SIDE, SIDE, AXIS), geometry: WHEEL, geomRpy: "0 0 0.7" });
  assert.equal(axesFinding(spun), undefined);
  // A cylinder with h = sqrt(3) r has three equal moments, like a sphere.
  const r = 0.04;
  const e = (r * r) / 2;
  const squat = part({
    inertia: diag(e, e, e),
    geometry: `<cylinder radius="${r}" length="${Math.sqrt(3) * r}"/>`,
    geomRpy: `${HALF_PI} 0 0`,
  });
  assert.equal(axesFinding(squat), undefined);
  // A cube is isotropic too.
  const c = (2 * 0.1 * 0.1) / 12;
  const cube = part({
    inertia: diag(c, c, c),
    geometry: '<box size="0.1 0.1 0.1"/>',
    geomRpy: "0 0 0.5",
  });
  assert.equal(axesFinding(cube), undefined);
});

test("a box tensor that was not rotated with the box is reported", () => {
  // 0.4 x 0.2 x 0.1 box, m = 2, yawed 90 deg; tensor still in the box frame.
  const ixx = (2 * 0.05) / 12;
  const iyy = (2 * 0.17) / 12;
  const izz = (2 * 0.2) / 12;
  const geometry = '<box size="0.4 0.2 0.1"/>';
  const geomRpy = `0 0 ${HALF_PI}`;
  const wrong = axesFinding(part({ inertia: diag(ixx, iyy, izz), geometry, geomRpy, mass: 2 }));
  assert.ok(wrong);
  assert.match(wrong.message, /not rotated with its box geometry/);
  const right = part({ inertia: diag(iyy, ixx, izz), geometry, geomRpy, mass: 2 });
  assert.equal(axesFinding(right), undefined);
});

test("small misalignments stay within the tolerance", () => {
  const deg = (/** @type {number} */ d) => `${(d * Math.PI) / 180} 0 0`;
  // Geometry rolled 5 deg, tensor unrotated: below the 10 deg tolerance.
  const small = part({ inertia: diag(SIDE, SIDE, AXIS), geometry: WHEEL, geomRpy: deg(5) });
  assert.equal(axesFinding(small), undefined);
  const large = part({ inertia: diag(SIDE, SIDE, AXIS), geometry: WHEEL, geomRpy: deg(15) });
  assert.ok(axesFinding(large));
  assert.equal(AXIS_ANGLE_TOL_DEG, 10);
  assert.equal(AXIS_GAP, 0.1);
});

test("a tensor far from the reference reports only inertia-implausible", () => {
  const found = ids(part({ inertia: diag(1, 1, 2), geometry: WHEEL, geomRpy: `${HALF_PI} 0 0` }));
  assert.ok(found.includes("inertia-implausible"));
  assert.ok(!found.includes("inertia-axes"));
});

test("symEigen returns orthonormal eigenvectors", () => {
  const t = rotateTensor(
    { ixx: 1, ixy: 0, ixz: 0, iyy: 2, iyz: 0, izz: 3 },
    [0.3, -0.4, 1.1],
    [0, 0, 0],
  );
  /** @type {import("../src/core/math.js").Mat3} */
  const S = [t.ixx, t.ixy, t.ixz, t.ixy, t.iyy, t.iyz, t.ixz, t.iyz, t.izz];
  const { values, vectors } = symEigen(S);
  assert.deepEqual(
    values.map((v) => Number(v.toFixed(12))),
    [1, 2, 3],
  );
  for (let k = 0; k < 3; k++) {
    const v = vectors[k];
    for (let i = 0; i < 3; i++) {
      const sv = S[3 * i] * v[0] + S[3 * i + 1] * v[1] + S[3 * i + 2] * v[2];
      assert.ok(Math.abs(sv - values[k] * v[i]) < 1e-12);
    }
    assert.ok(Math.abs(Math.hypot(v[0], v[1], v[2]) - 1) < 1e-12);
  }
});

test("principalAxisMismatch compares only well-separated axes", () => {
  const ref = /** @type {import("../src/core/urdf.js").InertiaTensor} */ (
    primitiveInertia({ type: "cylinder", radius: 0.04, length: 0.03 }, 1)
  );
  assert.equal(principalAxisMismatch(ref, ref), null);
  const rolled = rotateTensor(ref, [Math.PI / 2, 0, 0], [0, 0, 0]);
  const m = principalAxisMismatch(ref, rolled);
  assert.ok(m);
  assert.equal(m.rank, 2);
  assert.ok(Math.abs(m.angleDeg - 90) < 1e-9);
  const sphere = { ixx: 1, ixy: 0, ixz: 0, iyy: 1, iyz: 0, izz: 1 };
  const spun = rotateTensor(sphere, [0.4, 0.2, 0.1], [0, 0, 0]);
  assert.equal(principalAxisMismatch(sphere, spun), null);
});
