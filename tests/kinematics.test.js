// Math, forward kinematics, mass properties and primitive inertia against
// hand-computed values.

import assert from "node:assert/strict";
import { test } from "node:test";
import {
  forwardKinematics,
  jointTransform,
  massProperties,
  resolvePositions,
} from "../src/core/fk.js";
import { inertiaValidity, primitiveInertia, rotateTensor, volume } from "../src/core/inertia.js";
import { inspectText } from "../src/core/inspect.js";
import {
  apply,
  axisAngle,
  compose,
  cross,
  invert,
  matrixToRpy,
  mul3,
  rpyToMatrix,
  sig,
  symEigenvalues,
} from "../src/core/math.js";
import { buildTree } from "../src/core/tree.js";
import { readUrdf } from "../src/core/urdf.js";
import { parseXml } from "../src/core/xml.js";
import { emptyLink, example, joint, robot, solidLink } from "./helpers.js";

const EPS = 1e-12;
/** @param {number[]} a @param {number[]} b @param {number} [eps] */
const close = (a, b, eps = EPS) => {
  assert.equal(a.length, b.length);
  a.forEach((v, k) => assert.ok(Math.abs(v - b[k]) <= eps, `index ${k}: ${v} vs ${b[k]}`));
};

/** @param {string} text */
const treeOf = (text) => {
  const r = readUrdf(parseXml(text)).robot;
  assert.ok(r);
  return buildTree(r).tree;
};

test("rpy is fixed-axis X, then Y, then Z: R = Rz Ry Rx", () => {
  const h = Math.PI / 2;
  // Yaw 90 deg maps x to y.
  close(apply({ R: rpyToMatrix([0, 0, h]), p: [0, 0, 0] }, [1, 0, 0]), [0, 1, 0]);
  // Pitch 90 deg maps x to -z.
  close(apply({ R: rpyToMatrix([0, h, 0]), p: [0, 0, 0] }, [1, 0, 0]), [0, 0, -1]);
  // Roll 90 deg maps y to z.
  close(apply({ R: rpyToMatrix([h, 0, 0]), p: [0, 0, 0] }, [0, 1, 0]), [0, 0, 1]);
  // Composition order: rpy (r, p, y) equals Rz(y) * Ry(p) * Rx(r).
  const [r, p, y] = [0.3, -0.7, 1.1];
  close(
    rpyToMatrix([r, p, y]),
    mul3(mul3(axisAngle([0, 0, 1], y), axisAngle([0, 1, 0], p)), axisAngle([1, 0, 0], r)),
  );
  // Optical frame rotation from REP 103.
  close(rpyToMatrix([-h, 0, -h]), [0, 0, 1, -1, 0, 0, 0, -1, 0]);
});

test("matrixToRpy inverts rpyToMatrix, including at the pitch singularity", () => {
  for (const rpy of /** @type {[number, number, number][]} */ ([
    [0.1, 0.2, 0.3],
    [-2.5, 1.2, 3.0],
    [0, 0, 0],
  ]))
    close(matrixToRpy(rpyToMatrix(rpy)), rpy, 1e-12);
  const s = rpyToMatrix([0.4, Math.PI / 2, 0.1]);
  close(rpyToMatrix(matrixToRpy(s)), s, 1e-9);
  close(matrixToRpy(rpyToMatrix([0, -Math.PI / 2, 0.5])).slice(1), [-Math.PI / 2, 0.5], 1e-9);
});

test("compose, invert and cross", () => {
  const a = {
    R: rpyToMatrix([0.1, 0.2, 0.3]),
    p: /** @type {[number, number, number]} */ ([1, 2, 3]),
  };
  const id = compose(a, invert(a));
  close(id.R, [1, 0, 0, 0, 1, 0, 0, 0, 1]);
  close(id.p, [0, 0, 0]);
  close(cross([1, 0, 0], [0, 1, 0]), [0, 0, 1]);
  assert.equal(sig(0), 0);
  assert.equal(sig(1.23456789, 3), 1.23);
});

test("symmetric eigenvalues", () => {
  close(symEigenvalues([2, 0, 0, 0, 3, 0, 0, 0, 1]), [1, 2, 3]);
  // [[2,1,0],[1,2,0],[0,0,5]] has eigenvalues 1, 3, 5.
  close(symEigenvalues([2, 1, 0, 1, 2, 0, 0, 0, 5]), [1, 3, 5], 1e-12);
  // A rotated diagonal tensor keeps its eigenvalues.
  const R = rpyToMatrix([0.3, -0.4, 1.2]);
  const D = [0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.4];
  const Rt = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const S = mul3(mul3(R, /** @type {any} */ (D)), /** @type {any} */ (Rt));
  close(symEigenvalues(S), [0.1, 0.2, 0.4], 1e-12);
});

test("joint transforms: revolute rotates about the axis, prismatic translates", () => {
  const t = treeOf(
    robot(
      emptyLink("a") +
        emptyLink("b") +
        emptyLink("c") +
        emptyLink("d") +
        joint(
          "r",
          "revolute",
          "a",
          "b",
          '<origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="0 0 2"/><limit lower="-3" upper="3" effort="1" velocity="1"/>',
        ) +
        joint(
          "p",
          "prismatic",
          "b",
          "c",
          '<origin xyz="0 1 0"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/>',
        ) +
        joint("f", "floating", "c", "d", '<origin xyz="0 0 1"/>'),
    ),
  );
  const poses = forwardKinematics(t, { r: Math.PI / 2, p: 0.5, f: 7 });
  // b: at (1,0,0), yawed by 90 (origin) + 90 (joint) = 180 deg; the axis length is normalised.
  close(/** @type {any} */ (poses.get("b")).p, [1, 0, 0]);
  close(/** @type {any} */ (poses.get("b")).R, rpyToMatrix([0, 0, Math.PI]), 1e-12);
  // c: origin (0,1,0) in b -> (1,-1,0); then 0.5 along b's x (-x world) -> (0.5,-1,0).
  close(/** @type {any} */ (poses.get("c")).p, [0.5, -1, 0], 1e-12);
  // Floating joint ignores its value.
  close(/** @type {any} */ (poses.get("d")).p, [0.5, -1, 1], 1e-12);
  const j = /** @type {any} */ (t.joints.get("p"));
  close(jointTransform({ ...j, axis: [0, 0, 0] }, 1).p, [0, 1, 0]);
});

test("2-DOF example arm matches the closed-form tool position", () => {
  const t = treeOf(example("arm_2dof.urdf"));
  for (const [a, b] of [
    [0, 0],
    [0.5, 0.3],
    [-2.0, -1.2],
    [1.3, 1.5],
  ]) {
    const p = /** @type {any} */ (
      forwardKinematics(t, { shoulder_joint: a, elbow_joint: b }).get("tool0")
    ).p;
    close(
      p,
      [0.2 * Math.cos(b) * Math.cos(a), 0.2 * Math.cos(b) * Math.sin(a), 0.3 - 0.2 * Math.sin(b)],
      1e-12,
    );
  }
});

test("mimic joints follow their source; cycles and unknown sources fall back", () => {
  const LIM = '<limit lower="-3" upper="3" effort="1" velocity="1"/>';
  const t = treeOf(
    robot(
      emptyLink("a") +
        emptyLink("b") +
        emptyLink("c") +
        emptyLink("d") +
        emptyLink("e") +
        joint("j1", "revolute", "a", "b", LIM) +
        joint(
          "j2",
          "revolute",
          "b",
          "c",
          `${LIM}<mimic joint="j1" multiplier="-2" offset="0.1"/>`,
        ) +
        joint("j3", "revolute", "c", "d", `${LIM}<mimic joint="j4"/>`) +
        joint("j4", "revolute", "d", "e", `${LIM}<mimic joint="j3"/>`),
    ),
  );
  const q = resolvePositions(t, { j1: 0.5, j2: 99, j3: 0.25 });
  assert.equal(q.get("j1"), 0.5);
  assert.ok(Math.abs(/** @type {number} */ (q.get("j2")) - -0.9) < EPS);
  // j3 <-> j4 is a cycle: j3 takes j4's own value (0) through the mimic, j4 its own fallback.
  assert.equal(typeof q.get("j3"), "number");
  assert.equal(typeof q.get("j4"), "number");
});

test("total mass and centre of mass", () => {
  const t = treeOf(example("arm_2dof.urdf"));
  const zero = massProperties(t, forwardKinematics(t));
  assert.equal(zero.mass, 2.8);
  assert.equal(zero.contributors, 3);
  // base 2 kg at z 0.025, upper arm 0.5 kg at z 0.175, forearm 0.3 kg at (0.1, 0, 0.3).
  close(/** @type {any} */ (zero.com), [0.03 / 2.8, 0, (0.05 + 0.0875 + 0.09) / 2.8], 1e-12);
  // Elbow at 90 deg up (-pi/2): forearm COM at (0, 0, 0.4).
  const up = massProperties(t, forwardKinematics(t, { elbow_joint: -Math.PI / 2 }));
  close(/** @type {any} */ (up.com), [0, 0, (0.05 + 0.0875 + 0.12) / 2.8], 1e-12);
  const none = massProperties(
    treeOf(robot(emptyLink("a"))),
    forwardKinematics(treeOf(robot(emptyLink("a")))),
  );
  assert.equal(none.com, null);
});

test("forward kinematics on a model without a root returns no poses", () => {
  const t = treeOf(
    robot(
      emptyLink("a") +
        emptyLink("b") +
        joint("x", "fixed", "a", "b") +
        joint("y", "fixed", "b", "a"),
    ),
  );
  assert.equal(t.root, null);
  assert.equal(forwardKinematics(t).size, 0);
});

test("primitive inertia and volume reference values", () => {
  close(
    Object.values(
      /** @type {any} */ (primitiveInertia({ type: "box", size: [0.3, 0.2, 0.1] }, 12)),
    ),
    [0.05, 0, 0, 0.1, 0, 0.13],
    1e-12,
  );
  // Cylinder r 0.1, h 0.3, 6 kg: side = 6 (0.03 + 0.09) / 12 = 0.06, axial = 6 * 0.01 / 2 = 0.03.
  close(
    Object.values(
      /** @type {any} */ (primitiveInertia({ type: "cylinder", radius: 0.1, length: 0.3 }, 6)),
    ),
    [0.06, 0, 0, 0.06, 0, 0.03],
    1e-12,
  );
  // Sphere r 0.5, 10 kg: 2/5 * 10 * 0.25 = 1.
  close(
    Object.values(/** @type {any} */ (primitiveInertia({ type: "sphere", radius: 0.5 }, 10))),
    [1, 0, 0, 1, 0, 1],
    1e-12,
  );
  assert.equal(primitiveInertia({ type: "mesh", filename: "x", scale: [1, 1, 1] }, 1), null);
  close([/** @type {number} */ (volume({ type: "box", size: [1, 2, 3] }))], [6]);
  close(
    [/** @type {number} */ (volume({ type: "cylinder", radius: 1, length: 2 }))],
    [2 * Math.PI],
  );
  close([/** @type {number} */ (volume({ type: "sphere", radius: 3 }))], [36 * Math.PI], 1e-9);
  assert.equal(volume({ type: "unknown", tag: "capsule" }), null);
});

test("inertia validity: positive definiteness and the triangle inequality", () => {
  const ok = inertiaValidity({ ixx: 1, ixy: 0, ixz: 0, iyy: 2, iyz: 0, izz: 2.5 });
  assert.equal(ok.positiveDefinite, true);
  assert.equal(ok.triangle, true);
  // A thin rod is the limiting case: I1 = 0 is not positive definite.
  assert.equal(
    inertiaValidity({ ixx: 0, ixy: 0, ixz: 0, iyy: 1, iyz: 0, izz: 1 }).positiveDefinite,
    false,
  );
  // A flat plate sits exactly on the triangle boundary (Izz = Ixx + Iyy), printed to 6 digits.
  assert.equal(
    inertiaValidity({ ixx: 0.0833333, ixy: 0, ixz: 0, iyy: 0.333333, iyz: 0, izz: 0.416667 })
      .triangle,
    true,
  );
  const bad = inertiaValidity({ ixx: 1, ixy: 0, ixz: 0, iyy: 1, iyz: 0, izz: 3 });
  assert.equal(bad.triangle, false);
  close([bad.slack], [-1]);
  // Off-diagonal terms large enough to make a negative eigenvalue.
  assert.equal(
    inertiaValidity({ ixx: 1, ixy: 2, ixz: 0, iyy: 1, iyz: 0, izz: 1 }).positiveDefinite,
    false,
  );
});

test("rotateTensor expresses a tensor in another frame", () => {
  const t = { ixx: 1, ixy: 0, ixz: 0, iyy: 2, iyz: 0, izz: 3 };
  // Shape rotated +90 deg about z relative to the inertial frame: x and y swap.
  const r = rotateTensor(t, [0, 0, Math.PI / 2], [0, 0, 0]);
  close([r.ixx, r.iyy, r.izz, r.ixy, r.ixz, r.iyz], [2, 1, 3, 0, 0, 0], 1e-12);
  // Same rotation on both sides: unchanged.
  const s = rotateTensor(t, [0.3, 0.2, 0.1], [0.3, 0.2, 0.1]);
  close([s.ixx, s.iyy, s.izz], [1, 2, 3], 1e-12);
});

test("summary reports the mass and centre of mass of the examples", () => {
  const r = inspectText(example("mecanum_base.urdf"));
  // 3 + 4 * 0.15 + 0.17 + 0.06 = 3.83 kg.
  assert.ok(Math.abs(r.summary.totalMass - 3.83) < 1e-12);
  // x: lidar 0.17*0.08 + camera 0.06*0.16 = 0.0232 -> 0.0232/3.83.
  close(/** @type {any} */ (r.summary.com).slice(0, 2), [0.0232 / 3.83, 0], 1e-12);
  const solid = inspectText(robot(solidLink("base_link")));
  assert.equal(solid.summary.totalMass, 1);
});
