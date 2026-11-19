// Forward kinematics over the kinematic tree, total mass and centre of mass.
//
// For a joint with origin (xyz, rpy) and axis a, the child link frame in the
// parent link frame is
//   T_parent_child(q) = T(xyz, rpy) * M(q)
// with M(q) a rotation by q about a/|a| for revolute and continuous joints,
// a translation by q * a/|a| for prismatic joints, and the identity for
// fixed, floating and planar joints (their extra coordinates are not
// modelled; floating and planar are shown at their origin).

import {
  apply,
  axisAngle,
  compose,
  fromXyzRpy,
  identity3,
  identityTransform,
  normalize,
  norm,
  scale,
} from "./math.js";

/** @typedef {import("./math.js").Transform} Transform */
/** @typedef {import("./math.js").Vec3} Vec3 */
/** @typedef {import("./tree.js").KinTree} KinTree */
/** @typedef {import("./urdf.js").Joint} Joint */

/** @param {Joint} j */
export const isMovable = (j) =>
  j.type === "revolute" || j.type === "continuous" || j.type === "prismatic";

/**
 * Resolve every movable joint's position, applying mimic relations.
 * Unspecified joints are 0. A mimic of an unknown joint or a mimic cycle
 * falls back to the joint's own value.
 * @param {KinTree} tree
 * @param {Record<string, number>} q
 * @returns {Map<string, number>}
 */
export function resolvePositions(tree, q) {
  /** @type {Map<string, number>} */
  const out = new Map();
  /** @param {Joint} j @param {Set<string>} stack @returns {number} */
  const value = (j, stack) => {
    const known = out.get(j.name);
    if (known !== undefined) return known;
    let v = q[j.name] ?? 0;
    if (j.mimic) {
      const src = tree.joints.get(j.mimic.joint);
      if (src && !stack.has(src.name) && src !== j) {
        stack.add(j.name);
        v = j.mimic.multiplier * value(src, stack) + j.mimic.offset;
        stack.delete(j.name);
      }
    }
    out.set(j.name, v);
    return v;
  };
  for (const j of tree.joints.values()) value(j, new Set());
  return out;
}

/**
 * Transform of the child link in the parent link frame at position q.
 * @param {Joint} j
 * @param {number} q
 * @returns {Transform}
 */
export function jointTransform(j, q) {
  const base = fromXyzRpy(j.origin.xyz, j.origin.rpy);
  if (q === 0 || norm(j.axis) === 0) return base;
  const u = normalize(j.axis);
  if (j.type === "revolute" || j.type === "continuous")
    return compose(base, { R: axisAngle(u, q), p: [0, 0, 0] });
  if (j.type === "prismatic") return compose(base, { R: identity3(), p: scale(u, q) });
  return base;
}

/**
 * World (root-frame) pose of every link reachable from the root.
 * @param {KinTree} tree
 * @param {Record<string, number>} [q] joint positions in rad or m
 * @returns {Map<string, Transform>}
 */
export function forwardKinematics(tree, q = {}) {
  const pos = resolvePositions(tree, q);
  /** @type {Map<string, Transform>} */
  const poses = new Map();
  if (tree.root === null) return poses;
  poses.set(tree.root, identityTransform());
  for (const name of tree.order) {
    const parentPose = /** @type {Transform} */ (poses.get(name));
    for (const j of tree.childJoints.get(name) ?? []) {
      if (poses.has(j.child)) continue;
      poses.set(j.child, compose(parentPose, jointTransform(j, pos.get(j.name) ?? 0)));
    }
  }
  return poses;
}

/**
 * Total mass and centre of mass in the root frame. Links without an
 * inertial or with a non-positive or missing mass contribute nothing.
 * @param {KinTree} tree
 * @param {Map<string, Transform>} poses
 * @returns {{ mass: number, com: Vec3 | null, contributors: number }}
 */
export function massProperties(tree, poses) {
  let mass = 0;
  /** @type {Vec3} */
  let acc = [0, 0, 0];
  let contributors = 0;
  for (const name of tree.order) {
    const link = tree.links.get(name);
    const pose = poses.get(name);
    const m = link?.inertial?.mass;
    if (!link?.inertial || !pose || m === null || m === undefined || !(m > 0)) continue;
    const c = apply(pose, link.inertial.origin.xyz);
    acc = [acc[0] + m * c[0], acc[1] + m * c[1], acc[2] + m * c[2]];
    mass += m;
    contributors++;
  }
  return { mass, com: mass > 0 ? scale(acc, 1 / mass) : null, contributors };
}

/**
 * Slider range for a movable joint: its limits when they form a valid
 * interval (a missing lower or upper is 0, as in urdfdom), otherwise one
 * turn either way for rotary joints and 0.5 m for prismatic ones.
 * @param {Joint} j
 * @returns {{ min: number, max: number }}
 */
export function sliderRange(j) {
  if (j.type !== "continuous" && j.limit) {
    const lo = j.limit.lower ?? 0;
    const hi = j.limit.upper ?? 0;
    if (lo < hi) return { min: lo, max: hi };
  }
  return j.type === "prismatic" ? { min: -0.5, max: 0.5 } : { min: -Math.PI, max: Math.PI };
}
