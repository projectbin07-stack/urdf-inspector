// Shared input for every check: the model, its tree, link poses at the
// zero configuration and the detected wheel base.

import { forwardKinematics } from "../fk.js";
import { measureWheels } from "../wheels.js";

/**
 * @typedef {object} Context
 * @property {import("../urdf.js").Robot} robot
 * @property {import("../tree.js").KinTree} tree
 * @property {Map<string, import("../math.js").Transform>} poses root-frame poses, all joints at 0
 * @property {import("../wheels.js").WheelBase | null} wheels
 */

/**
 * @param {import("../urdf.js").Robot} robot
 * @param {import("../tree.js").KinTree} tree
 * @returns {Context}
 */
export function makeContext(robot, tree) {
  const poses = forwardKinematics(tree);
  return { robot, tree, poses, wheels: measureWheels(tree, poses) };
}

/**
 * The single primitive shape that describes a link's volume: its only
 * collision shape if that is a box, cylinder or sphere, else its only
 * visual shape when it has no collision. Null when ambiguous.
 * @param {import("../urdf.js").Link} link
 * @returns {import("../urdf.js").Shape | null}
 */
export function singlePrimitive(link) {
  const list = link.collisions.length > 0 ? link.collisions : link.visuals;
  if (list.length !== 1) return null;
  const g = list[0].geometry;
  return g && (g.type === "box" || g.type === "cylinder" || g.type === "sphere") ? list[0] : null;
}

/** Format a number with six significant digits. @param {number} v */
export const fmt = (v) => String(Number(v.toPrecision(6)));
