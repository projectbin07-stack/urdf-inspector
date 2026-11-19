// URDF snippet builders for tests. Each builder returns XML text.

import { readFileSync } from "node:fs";
import { inspectText } from "../src/core/inspect.js";

/** @param {string} body @param {string} [attrs] */
export const robot = (body, attrs = 'name="t"') =>
  `<?xml version="1.0"?>\n<robot ${attrs}>\n${body}\n</robot>\n`;

/**
 * A solid box inertial that passes every inertia check.
 * @param {number} m
 * @param {[number, number, number]} size
 * @param {string} [origin]
 */
export function boxInertial(m, size, origin = "") {
  const [a, b, c] = size;
  const ixx = (m * (b * b + c * c)) / 12;
  const iyy = (m * (a * a + c * c)) / 12;
  const izz = (m * (a * a + b * b)) / 12;
  return `<inertial>${origin}<mass value="${m}"/><inertia ixx="${ixx}" ixy="0" ixz="0" iyy="${iyy}" iyz="0" izz="${izz}"/></inertial>`;
}

/** A link with a 0.1 m cube of 1 kg: visual, collision and matching inertia. @param {string} name */
export const solidLink = (name) =>
  `<link name="${name}">${boxInertial(1, [0.1, 0.1, 0.1])}` +
  `<visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>` +
  `<collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>`;

/** @param {string} name */
export const emptyLink = (name) => `<link name="${name}"/>`;

/**
 * @param {string} name
 * @param {string} type
 * @param {string} parent
 * @param {string} child
 * @param {string} [extra] inner XML (origin, axis, limit, ...)
 */
export const joint = (name, type, parent, child, extra = "") =>
  `<joint name="${name}" type="${type}"><parent link="${parent}"/><child link="${child}"/>${extra}</joint>`;

export const LIMIT = '<limit lower="-1" upper="1" effort="1" velocity="1"/>';

/**
 * A minimal valid two-link robot: base_link and a revolute child `arm`.
 * @param {string} [jointExtra] replaces the joint's inner XML
 */
export const twoLink = (jointExtra = `<axis xyz="0 0 1"/>${LIMIT}`) =>
  robot(
    solidLink("base_link") +
      solidLink("arm") +
      joint("j", "revolute", "base_link", "arm", jointExtra),
  );

/**
 * A wheel link: cylinder of radius r and width 0.03, 0.2 kg, upright
 * (rotated onto y) unless cylRpy says otherwise, with matching inertia.
 * @param {string} name
 * @param {number} [r]
 * @param {string} [cylRpy]
 */
export function wheelLink(name, r = 0.05, cylRpy = "1.5707963267948966 0 0") {
  const m = 0.2;
  const h = 0.03;
  const side = (m * (3 * r * r + h * h)) / 12;
  const axial = (m * r * r) / 2;
  const upright = cylRpy !== "0 0 0";
  const inertia = upright
    ? `ixx="${side}" ixy="0" ixz="0" iyy="${axial}" iyz="0" izz="${side}"`
    : `ixx="${side}" ixy="0" ixz="0" iyy="${side}" iyz="0" izz="${axial}"`;
  const shape = `<origin xyz="0 0 0" rpy="${cylRpy}"/><geometry><cylinder radius="${r}" length="${h}"/></geometry>`;
  return `<link name="${name}"><inertial><mass value="${m}"/><inertia ${inertia}/></inertial><visual>${shape}</visual><collision>${shape}</collision></link>`;
}

/**
 * A wheeled base under base_footprint (base_link at height z).
 * @param {{ name: string, xyz: string, axis?: string, r?: number, cylRpy?: string }[]} wheels
 * @param {{ z?: number, extra?: string }} [opts]
 */
export function wheeled(wheels, opts = {}) {
  const z = opts.z ?? 0.05;
  let body =
    emptyLink("base_footprint") +
    joint("base_joint", "fixed", "base_footprint", "base_link", `<origin xyz="0 0 ${z}"/>`) +
    `<link name="base_link">${boxInertial(2, [0.3, 0.2, 0.05])}<visual><geometry><box size="0.3 0.2 0.05"/></geometry></visual><collision><geometry><box size="0.3 0.2 0.05"/></geometry></collision></link>`;
  for (const w of wheels) {
    body += joint(
      `${w.name}_joint`,
      "continuous",
      "base_link",
      w.name,
      `<origin xyz="${w.xyz}"/><axis xyz="${w.axis ?? "0 1 0"}"/><limit effort="1" velocity="10"/>`,
    );
    body += wheelLink(w.name, w.r ?? 0.05, w.cylRpy);
  }
  return robot(body + (opts.extra ?? ""));
}

/** Four mecanum wheels at lx = 0.1, ly = 0.12, radius 0.05, base_link at z 0.05. */
export const MECANUM = [
  { name: "front_left_wheel", xyz: "0.1 0.12 0" },
  { name: "front_right_wheel", xyz: "0.1 -0.12 0" },
  { name: "rear_left_wheel", xyz: "-0.1 0.12 0" },
  { name: "rear_right_wheel", xyz: "-0.1 -0.12 0" },
];

/** @param {string} text */
export const ids = (text) => inspectText(text).findings.map((f) => f.id);

/** @param {string} name */
export const example = (name) =>
  readFileSync(new URL(`../examples/${name}`, import.meta.url), "utf8");
