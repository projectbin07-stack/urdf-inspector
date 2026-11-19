// Wheel detection and measurement for differential and four-wheel
// (mecanum or skid-steer) bases, following the conventions of the
// mecanum-kinematics project: body frame x forward, y left, z up (REP 103),
// wheels FL (+lx, +ly), FR (+lx, -ly), RL (-lx, +ly), RR (-lx, -ly), and a
// positive wheel speed drives the robot forward.

import { apply, column, compose, dot, invert, mulVec, normalize, rpyToMatrix } from "./math.js";

/** @typedef {import("./math.js").Vec3} Vec3 */
/** @typedef {import("./math.js").Transform} Transform */
/** @typedef {import("./tree.js").KinTree} KinTree */
/** @typedef {import("./urdf.js").Joint} Joint */
/** @typedef {import("./urdf.js").Link} Link */

/**
 * @typedef {object} Wheel
 * @property {string} joint
 * @property {string} link
 * @property {Vec3} centre wheel centre in the base frame, m
 * @property {Vec3} axis unit joint axis in the base frame
 * @property {Vec3 | null} cylinderAxis unit cylinder axis in the base frame, if the wheel is a cylinder
 * @property {number | null} radius m, from the collision (else visual) cylinder or sphere
 * @property {number | null} velocityLimit rad/s from <limit velocity>, if any
 * @property {"FL" | "FR" | "RL" | "RR" | "L" | "R" | ""} label
 */

/**
 * @typedef {object} WheelBase
 * @property {string} baseFrame the frame wheels are measured in
 * @property {"differential" | "four-wheel" | "other"} kind
 * @property {Wheel[]} wheels
 * @property {Vec3} centroid mean wheel centre in the base frame
 * @property {number | null} radius mean radius
 * @property {number | null} lx half wheelbase (four-wheel)
 * @property {number | null} ly half track (four-wheel), or half wheel separation (differential)
 */

const COS_5DEG = Math.cos((5 * Math.PI) / 180);

/** @param {number} a @param {number} b */
export const lengthTolerance = (a, b) => Math.max(0.002, 0.02 * Math.max(Math.abs(a), Math.abs(b)));

/**
 * Is this joint a drive wheel? Continuous joints whose joint or child link
 * name contains "wheel", or whose child link is a cylinder; casters excluded.
 * @param {Joint} j
 * @param {KinTree} tree
 */
export function isWheelJoint(j, tree) {
  if (j.type !== "continuous") return false;
  if (/caster|castor/i.test(j.name) || /caster|castor/i.test(j.child)) return false;
  if (/wheel/i.test(j.name) || /wheel/i.test(j.child)) return true;
  const link = tree.links.get(j.child);
  return (
    !!link && [...link.collisions, ...link.visuals].some((s) => s.geometry?.type === "cylinder")
  );
}

/**
 * @param {Link | undefined} link
 * @returns {{ shape: import("./urdf.js").Shape, radius: number } | null}
 */
function wheelShape(link) {
  if (!link) return null;
  for (const list of [link.collisions, link.visuals]) {
    for (const s of list) {
      const g = s.geometry;
      if (g?.type === "cylinder" || g?.type === "sphere") return { shape: s, radius: g.radius };
    }
  }
  return null;
}

/**
 * Find and measure the wheels. Returns null when there are none.
 * @param {KinTree} tree
 * @param {Map<string, Transform>} poses root-frame poses at the zero configuration
 * @returns {WheelBase | null}
 */
export function measureWheels(tree, poses) {
  const baseFrame = tree.links.has("base_link") && poses.has("base_link") ? "base_link" : tree.root;
  if (baseFrame === null) return null;
  const baseInv = invert(/** @type {Transform} */ (poses.get(baseFrame)));
  /** @type {Wheel[]} */
  const wheels = [];
  for (const name of tree.order) {
    const j = tree.parentJoint.get(name);
    if (!j || !isWheelJoint(j, tree)) continue;
    const pose = compose(baseInv, /** @type {Transform} */ (poses.get(name)));
    const ws = wheelShape(tree.links.get(name));
    const centre = ws ? apply(pose, ws.shape.origin.xyz) : pose.p;
    const cylinderAxis =
      ws && ws.shape.geometry?.type === "cylinder"
        ? normalize(mulVec(pose.R, column(rpyToMatrix(ws.shape.origin.rpy), 2)))
        : null;
    wheels.push({
      joint: j.name,
      link: name,
      centre,
      axis: normalize(mulVec(pose.R, j.axis)),
      cylinderAxis,
      radius: ws ? ws.radius : null,
      velocityLimit: j.limit?.velocity ?? null,
      label: "",
    });
  }
  if (wheels.length === 0) return null;
  const centroid = /** @type {Vec3} */ (
    [0, 1, 2].map((k) => wheels.reduce((s, w) => s + w.centre[k], 0) / wheels.length)
  );
  const radii = wheels.map((w) => w.radius).filter((r) => r !== null);
  const radius =
    radii.length === wheels.length ? radii.reduce((a, b) => a + b, 0) / radii.length : null;
  /** @type {WheelBase} */
  const base = { baseFrame, kind: "other", wheels, centroid, radius, lx: null, ly: null };
  if (wheels.length === 2) {
    base.kind = "differential";
    const [a, b] = wheels;
    const left = a.centre[1] >= b.centre[1] ? a : b;
    left.label = "L";
    (left === a ? b : a).label = "R";
    base.ly = Math.abs(a.centre[1] - b.centre[1]) / 2;
  } else if (wheels.length === 4) {
    base.kind = "four-wheel";
    for (const w of wheels) {
      const front = w.centre[0] > centroid[0];
      const left = w.centre[1] > centroid[1];
      w.label = front ? (left ? "FL" : "FR") : left ? "RL" : "RR";
    }
    const labels = new Set(wheels.map((w) => w.label));
    if (labels.size === 4) {
      base.lx = wheels.reduce((s, w) => s + Math.abs(w.centre[0] - centroid[0]), 0) / 4;
      base.ly = wheels.reduce((s, w) => s + Math.abs(w.centre[1] - centroid[1]), 0) / 4;
    }
  }
  return base;
}

/**
 * Wheels whose axis is within 5 degrees of the base y axis, and the sign of
 * that alignment (+1: positive joint velocity drives forward).
 * @param {Wheel} w
 * @returns {-1 | 0 | 1}
 */
export function lateralSign(w) {
  const c = dot(w.axis, [0, 1, 0]);
  if (Math.abs(c) < COS_5DEG) return 0;
  return c > 0 ? 1 : -1;
}

/** @param {Vec3 | null} a @param {Vec3} b */
export const parallel = (a, b) => a !== null && Math.abs(dot(a, b)) >= COS_5DEG;

/** @param {number} v */
const fmt = (v) => String(Number(v.toPrecision(6)));

/**
 * mecanum-kinematics TOML config for a four-wheel base.
 * @param {WheelBase | null} base
 * @param {string} robotName
 * @returns {{ ok: true, toml: string } | { ok: false, reason: string }}
 */
export function mecanumToml(base, robotName) {
  if (!base)
    return { ok: false, reason: "no wheel joints found (continuous joints named *wheel*)" };
  if (base.kind !== "four-wheel")
    return { ok: false, reason: `found ${base.wheels.length} wheel(s); a mecanum base needs 4` };
  if (base.lx === null || base.ly === null)
    return { ok: false, reason: "the four wheels are not one per corner (FL, FR, RL, RR)" };
  if (base.radius === null)
    return { ok: false, reason: "not every wheel has a cylinder or sphere geometry to measure" };
  if (!(base.lx > 0) || !(base.ly > 0))
    return { ok: false, reason: "lx and ly must be positive; the wheels are not spread out" };
  const byLabel = new Map(base.wheels.map((w) => [w.label, w]));
  const lines = [
    "# Generated by urdf-inspector from the URDF geometry. Check every value",
    "# against the real robot before using it for odometry.",
    `name = "${robotName.replace(/[^A-Za-z0-9_.-]/g, "-") || "from-urdf"}"`,
    `description = "Measured from URDF '${robotName.replace(/["\\]/g, "")}' (frame ${base.baseFrame})"`,
    "",
    `wheel_radius = ${fmt(base.radius)}      # m, mean of the wheel cylinders`,
    `half_length = ${fmt(base.lx)}       # m, lx: wheel centroid to wheel centre along x`,
    `half_width = ${fmt(base.ly)}        # m, ly: wheel centroid to wheel centre along y`,
    "roller_angle_deg = 45.0   # not in the URDF; 45 for ordinary mecanum wheels",
    "# ticks_per_rev and gear_ratio are not in the URDF; add them for encoder odometry.",
  ];
  const limits = base.wheels.map((w) => w.velocityLimit);
  if (limits.every((v) => v !== null && v > 0)) {
    const vmin = Math.min(.../** @type {number[]} */ (limits));
    lines.push(`max_wheel_speed = ${fmt(vmin)}   # rad/s, smallest wheel <limit velocity>`);
  }
  lines.push('wheel_order = ["FL", "FR", "RL", "RR"]');
  lines.push("");
  lines.push("# URDF joints in that order:");
  for (const label of /** @type {const} */ (["FL", "FR", "RL", "RR"])) {
    const w = /** @type {Wheel} */ (byLabel.get(label));
    const sign = lateralSign(w);
    const note =
      sign === 1
        ? "+y axis"
        : sign === -1
          ? "-y axis: negate its commands and encoder counts"
          : "axis not lateral";
    lines.push(`#   ${label}: ${w.joint} (${note})`);
  }
  const off = [base.centroid[0], base.centroid[1]];
  if (Math.hypot(off[0], off[1]) > 0.001)
    lines.push(
      `# The wheel centroid is at x=${fmt(off[0])}, y=${fmt(off[1])} m in ${base.baseFrame}; mecanum-kinematics assumes the robot centre there.`,
    );
  return { ok: true, toml: lines.join("\n") + "\n" };
}
