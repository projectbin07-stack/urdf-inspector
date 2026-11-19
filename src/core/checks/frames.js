// Frame-name and convention checks from REP 103, REP 105 and kdl_parser.

import { finding } from "../findings.js";
import { column, compose, dot, invert, mulVec } from "../math.js";
import { namedPath } from "../urdf.js";
import { fmt } from "./context.js";

/** @typedef {import("./context.js").Context} Context */
/** @typedef {import("../findings.js").Finding} Finding */
/** @typedef {import("../math.js").Transform} Transform */

const WORLD_FRAMES = ["odom", "map", "earth"];

/**
 * @param {Context} ctx
 * @returns {Finding[]}
 */
export function checkFrames(ctx) {
  const { tree, poses } = ctx;
  /** @type {Finding[]} */
  const out = [];
  /** @param {string} name @returns {Finding["target"]} */
  const link = (name) => ({ kind: "link", name });
  const lineOf = (/** @type {string} */ name) => tree.links.get(name)?.line ?? 0;

  if (tree.links.size > 0 && !tree.links.has("base_link"))
    out.push(
      finding(
        "no-base-link",
        "warning",
        `no link is called base_link (root is '${tree.root ?? "none"}')`,
        "/robot",
        { kind: "robot", name: ctx.robot.name },
      ),
    );

  for (const w of WORLD_FRAMES) {
    if (tree.links.has(w))
      out.push(
        finding(
          "odom-in-urdf",
          "warning",
          `link '${w}' is a REP 105 world frame and should be published by ${w === "odom" ? "odometry" : "localisation"}, not by the URDF`,
          namedPath("link", w),
          link(w),
          lineOf(w),
        ),
      );
  }

  if (tree.root !== null && tree.links.get(tree.root)?.inertial)
    out.push(
      finding(
        "root-inertia",
        "warning",
        `root link '${tree.root}' has an <inertial>; KDL ignores it`,
        `${namedPath("link", tree.root)}/inertial`,
        link(tree.root),
        lineOf(tree.root),
      ),
    );

  for (const j of tree.joints.values())
    if (j.type === "floating" || j.type === "planar")
      out.push(
        finding(
          "kdl-unsupported-joint",
          "warning",
          `${j.type} joint '${j.name}' is treated as fixed by kdl_parser and robot_state_publisher`,
          j.path,
          { kind: "joint", name: j.name },
          j.line,
        ),
      );

  // base_footprint <-> base_link: rigid, no roll or pitch between them.
  const fp = poses.get("base_footprint");
  const bl = poses.get("base_link");
  if (tree.links.has("base_footprint") && tree.links.has("base_link") && fp && bl) {
    const rel = compose(invert(fp), bl);
    const between = jointsBetween(ctx, "base_footprint", "base_link");
    const moving = between.filter((j) => j.type !== "fixed");
    const z = column(rel.R, 2);
    const tilt = Math.acos(Math.max(-1, Math.min(1, z[2])));
    if (moving.length > 0)
      out.push(
        finding(
          "footprint-relation",
          "warning",
          `base_footprint and base_link are connected through non-fixed joint(s) ${moving.map((j) => `'${j.name}'`).join(", ")}`,
          moving[0].path,
          { kind: "joint", name: moving[0].name },
          moving[0].line,
        ),
      );
    else if (tilt > 1e-3)
      out.push(
        finding(
          "footprint-relation",
          "warning",
          `base_link is tilted ${fmt((tilt * 180) / Math.PI)} deg relative to base_footprint; they should differ only by a translation and yaw`,
          between[0]?.path ?? namedPath("link", "base_footprint"),
          link("base_footprint"),
          lineOf("base_footprint"),
        ),
      );
    // Footprint on the floor: lowest wheel contact at z = 0.
    const wb = ctx.wheels;
    if (moving.length === 0 && wb && wb.radius !== null) {
      const base = /** @type {Transform} */ (poses.get(wb.baseFrame));
      const toFp = compose(invert(fp), base);
      const contacts = wb.wheels.map((w) => {
        const c = mulVec(toFp.R, w.centre);
        return c[2] + toFp.p[2] - /** @type {number} */ (w.radius);
      });
      const zc = Math.min(...contacts);
      if (Math.abs(zc) > 0.005)
        out.push(
          finding(
            "footprint-height",
            "warning",
            `lowest wheel contact is at z = ${fmt(zc)} m in base_footprint; expected 0`,
            namedPath("link", "base_footprint"),
            link("base_footprint"),
            lineOf("base_footprint"),
            { contactZ: zc },
          ),
        );
    }
  }

  // Optical frames: z forward, x right, y down relative to the body frame
  // they hang from (the nearest ancestor that is not itself optical).
  for (const name of tree.order) {
    if (!/_optical(_frame)?$/.test(name)) continue;
    let anc = tree.parentJoint.get(name)?.parent;
    while (anc && /_optical(_frame)?$/.test(anc)) anc = tree.parentJoint.get(anc)?.parent;
    const pose = poses.get(name);
    const ap = anc ? poses.get(anc) : undefined;
    if (!anc || !pose || !ap) continue;
    const R = compose(invert(ap), pose).R;
    const ok =
      dot(column(R, 2), [1, 0, 0]) > 0.999 &&
      dot(column(R, 0), [0, -1, 0]) > 0.999 &&
      dot(column(R, 1), [0, 0, -1]) > 0.999;
    if (!ok) {
      const pj = /** @type {import("../urdf.js").Joint} */ (tree.parentJoint.get(name));
      out.push(
        finding(
          "optical-frame",
          "warning",
          `'${name}' axes relative to '${anc}' are not z forward, x right, y down; the usual joint rpy is "-1.5708 0 -1.5708"`,
          `${pj.path}/origin`,
          { kind: "joint", name: pj.name },
          pj.line,
        ),
      );
    }
  }
  for (const name of tree.order) {
    if (!/camera/i.test(name) || /optical/i.test(name)) continue;
    if (tree.parentJoint.get(name) && /camera/i.test(tree.parentJoint.get(name)?.parent ?? ""))
      continue; // a sub-frame of another camera link
    const hasOptical = descendantsOf(ctx, name).some((d) => /_optical(_frame)?$/.test(d));
    if (!hasOptical)
      out.push(
        finding(
          "camera-no-optical",
          "info",
          `camera link '${name}' has no *_optical_frame child for image data`,
          namedPath("link", name),
          link(name),
          lineOf(name),
        ),
      );
  }
  return out;
}

/**
 * Joints on the tree path between two links (either may be the ancestor).
 * @param {Context} ctx
 * @param {string} a
 * @param {string} b
 * @returns {import("../urdf.js").Joint[]}
 */
export function jointsBetween(ctx, a, b) {
  /** @param {string} l */
  const up = (l) => {
    /** @type {import("../urdf.js").Joint[]} */
    const chain = [];
    let cur = ctx.tree.parentJoint.get(l);
    const seen = new Set();
    while (cur && !seen.has(cur.name)) {
      seen.add(cur.name);
      chain.push(cur);
      cur = ctx.tree.parentJoint.get(cur.parent);
    }
    return chain;
  };
  const ca = up(a);
  const cb = up(b);
  const inA = new Set(ca.map((j) => j.name));
  const common = cb.find((j) => inA.has(j.name));
  const trimA = common
    ? ca.slice(
        0,
        ca.findIndex((j) => j.name === common.name),
      )
    : ca;
  const trimB = common
    ? cb.slice(
        0,
        cb.findIndex((j) => j.name === common.name),
      )
    : cb;
  return [...trimA, ...trimB];
}

/** @param {Context} ctx @param {string} name @returns {string[]} */
function descendantsOf(ctx, name) {
  /** @type {string[]} */
  const out = [];
  for (const j of ctx.tree.childJoints.get(name) ?? [])
    out.push(j.child, ...descendantsOf(ctx, j.child));
  return out;
}
