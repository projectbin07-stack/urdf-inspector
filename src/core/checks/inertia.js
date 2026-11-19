// Mass and inertia checks, including the links Gazebo's URDF to SDF
// conversion drops.

import { finding } from "../findings.js";
import { inertiaValidity, primitiveInertia, rotateTensor, volume } from "../inertia.js";
import { fmt, singlePrimitive } from "./context.js";

/** @typedef {import("./context.js").Context} Context */
/** @typedef {import("../findings.js").Finding} Finding */
/** @typedef {import("../urdf.js").Link} Link */

export const DENSITY_MIN = 10; // kg/m^3, lighter than any solid robot part (foam is ~30)
export const DENSITY_MAX = 25000; // kg/m^3, denser than tungsten (19300)
export const INERTIA_RATIO = 10;

/** @param {Link} link */
const ownMass = (link) => {
  const m = link.inertial?.mass;
  return m !== null && m !== undefined && m > 0 ? m : 0;
};

/**
 * @param {Context} ctx
 * @returns {Finding[]}
 */
export function checkInertia(ctx) {
  /** @type {Finding[]} */
  const out = [];
  const { tree } = ctx;
  for (const link of tree.links.values()) {
    const t = /** @type {const} */ ({ kind: "link", name: link.name });
    /** @param {string} id @param {import("../findings.js").Severity} sev @param {string} msg @param {string} path @param {Record<string, unknown>} [data] */
    const add = (id, sev, msg, path, data) =>
      out.push(finding(id, sev, msg, path, t, link.line, data));
    const pj = tree.parentJoint.get(link.name);
    const movedBy =
      pj && (pj.type === "revolute" || pj.type === "continuous" || pj.type === "prismatic");
    const inert = link.inertial;
    if (!inert) {
      if (movedBy)
        add(
          "inertial-missing",
          "warning",
          `link '${link.name}' is moved by ${pj.type} joint '${pj.name}' but has no <inertial>`,
          link.path,
        );
      continue;
    }
    const m = inert.mass;
    if (m !== null && m <= 0) {
      add(
        "mass-nonpositive",
        "error",
        `link '${link.name}' mass is ${fmt(m)} kg`,
        `${inert.path}/mass`,
      );
      continue;
    }
    if (m === null || inert.inertia === null) continue;
    const tp = `${inert.path}/inertia`;
    const v = inertiaValidity(inert.inertia);
    const pm = v.principal.map(fmt).join(", ");
    if (!v.positiveDefinite) {
      add(
        "inertia-not-pd",
        "error",
        `link '${link.name}' inertia has principal moments ${pm}; all must be positive`,
        tp,
        { principal: v.principal },
      );
      continue;
    }
    if (!v.triangle) {
      add(
        "inertia-triangle",
        "error",
        `link '${link.name}' principal moments ${pm}: ${fmt(v.principal[0])} + ${fmt(v.principal[1])} < ${fmt(v.principal[2])}`,
        tp,
        { principal: v.principal },
      );
      continue;
    }
    const shape = singlePrimitive(link);
    const g = shape?.geometry;
    if (!shape || !g) continue;
    const vol = volume(g);
    if (vol !== null && vol > 0) {
      const rho = m / vol;
      if (rho < DENSITY_MIN || rho > DENSITY_MAX)
        add(
          "density-implausible",
          "warning",
          `link '${link.name}': ${fmt(m)} kg in a ${g.type} of ${fmt(vol)} m^3 is ${fmt(rho)} kg/m^3`,
          `${inert.path}/mass`,
          { density: rho, volume: vol },
        );
    }
    const ref = primitiveInertia(g, m);
    if (!ref) continue;
    const rv = inertiaValidity(ref);
    if (!(rv.principal[0] > 0)) continue;
    const ratios = v.principal.map((p, k) => p / rv.principal[k]);
    const worst = ratios.reduce((a, b) => (Math.abs(Math.log(b)) > Math.abs(Math.log(a)) ? b : a));
    if (worst > INERTIA_RATIO || worst < 1 / INERTIA_RATIO) {
      const reference = rotateTensor(ref, shape.origin.rpy, inert.origin.rpy);
      const r = reference;
      add(
        "inertia-implausible",
        "warning",
        `link '${link.name}' principal moments ${pm} are up to ${fmt(worst > 1 ? worst : 1 / worst)}x ${worst > 1 ? "larger" : "smaller"} than a solid ${g.type} of ${fmt(m)} kg (${rv.principal.map(fmt).join(", ")}); ` +
          `reference ixx=${fmt(r.ixx)} ixy=${fmt(r.ixy)} ixz=${fmt(r.ixz)} iyy=${fmt(r.iyy)} iyz=${fmt(r.iyz)} izz=${fmt(r.izz)}`,
        tp,
        { reference, principal: v.principal, referencePrincipal: rv.principal },
      );
    }
  }
  out.push(...gazeboDropped(ctx));
  return out;
}

/**
 * Fixed joints that Gazebo keeps (not lumped) because of a
 * <preserveFixedJoint> or <disableFixedJointLumping> extension.
 * @param {Context} ctx
 */
function preservedJoints(ctx) {
  /** @type {Set<string>} */
  const keep = new Set();
  for (const g of ctx.robot.gazebo)
    if (
      g.elements.includes("preserveFixedJoint") ||
      g.elements.includes("disableFixedJointLumping")
    )
      keep.add(g.reference);
  return keep;
}

/**
 * @param {Context} ctx
 * @returns {Finding[]}
 */
function gazeboDropped(ctx) {
  const { tree } = ctx;
  const keep = preservedJoints(ctx);
  /** @param {import("../urdf.js").Joint} j */
  const lumped = (j) => j.type === "fixed" && !keep.has(j.name);
  /** @type {Map<string, number>} */
  const memo = new Map();
  /** @param {string} name @returns {number} */
  const effective = (name) => {
    const cached = memo.get(name);
    if (cached !== undefined) return cached;
    memo.set(name, 0);
    const link = tree.links.get(name);
    let m = link ? ownMass(link) : 0;
    for (const j of tree.childJoints.get(name) ?? []) if (lumped(j)) m += effective(j.child);
    memo.set(name, m);
    return m;
  };
  /** @param {string} name @returns {number} */
  const descendants = (name) => {
    let c = 0;
    for (const j of tree.childJoints.get(name) ?? []) c += 1 + descendants(j.child);
    return c;
  };
  /** @type {Finding[]} */
  const out = [];
  for (const name of tree.order) {
    if (name === "world") continue;
    const pj = tree.parentJoint.get(name);
    if (pj && lumped(pj)) continue;
    if (effective(name) > 0) continue;
    const link = /** @type {Link} */ (tree.links.get(name));
    const d = descendants(name);
    const kids = (tree.childJoints.get(name) ?? []).length;
    out.push(
      finding(
        "gazebo-dropped",
        "warning",
        `link '${name}' has no mass ${pj ? `behind ${pj.type} joint '${pj.name}'` : "as the root"}` +
          (kids > 0
            ? `, nor do its fixed children; Gazebo drops it with ${kids} child joint(s) and ${d} descendant link(s)`
            : "; Gazebo drops it"),
        link.path,
        { kind: "link", name },
        link.line,
      ),
    );
  }
  return out;
}
