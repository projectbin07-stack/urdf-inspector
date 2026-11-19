// Text and JSON renderings shared by the CLI (and tested directly). Output
// is deterministic: numbers are rounded to fixed significant digits and
// nothing depends on time or locale.

import { forwardKinematics, massProperties, resolvePositions } from "./fk.js";
import { matrixToRpy } from "./math.js";
import { VERSION } from "./version.js";

/** @typedef {import("./inspect.js").Report} Report */
/** @typedef {import("./tree.js").KinTree} KinTree */

/** @param {number} v @param {number} [d] */
export const fx = (v, d = 4) => {
  const s = (Math.abs(v) < 0.5 * 10 ** -d ? 0 : v).toFixed(d);
  return s === "-0." + "0".repeat(d) ? s.slice(1) : s;
};
/** @param {number} v */
const g6 = (v) => String(Number(v.toPrecision(6)));

/**
 * @param {Report} r
 * @param {string} file
 */
export function checkJson(r, file) {
  return {
    tool: "urdf-inspector",
    version: VERSION,
    file,
    summary: {
      ...r.summary,
      totalMass: Number(r.summary.totalMass.toPrecision(6)),
      com: r.summary.com ? r.summary.com.map((v) => Number(v.toPrecision(6))) : null,
    },
    findings: r.findings,
  };
}

/** @param {Report} r */
export function summaryLines(r) {
  const s = r.summary;
  if (!r.robot) return [];
  const types = Object.entries(s.jointTypes)
    .sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0))
    .map(([t, n]) => `${n} ${t}`)
    .join(", ");
  const lines = [
    `robot '${s.name}': ${s.links} links, ${s.joints} joints${types ? ` (${types})` : ""}, root ${s.root ?? "none"}`,
  ];
  if (s.com)
    lines.push(
      `mass ${g6(s.totalMass)} kg, centre of mass (${s.com.map((v) => fx(v)).join(", ")}) m in ${s.root}`,
    );
  else lines.push("mass 0 kg: no link has a positive mass");
  return lines;
}

/**
 * @param {Report} r
 * @param {string} file
 */
export function checkText(r, file) {
  const out = [`${file}`];
  out.push(...summaryLines(r).map((l) => `  ${l}`));
  const s = r.summary;
  out.push(`  ${s.errors} error(s), ${s.warnings} warning(s), ${s.infos} note(s)`, "");
  const tag = { error: "ERROR", warning: "WARN ", info: "INFO " };
  for (const f of r.findings) {
    out.push(`${tag[f.severity]} ${f.id}${f.line ? `  (line ${f.line})` : ""}`);
    out.push(`      ${f.message}`);
    out.push(`      at ${f.path}`);
  }
  return out.join("\n") + "\n";
}

/**
 * ASCII tree of links and joints.
 * @param {KinTree} tree
 */
export function treeText(tree) {
  /** @type {string[]} */
  const out = [];
  if (tree.root === null) return "(no root link)\n";
  /** @param {string} link @param {string} prefix */
  const visit = (link, prefix) => {
    const kids = tree.childJoints.get(link) ?? [];
    kids.forEach((j, k) => {
      const last = k === kids.length - 1;
      const mov = j.type === "revolute" || j.type === "continuous" || j.type === "prismatic";
      const axis = mov ? `, axis ${j.axis.map(g6).join(" ")}` : "";
      let lim = "";
      if (j.limit && (j.type === "revolute" || j.type === "prismatic"))
        lim = `, [${g6(j.limit.lower ?? 0)}, ${g6(j.limit.upper ?? 0)}]`;
      const mimic = j.mimic ? `, mimics ${j.mimic.joint}` : "";
      out.push(
        `${prefix}${last ? "`-- " : "|-- "}${j.name} (${j.type}${axis}${lim}${mimic}) -> ${j.child}`,
      );
      visit(j.child, prefix + (last ? "    " : "|   "));
    });
  };
  out.push(tree.root);
  visit(tree.root, "");
  const others = tree.roots.filter((r) => r !== tree.root);
  for (const o of others) {
    out.push(`${o}  (not connected)`);
    visit(o, "");
  }
  return out.join("\n") + "\n";
}

/**
 * Link poses for a joint configuration.
 * @param {KinTree} tree
 * @param {Record<string, number>} q
 */
export function fkData(tree, q) {
  const poses = forwardKinematics(tree, q);
  const pos = resolvePositions(tree, q);
  const mp = massProperties(tree, poses);
  /** @type {Record<string, number>} */
  const joints = {};
  for (const j of tree.joints.values())
    if (j.type === "revolute" || j.type === "continuous" || j.type === "prismatic")
      joints[j.name] = Number((pos.get(j.name) ?? 0).toPrecision(12));
  return {
    frame: tree.root,
    joints,
    links: tree.order.map((name) => {
      const p = /** @type {import("./math.js").Transform} */ (poses.get(name));
      return {
        name,
        xyz: p.p.map((v) => Number(fx(v, 6))),
        rpy: matrixToRpy(p.R).map((v) => Number(fx(v, 6))),
      };
    }),
    mass: Number(mp.mass.toPrecision(6)),
    com: mp.com ? mp.com.map((v) => Number(fx(v, 6))) : null,
  };
}

/** @param {ReturnType<typeof fkData>} d */
export function fkText(d) {
  const w = Math.max(4, ...d.links.map((l) => l.name.length));
  const out = [];
  const qs = Object.entries(d.joints);
  out.push(
    `joint positions: ${qs.length ? qs.map(([n, v]) => `${n}=${g6(v)}`).join(", ") : "(none)"}`,
  );
  out.push(`poses in ${d.frame} (m, rad; rpy as in URDF)`);
  out.push(
    `${"link".padEnd(w)}  ${"x".padStart(9)} ${"y".padStart(9)} ${"z".padStart(9)}   ${"roll".padStart(8)} ${"pitch".padStart(8)} ${"yaw".padStart(8)}`,
  );
  for (const l of d.links)
    out.push(
      `${l.name.padEnd(w)}  ${l.xyz.map((v) => fx(v).padStart(9)).join(" ")}   ${l.rpy.map((v) => fx(v).padStart(8)).join(" ")}`,
    );
  out.push(
    d.com
      ? `mass ${g6(d.mass)} kg, centre of mass (${d.com.map((v) => fx(v)).join(", ")})`
      : "mass 0 kg",
  );
  return out.join("\n") + "\n";
}
