// Geometry, mesh reference and unit sanity checks.

import { finding } from "../findings.js";
import { norm } from "../math.js";
import { fmt } from "./context.js";

/** @typedef {import("./context.js").Context} Context */
/** @typedef {import("../findings.js").Finding} Finding */

export const MAX_PRIMITIVE_M = 10; // a box side, cylinder length or radius above this is suspicious
export const MAX_OFFSET_M = 50; // a joint or shape offset above this is suspicious
const MESH_FORMATS = ["stl", "dae", "obj"];

/**
 * @param {Context} ctx
 * @returns {Finding[]}
 */
export function checkGeometry(ctx) {
  /** @type {Finding[]} */
  const out = [];
  let mmReported = false;
  /** @param {string} msg @param {string} path @param {Finding["target"]} t @param {number} line */
  const mm = (msg, path, t, line) => {
    if (mmReported) return;
    mmReported = true;
    out.push(finding("units-mm", "warning", msg, path, t, line));
  };
  /** @param {import("../math.js").Vec3} rpy @param {string} path @param {Finding["target"]} t @param {number} line @param {string} what */
  const degrees = (rpy, path, t, line, what) => {
    const big = rpy.filter((v) => Math.abs(v) > 2 * Math.PI + 1e-3);
    if (big.length > 0)
      out.push(
        finding(
          "rpy-degrees",
          "warning",
          `${what} rpy (${rpy.map(fmt).join(" ")}) has angles above 2*pi rad; degrees?`,
          path,
          t,
          line,
        ),
      );
  };

  for (const link of ctx.tree.links.values()) {
    const t = /** @type {const} */ ({ kind: "link", name: link.name });
    if (link.visuals.length > 0 && link.collisions.length === 0)
      out.push(
        finding(
          "collision-missing",
          "warning",
          `link '${link.name}' has ${link.visuals.length} visual(s) and no collision`,
          link.path,
          t,
          link.line,
        ),
      );
    if (link.inertial)
      degrees(
        link.inertial.origin.rpy,
        `${link.inertial.path}/origin`,
        t,
        link.line,
        `link '${link.name}' inertial`,
      );
    for (const s of [...link.visuals, ...link.collisions]) {
      const what = `link '${link.name}' ${s.kind}`;
      degrees(s.origin.rpy, `${s.path}/origin`, t, link.line, what);
      if (norm(s.origin.xyz) > MAX_OFFSET_M)
        mm(
          `${what} offset is ${fmt(norm(s.origin.xyz))} m; millimetres?`,
          `${s.path}/origin`,
          t,
          link.line,
        );
      const g = s.geometry;
      if (!g) continue;
      const gp = `${s.path}/geometry/${g.type === "unknown" ? g.tag : g.type}`;
      /** @type {number[]} */
      let dims = [];
      if (g.type === "box") dims = g.size;
      else if (g.type === "cylinder") dims = [g.radius, g.length];
      else if (g.type === "sphere") dims = [g.radius];
      if (dims.some((d) => !(d > 0)))
        out.push(
          finding(
            "geometry-nonpositive",
            "error",
            `${what} ${g.type} has dimension(s) ${dims.map(fmt).join(" ")}; all must be > 0`,
            gp,
            t,
            link.line,
          ),
        );
      else if (dims.some((d) => d > MAX_PRIMITIVE_M))
        mm(
          `${what} ${g.type} dimensions ${dims.map(fmt).join(" ")} m; millimetres?`,
          gp,
          t,
          link.line,
        );
      if (g.type === "mesh") out.push(...meshFindings(g, what, gp, t, link.line));
    }
  }
  for (const j of ctx.tree.joints.values()) {
    const t = /** @type {const} */ ({ kind: "joint", name: j.name });
    degrees(j.origin.rpy, `${j.path}/origin`, t, j.line, `joint '${j.name}'`);
    const d = norm(j.origin.xyz);
    if (d > MAX_OFFSET_M)
      mm(
        `joint '${j.name}' origin is ${fmt(d)} m from its parent; millimetres?`,
        `${j.path}/origin`,
        t,
        j.line,
      );
  }
  return out;
}

/**
 * @param {{ filename: string, scale: import("../math.js").Vec3 }} g
 * @param {string} what
 * @param {string} path
 * @param {Finding["target"]} t
 * @param {number} line
 * @returns {Finding[]}
 */
export function meshFindings(g, what, path, t, line) {
  /** @type {Finding[]} */
  const out = [];
  const f = g.filename;
  const scheme = /^([A-Za-z][A-Za-z0-9+.-]*):\/\//.exec(f)?.[1]?.toLowerCase();
  if (f !== "") {
    let problem = "";
    if (scheme === undefined)
      problem =
        /^[A-Za-z]:[\\/]/.test(f) || f.startsWith("/")
          ? "is an absolute path without file://; use package://"
          : "is a relative path, resolved against the working directory; use package://";
    else if (scheme === "http" || scheme === "https")
      problem = "is fetched over the network at load time, if the loader supports it at all";
    else if (scheme === "model") problem = "uses model://, which only Gazebo resolves; RViz cannot";
    else if (scheme !== "package" && scheme !== "file")
      problem = `uses unsupported scheme ${scheme}://`;
    if (problem)
      out.push(finding("mesh-scheme", "warning", `${what} mesh '${f}' ${problem}`, path, t, line));
    const ext = /\.([A-Za-z0-9]+)$/.exec(f.split(/[?#]/)[0])?.[1]?.toLowerCase();
    if (!ext || !MESH_FORMATS.includes(ext))
      out.push(
        finding(
          "mesh-format",
          "warning",
          `${what} mesh '${f}' is ${ext ? `.${ext}` : "without an extension"}; use STL, DAE or OBJ`,
          path,
          t,
          line,
        ),
      );
  }
  if (g.scale.some((s) => s === 0))
    out.push(
      finding(
        "mesh-scale",
        "error",
        `${what} mesh scale (${g.scale.map(fmt).join(" ")}) has a zero`,
        path,
        t,
        line,
      ),
    );
  else if (g.scale.some((s) => s < 0))
    out.push(
      finding(
        "mesh-scale",
        "warning",
        `${what} mesh scale (${g.scale.map(fmt).join(" ")}) is negative, which mirrors the mesh and flips its normals`,
        path,
        t,
        line,
      ),
    );
  return out;
}
