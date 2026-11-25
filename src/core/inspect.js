// One entry point for the CLI and the browser: XML -> model -> tree ->
// checks -> sorted findings and a summary. Deterministic for a given input.

import { checkFrames } from "./checks/frames.js";
import { checkGeometry } from "./checks/geometry.js";
import { checkInertia } from "./checks/inertia.js";
import { checkIntegration } from "./checks/integration.js";
import { checkJoints } from "./checks/joints.js";
import { makeContext } from "./checks/context.js";
import { checkWheels } from "./checks/wheels.js";
import { massProperties } from "./fk.js";
import { countBySeverity, finding, sortFindings } from "./findings.js";
import { buildTree } from "./tree.js";
import { readUrdf } from "./urdf.js";
import { parseXml, XmlError } from "./xml.js";

/** @typedef {import("./findings.js").Finding} Finding */

/**
 * @typedef {object} Summary
 * @property {string} name
 * @property {number} links
 * @property {number} joints
 * @property {Record<string, number>} jointTypes
 * @property {string | null} root
 * @property {number} totalMass kg, links with positive mass
 * @property {import("./math.js").Vec3 | null} com centre of mass in the root frame, zero configuration
 * @property {number} errors
 * @property {number} warnings
 * @property {number} infos
 */

/**
 * @typedef {object} Report
 * @property {import("./urdf.js").Robot | null} robot
 * @property {import("./checks/context.js").Context | null} context
 * @property {Finding[]} findings sorted by severity
 * @property {Summary} summary
 */

/**
 * @param {Finding[]} findings
 * @param {Partial<Summary>} extra
 * @returns {Summary}
 */
function summarise(findings, extra) {
  const c = countBySeverity(findings);
  return {
    name: "",
    links: 0,
    joints: 0,
    jointTypes: {},
    root: null,
    totalMass: 0,
    com: null,
    ...extra,
    errors: c.error,
    warnings: c.warning,
    infos: c.info,
  };
}

/**
 * @typedef {object} InspectOptions
 * @property {(filename: string) => true | string} [meshExists] Look a mesh
 *   filename up (as written in the URDF). Return true when it exists, or the
 *   reason it was not found; each missing mesh becomes a `mesh-missing`
 *   warning. Without it no file is looked up.
 */

/**
 * The file of the top-level element a finding's path starts with, or of the
 * root element.
 * @param {import("./xml.js").XmlElement} root
 * @returns {(path: string) => string | undefined}
 */
function sourceLookup(root) {
  /** @type {Map<string, string | undefined>} */
  const byKey = new Map();
  for (const c of root.children) {
    const n = c.attrs.name ?? c.attrs.reference;
    const key = `${c.name}|${n}`;
    if (n !== undefined && !byKey.has(key)) byKey.set(key, c.source);
  }
  return (path) => {
    const m = /^\/robot\/([^[/]+)\[@(?:name|reference)='((?:[^']|&apos;)*)'\]/.exec(path);
    const s = m ? byKey.get(`${m[1]}|${m[2].replace(/&apos;/g, "'")}`) : undefined;
    return s ?? root.source;
  };
}

/**
 * Whether any element of the tree carries a source file.
 * @param {import("./xml.js").XmlElement} el
 * @returns {boolean}
 */
const hasSource = (el) => el.source !== undefined || el.children.some(hasSource);

/**
 * Inspect an already-parsed XML tree (from parseXml or DOMParser).
 * @param {import("./xml.js").XmlElement} root
 * @param {InspectOptions} [options]
 * @returns {Report}
 */
export function inspectXml(root, options = {}) {
  const report = inspectTree(root, options);
  if (hasSource(root)) {
    const sourceOf = sourceLookup(root);
    for (const f of report.findings) {
      const s = sourceOf(f.path);
      if (s !== undefined) f.source = s;
    }
  }
  return report;
}

/**
 * @param {import("./xml.js").XmlElement} root
 * @param {InspectOptions} options
 * @returns {Report}
 */
function inspectTree(root, options) {
  const { robot, findings: input } = readUrdf(root);
  if (!robot) {
    const findings = sortFindings(input);
    return { robot: null, context: null, findings, summary: summarise(findings, {}) };
  }
  const { tree, findings: structure } = buildTree(robot);
  const ctx = makeContext(robot, tree);
  const findings = sortFindings([
    ...input,
    ...structure,
    ...checkFrames(ctx),
    ...checkJoints(ctx),
    ...checkInertia(ctx),
    ...checkGeometry(ctx),
    ...checkIntegration(ctx),
    ...checkWheels(ctx),
    ...missingMeshes(robot, options.meshExists),
  ]);
  /** @type {Record<string, number>} */
  const jointTypes = {};
  for (const j of robot.joints)
    jointTypes[j.type || "(none)"] = (jointTypes[j.type || "(none)"] ?? 0) + 1;
  const mp = massProperties(tree, ctx.poses);
  return {
    robot,
    context: ctx,
    findings,
    summary: summarise(findings, {
      name: robot.name,
      links: robot.links.length,
      joints: robot.joints.length,
      jointTypes,
      root: tree.root,
      totalMass: mp.mass,
      com: mp.com,
    }),
  };
}

/**
 * @param {import("./urdf.js").Robot} robot
 * @param {InspectOptions["meshExists"]} exists
 * @returns {Finding[]}
 */
function missingMeshes(robot, exists) {
  /** @type {Finding[]} */
  const out = [];
  if (!exists) return out;
  for (const link of robot.links)
    for (const kind of /** @type {const} */ (["visual", "collision"]))
      for (const s of kind === "visual" ? link.visuals : link.collisions) {
        if (s.geometry?.type !== "mesh" || s.geometry.filename === "") continue;
        const r = exists(s.geometry.filename);
        if (r === true) continue;
        out.push(
          finding(
            "mesh-missing",
            "warning",
            `${kind} mesh '${s.geometry.filename}' of link '${link.name}' was not found: ${r}`,
            s.path,
            { kind: "link", name: link.name },
            link.line,
          ),
        );
      }
  return out;
}

/**
 * Parse and inspect URDF source text with the built-in XML reader.
 * @param {string} text
 * @param {InspectOptions} [options]
 * @returns {Report}
 */
export function inspectText(text, options = {}) {
  let root;
  try {
    root = parseXml(text);
  } catch (e) {
    if (!(e instanceof XmlError)) throw e;
    const findings = [
      finding("xml-syntax", "error", e.message, "/", { kind: "element", name: "" }, e.line),
    ];
    return { robot: null, context: null, findings, summary: summarise(findings, {}) };
  }
  return inspectXml(root, options);
}
