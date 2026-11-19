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
 * Inspect an already-parsed XML tree (from parseXml or DOMParser).
 * @param {import("./xml.js").XmlElement} root
 * @returns {Report}
 */
export function inspectXml(root) {
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
 * Parse and inspect URDF source text with the built-in XML reader.
 * @param {string} text
 * @returns {Report}
 */
export function inspectText(text) {
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
  return inspectXml(root);
}
