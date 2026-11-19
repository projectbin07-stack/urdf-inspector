// Kinematic tree: parent/child maps, root, structural findings (duplicates,
// missing links, several parents, cycles, orphans) and traversal order.

import { finding } from "./findings.js";
import { namedPath } from "./urdf.js";

/** @typedef {import("./urdf.js").Robot} Robot */
/** @typedef {import("./urdf.js").Link} Link */
/** @typedef {import("./urdf.js").Joint} Joint */
/** @typedef {import("./findings.js").Finding} Finding */

/**
 * @typedef {object} KinTree
 * @property {Map<string, Link>} links first link of each name
 * @property {Map<string, Joint>} joints first joint of each name
 * @property {Map<string, Joint>} parentJoint link name -> the joint that has it as child (first one)
 * @property {Map<string, Joint[]>} childJoints link name -> joints that have it as parent, in document order
 * @property {string | null} root the root link of the main tree
 * @property {string[]} order links reachable from the root, depth first, document order
 * @property {string[]} roots every link without a parent joint, in document order
 */

/**
 * @param {Robot} robot
 * @returns {{ tree: KinTree, findings: Finding[] }}
 */
export function buildTree(robot) {
  /** @type {Finding[]} */
  const findings = [];
  /** @type {Map<string, Link>} */
  const links = new Map();
  /** @type {Map<string, Joint>} */
  const joints = new Map();

  if (robot.links.length === 0)
    findings.push(
      finding("no-links", "error", "the robot has no <link> elements", "/robot", {
        kind: "robot",
        name: robot.name,
      }),
    );

  for (const l of robot.links) {
    if (l.name === "") continue;
    if (links.has(l.name))
      findings.push(
        finding(
          "duplicate-link",
          "error",
          `link '${l.name}' is defined more than once`,
          l.path,
          { kind: "link", name: l.name },
          l.line,
        ),
      );
    else links.set(l.name, l);
  }
  for (const j of robot.joints) {
    if (j.name === "") continue;
    if (joints.has(j.name))
      findings.push(
        finding(
          "duplicate-joint",
          "error",
          `joint '${j.name}' is defined more than once`,
          j.path,
          { kind: "joint", name: j.name },
          j.line,
        ),
      );
    else joints.set(j.name, j);
  }

  /** @type {Map<string, Joint>} */
  const parentJoint = new Map();
  /** @type {Map<string, Joint[]>} */
  const childJoints = new Map();
  for (const j of joints.values()) {
    const t = /** @type {const} */ ({ kind: "joint", name: j.name });
    let ok = true;
    for (const [role, name] of /** @type {const} */ ([
      ["parent", j.parent],
      ["child", j.child],
    ])) {
      if (name !== "" && !links.has(name)) {
        ok = false;
        findings.push(
          finding(
            "missing-link",
            "error",
            `joint '${j.name}' has ${role} link '${name}', which does not exist`,
            `${j.path}/${role}`,
            t,
            j.line,
          ),
        );
      }
    }
    if (j.parent === "" || j.child === "") ok = false;
    if (ok && j.parent === j.child) {
      findings.push(
        finding(
          "self-joint",
          "error",
          `joint '${j.name}' has '${j.parent}' as both parent and child`,
          j.path,
          t,
          j.line,
        ),
      );
      ok = false;
    }
    if (!ok) continue;
    const prev = parentJoint.get(j.child);
    if (prev) {
      findings.push(
        finding(
          "multiple-parents",
          "error",
          `link '${j.child}' is the child of joints '${prev.name}' and '${j.name}'`,
          j.path,
          t,
          j.line,
        ),
      );
      continue;
    }
    parentJoint.set(j.child, j);
    const list = childJoints.get(j.parent) ?? [];
    list.push(j);
    childJoints.set(j.parent, list);
  }

  // Cycles: follow parent pointers from every link.
  /** @type {Set<string>} */
  const inCycle = new Set();
  /** @type {Set<string>} */
  const settled = new Set();
  for (const start of links.keys()) {
    /** @type {string[]} */
    const pathLinks = [];
    /** @type {Set<string>} */
    const onPath = new Set();
    let cur = /** @type {string | undefined} */ (start);
    while (cur !== undefined && !settled.has(cur) && !onPath.has(cur)) {
      onPath.add(cur);
      pathLinks.push(cur);
      cur = parentJoint.get(cur)?.parent;
    }
    if (cur !== undefined && onPath.has(cur) && !inCycle.has(cur)) {
      const loop = pathLinks.slice(pathLinks.indexOf(cur));
      for (const l of loop) inCycle.add(l);
      const names = loop.map((l) => /** @type {Joint} */ (parentJoint.get(l)).name);
      const j = /** @type {Joint} */ (parentJoint.get(loop[0]));
      findings.push(
        finding(
          "cycle",
          "error",
          `links ${loop.map((l) => `'${l}'`).join(", ")} form a cycle through joints ${names.map((n) => `'${n}'`).join(", ")}`,
          j.path,
          { kind: "joint", name: j.name },
          j.line,
        ),
      );
    }
    for (const l of pathLinks) settled.add(l);
  }

  const roots = [...links.keys()].filter((l) => !parentJoint.has(l));
  // The main root is the one with the most descendants; ties go to document order.
  /** @param {string} r */
  const subtree = (r) => {
    /** @type {string[]} */
    const out = [];
    /** @param {string} l */
    const visit = (l) => {
      out.push(l);
      for (const j of childJoints.get(l) ?? []) if (!inCycle.has(j.child)) visit(j.child);
    };
    visit(r);
    return out;
  };
  let root = null;
  let order = /** @type {string[]} */ ([]);
  for (const r of roots) {
    const s = subtree(r);
    if (s.length > order.length) {
      root = r;
      order = s;
    }
  }
  for (const r of roots) {
    if (r === root) continue;
    const s = subtree(r);
    const l = /** @type {Link} */ (links.get(r));
    findings.push(
      finding(
        "orphan-link",
        "error",
        s.length > 1
          ? `link '${r}' and its ${s.length - 1} descendant(s) are not connected to root '${root}'`
          : `link '${r}' is not connected to root '${root}' by any joint`,
        namedPath("link", r),
        { kind: "link", name: r },
        l.line,
      ),
    );
  }
  if (root === null && links.size > 0)
    findings.push(
      finding("cycle", "error", "no root link: every link has a parent joint", "/robot", {
        kind: "robot",
        name: robot.name,
      }),
    );

  return { tree: { links, joints, parentJoint, childJoints, root, order, roots }, findings };
}

/**
 * Joints on the path from the root down to `link`, root side first.
 * @param {KinTree} tree
 * @param {string} link
 * @returns {Joint[]}
 */
export function chainTo(tree, link) {
  /** @type {Joint[]} */
  const out = [];
  /** @type {Set<string>} */
  const seen = new Set();
  let cur = tree.parentJoint.get(link);
  while (cur && !seen.has(cur.name)) {
    seen.add(cur.name);
    out.unshift(cur);
    cur = tree.parentJoint.get(cur.parent);
  }
  return out;
}
