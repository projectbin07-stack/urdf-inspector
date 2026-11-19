// Layout of the kinematic tree diagram: an indented tree (one row per link,
// children below their parent, indented by depth), which stays readable on
// a phone. Collapsed links hide their descendants. Links outside the main
// tree (orphans, cycles) are listed after it at depth 0.

/** @typedef {import("./tree.js").KinTree} KinTree */

/**
 * @typedef {object} Row
 * @property {string} link
 * @property {number} depth
 * @property {number} row
 * @property {string | null} joint incoming joint name
 * @property {string | null} jointType
 * @property {number | null} parentRow row of the parent link
 * @property {number} children number of child joints
 * @property {boolean} collapsed
 * @property {boolean} connected false for links outside the main tree
 */

/**
 * @param {KinTree} tree
 * @param {Set<string>} collapsed links whose subtree is hidden
 * @returns {Row[]}
 */
export function layoutTree(tree, collapsed) {
  /** @type {Row[]} */
  const rows = [];
  /** @type {Set<string>} */
  const placed = new Set();
  /**
   * @param {string} link
   * @param {number} depth
   * @param {number | null} parentRow
   * @param {boolean} connected
   */
  const visit = (link, depth, parentRow, connected) => {
    if (placed.has(link)) return;
    placed.add(link);
    const kids = tree.childJoints.get(link) ?? [];
    const pj = parentRow === null ? null : (tree.parentJoint.get(link) ?? null);
    const row = rows.length;
    const isCollapsed = collapsed.has(link) && kids.length > 0;
    rows.push({
      link,
      depth,
      row,
      joint: pj?.name ?? null,
      jointType: pj?.type ?? null,
      parentRow,
      children: kids.length,
      collapsed: isCollapsed,
      connected,
    });
    if (isCollapsed) {
      hide(link);
      return;
    }
    for (const j of kids) visit(j.child, depth + 1, row, connected);
  };
  /** Mark the descendants of a collapsed link as placed. @param {string} link */
  const hide = (link) => {
    for (const j of tree.childJoints.get(link) ?? [])
      if (!placed.has(j.child)) {
        placed.add(j.child);
        hide(j.child);
      }
  };
  if (tree.root !== null) visit(tree.root, 0, null, true);
  for (const r of tree.roots) visit(r, 0, null, false);
  for (const l of tree.links.keys()) visit(l, 0, null, false);
  return rows;
}

/**
 * Links on the path from the root to `link` (inclusive), used to expand
 * collapsed ancestors when something inside them is selected.
 * @param {KinTree} tree
 * @param {string} link
 * @returns {string[]}
 */
export function ancestors(tree, link) {
  /** @type {string[]} */
  const out = [];
  /** @type {Set<string>} */
  const seen = new Set([link]);
  let cur = tree.parentJoint.get(link)?.parent;
  while (cur !== undefined && !seen.has(cur)) {
    seen.add(cur);
    out.unshift(cur);
    cur = tree.parentJoint.get(cur)?.parent;
  }
  return out;
}
