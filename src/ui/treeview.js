// SVG kinematic tree diagram drawn from the rows of src/core/layout.js.
// Names are inserted as text nodes, never as markup.

import { layoutTree } from "../core/layout.js";

const SVG = "http://www.w3.org/2000/svg";
const ROW = 26;
const INDENT = 22;

/** @type {Record<string, string>} */
const TYPE_COLOUR = {
  revolute: "var(--z)",
  continuous: "var(--y)",
  prismatic: "var(--com)",
  fixed: "var(--neutral)",
  floating: "var(--x)",
  planar: "var(--x)",
};

/**
 * @param {string} tag
 * @param {Record<string, string | number>} attrs
 * @param {string} [text]
 */
function el(tag, attrs, text) {
  const e = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
  if (text !== undefined) e.textContent = text;
  return e;
}

/**
 * @param {HTMLElement} host
 * @param {import("../core/tree.js").KinTree} tree
 * @param {{
 *   collapsed: Set<string>,
 *   selected: { kind: "link" | "joint", name: string } | null,
 *   badges: Map<string, "error" | "warning" | "info">,
 *   onSelect: (sel: { kind: "link" | "joint", name: string }) => void,
 *   onToggle: (link: string) => void,
 * }} opts
 */
export function renderTree(host, tree, opts) {
  const rows = layoutTree(tree, opts.collapsed);
  const width = Math.max(
    280,
    ...rows.map(
      (r) => 40 + r.depth * INDENT + (r.link.length + (r.joint ? r.joint.length + 14 : 0)) * 7.4,
    ),
  );
  const svg = el("svg", {
    width,
    height: rows.length * ROW + 8,
    viewBox: `0 0 ${width} ${rows.length * ROW + 8}`,
    role: "tree",
    "aria-label": "Kinematic tree",
  });
  // Edges first, under the nodes.
  for (const r of rows) {
    if (r.parentRow === null) continue;
    const px = 14 + (r.depth - 1) * INDENT;
    const py = r.parentRow * ROW + ROW / 2 + 8;
    const x = 14 + r.depth * INDENT - 8;
    const y = r.row * ROW + ROW / 2 + 4;
    svg.append(el("path", { class: "edge", d: `M${px} ${py} V${y} H${x}` }));
  }
  for (const r of rows) {
    const x = 14 + r.depth * INDENT;
    const y = r.row * ROW + 4;
    const sel =
      opts.selected &&
      ((opts.selected.kind === "link" && opts.selected.name === r.link) ||
        (opts.selected.kind === "joint" && opts.selected.name === r.joint));
    const g = el("g", {
      class: `node${sel ? " selected" : ""}${r.connected ? "" : " orphan"}`,
      role: "treeitem",
      tabindex: 0,
      "aria-label": `link ${r.link}${r.joint ? `, joint ${r.joint} (${r.jointType})` : ""}`,
    });
    g.append(
      el("rect", { class: "bg", x: x - 12, y, width: width - x + 8, height: ROW - 2, rx: 4 }),
    );
    if (r.children > 0) {
      const t = el("text", { class: "toggle", x: x - 10, y: y + 16 }, r.collapsed ? "▸" : "▾");
      t.addEventListener("click", (e) => {
        e.stopPropagation();
        opts.onToggle(r.link);
      });
      g.append(t);
    }
    const badge = opts.badges.get(r.link);
    let tx = x + 2;
    if (badge) {
      g.append(
        el("circle", {
          cx: tx + 3,
          cy: y + 12,
          r: 4,
          fill: `var(--${badge === "error" ? "crit" : badge === "warning" ? "warn" : "info"})`,
        }),
      );
      tx += 10;
    }
    const name = el(
      "text",
      { class: "link-name", x: tx, y: y + 16 },
      r.link + (r.collapsed ? ` (+${r.children})` : ""),
    );
    g.append(name);
    if (r.joint) {
      const jx = tx + (r.link.length + (r.collapsed ? 6 : 0)) * 7.6 + 10;
      const jb = opts.badges.get(`joint:${r.joint}`);
      const jt = el(
        "text",
        { class: "joint-name", x: jx + 12, y: y + 16 },
        `${r.joint} · ${r.jointType}`,
      );
      g.append(
        el("rect", {
          x: jx,
          y: y + 7,
          width: 8,
          height: 8,
          rx: 2,
          fill: TYPE_COLOUR[r.jointType ?? ""] ?? "var(--neutral)",
        }),
      );
      if (jb)
        g.append(
          el("circle", {
            cx: jx + 4,
            cy: y + 11,
            r: 6,
            fill: "none",
            stroke: `var(--${jb === "error" ? "crit" : jb === "warning" ? "warn" : "info"})`,
            "stroke-width": 2,
          }),
        );
      jt.addEventListener("click", (e) => {
        e.stopPropagation();
        opts.onSelect({ kind: "joint", name: /** @type {string} */ (r.joint) });
      });
      g.append(jt);
    }
    g.addEventListener("click", () => opts.onSelect({ kind: "link", name: r.link }));
    g.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        opts.onSelect({ kind: "link", name: r.link });
      } else if ((e.key === "ArrowRight" || e.key === "ArrowLeft") && r.children > 0) {
        if ((e.key === "ArrowLeft") !== r.collapsed) opts.onToggle(r.link);
      }
    });
    svg.append(g);
  }
  host.replaceChildren(svg);
}
