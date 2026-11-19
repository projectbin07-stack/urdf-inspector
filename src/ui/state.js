// URL parameters <-> page state. DOM-free so it can be tested under Node.
// Only values that differ from the defaults are written, so links stay
// short. The model itself is never put in the URL: a link made with a local
// file or pasted XML says so and asks for the model again.

import { EXAMPLES } from "../core/examples.js";

/**
 * @typedef {object} PageState
 * @property {string} source example id, "file" or "paste"
 * @property {{ kind: "link" | "joint", name: string } | null} selected
 * @property {"solid" | "wire"} view
 * @property {"visual" | "collision"} geometry
 * @property {boolean} frames show link frame axes
 * @property {boolean} com show the centre of mass
 * @property {Record<string, number>} q joint positions (examples only)
 * @property {"auto" | "light" | "dark"} theme
 */

/** @returns {PageState} */
export function defaultState() {
  return {
    source: "mecanum",
    selected: null,
    view: "solid",
    geometry: "visual",
    frames: true,
    com: true,
    q: {},
    theme: "auto",
  };
}

const THEMES = ["auto", "light", "dark"];
const NAME = /^[A-Za-z0-9_.\-/:]{1,128}$/;

/** @param {number} v */
const num = (v) => String(Number(v.toPrecision(6)));

/**
 * @param {PageState} st
 * @returns {string} query string without the leading "?"
 */
export function encodeState(st) {
  const d = defaultState();
  const p = new URLSearchParams();
  if (st.source !== d.source) p.set("s", st.source);
  if (st.selected) p.set("sel", `${st.selected.kind === "link" ? "l" : "j"}:${st.selected.name}`);
  if (st.view !== d.view) p.set("v", st.view);
  if (st.geometry !== d.geometry) p.set("g", st.geometry);
  if (st.frames !== d.frames) p.set("ax", st.frames ? "1" : "0");
  if (st.com !== d.com) p.set("com", st.com ? "1" : "0");
  const isExample = EXAMPLES.some((e) => e.id === st.source);
  const q = Object.entries(st.q)
    .filter(([, v]) => v !== 0 && Number.isFinite(v))
    .sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0))
    .map(([n, v]) => `${n}:${num(v)}`);
  if (isExample && q.length > 0) p.set("q", q.join(","));
  if (st.theme !== d.theme) p.set("theme", st.theme);
  return p.toString();
}

/**
 * Parse a query string; unknown or malformed values fall back to defaults.
 * @param {string} search with or without the leading "?"
 * @returns {PageState}
 */
export function decodeState(search) {
  const st = defaultState();
  const p = new URLSearchParams(search);
  const s = p.get("s");
  if (s !== null && (EXAMPLES.some((e) => e.id === s) || s === "file" || s === "paste"))
    st.source = s;
  const sel = p.get("sel");
  const m = sel ? /^([lj]):(.+)$/.exec(sel) : null;
  if (m && NAME.test(m[2])) st.selected = { kind: m[1] === "l" ? "link" : "joint", name: m[2] };
  if (p.get("v") === "wire") st.view = "wire";
  if (p.get("g") === "collision") st.geometry = "collision";
  if (p.get("ax") === "0") st.frames = false;
  if (p.get("com") === "0") st.com = false;
  const q = p.get("q");
  if (q)
    for (const part of q.split(",")) {
      const k = part.lastIndexOf(":");
      if (k <= 0) continue;
      const name = part.slice(0, k);
      const v = Number(part.slice(k + 1));
      if (NAME.test(name) && Number.isFinite(v) && Math.abs(v) < 1e4) st.q[name] = v;
    }
  const theme = p.get("theme");
  if (theme && THEMES.includes(theme)) st.theme = /** @type {PageState["theme"]} */ (theme);
  return st;
}
