// Browser app: wiring only. Every computation lives in src/core and is
// tested under Node. The model is parsed with the browser's DOMParser and
// converted to the same element tree the CLI uses. Nothing is uploaded:
// the only network requests are for this page's own files (the examples).

import { EXAMPLES } from "../core/examples.js";
import {
  forwardKinematics,
  isMovable,
  massProperties,
  resolvePositions,
  sliderRange,
} from "../core/fk.js";
import { CHECKS, SEVERITY_RANK } from "../core/findings.js";
import { inertiaValidity, primitiveInertia, rotateTensor } from "../core/inertia.js";
import { inspectText, inspectXml } from "../core/inspect.js";
import { ancestors } from "../core/layout.js";
import { matrixToRpy } from "../core/math.js";
import { singlePrimitive } from "../core/checks/context.js";
import { fromDomElement } from "../core/xml.js";
import { decodeState, encodeState } from "./state.js";
import { renderTree } from "./treeview.js";
import { createView } from "./view3d.js";

/** @typedef {import("../core/inspect.js").Report} Report */
/** @typedef {import("../core/findings.js").Finding} Finding */
/** @typedef {{ kind: "link" | "joint", name: string }} Selection */

/** @param {string} id */
const el = (id) => /** @type {HTMLElement} */ (document.getElementById(id));
/** @param {string} id */
const input = (id) => /** @type {HTMLInputElement} */ (document.getElementById(id));

/**
 * @param {string} tag
 * @param {Record<string, string>} [attrs]
 * @param {(Node | string)[]} [children]
 */
function h(tag, attrs = {}, children = []) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  for (const c of children) e.append(c);
  return e;
}

/** @param {number} v @param {number} [d] */
const f = (v, d = 4) => {
  if (!Number.isFinite(v)) return String(v);
  const s = (Math.abs(v) < 0.5 * 10 ** -d ? 0 : v).toFixed(d);
  return s;
};
/** @param {number} v */
const g = (v) => String(Number(v.toPrecision(6)));
/** @param {number[]} v @param {number} [d] */
const vec = (v, d) => v.map((x) => f(x, d)).join("  ");

const app = {
  state: decodeState(location.search),
  /** @type {Report | null} */
  report: null,
  name: "",
  /** @type {Set<string>} */
  collapsed: new Set(),
  /** @type {Map<string, import("../core/math.js").Transform>} */
  poses: new Map(),
  /** @type {import("../core/math.js").Vec3 | null} */
  com: null,
  view: /** @type {ReturnType<typeof createView> | null} */ (null),
};

// ------------------------------------------------------------------ input

/**
 * Parse with DOMParser; on a syntax error fall back to the built-in reader,
 * which reports the line and the reason in the same words as the CLI.
 * @param {string} text
 * @returns {Report}
 */
function inspectWithDom(text) {
  const doc = new DOMParser().parseFromString(text, "application/xml");
  if (doc.getElementsByTagName("parsererror").length > 0 || !doc.documentElement) {
    return inspectText(text);
  }
  return inspectXml(fromDomElement(doc.documentElement));
}

/**
 * @param {string} text
 * @param {string} label
 * @param {boolean} keepQ keep joint positions from the URL
 */
function load(text, label, keepQ) {
  app.report = inspectWithDom(text);
  app.name = label;
  app.collapsed = new Set();
  if (!keepQ) app.state.q = {};
  const tree = app.report.context?.tree;
  if (tree) {
    for (const k of Object.keys(app.state.q)) {
      const j = tree.joints.get(k);
      if (!j || !isMovable(j) || j.mimic) delete app.state.q[k];
    }
    const sel = app.state.selected;
    if (sel && !(sel.kind === "link" ? tree.links.has(sel.name) : tree.joints.has(sel.name)))
      app.state.selected = null;
  } else app.state.selected = null;
  const s = app.report.summary;
  const status = el("status");
  status.textContent = `ready: ${label}`;
  status.classList.toggle("bad", s.errors > 0);
  renderAll(true);
}

async function loadExample() {
  const ex = EXAMPLES.find((e) => e.id === app.state.source) ?? EXAMPLES[0];
  app.state.source = ex.id;
  /** @type {HTMLSelectElement} */ (el("example")).value = ex.id;
  el("example-note").textContent = ex.note;
  el("status").textContent = `loading example ${ex.id}`;
  try {
    const res = await fetch(`examples/${ex.file}`);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    load(await res.text(), `example ${ex.id} (${ex.file})`, true);
  } catch (e) {
    el("status").textContent = `could not load the example: ${e instanceof Error ? e.message : e}`;
    el("status").classList.add("bad");
  }
}

/** @param {File} file */
function loadFile(file) {
  if (file.size > 20e6) {
    el("status").textContent = `${file.name} is larger than 20 MB; is it a URDF?`;
    return;
  }
  const reader = new FileReader();
  reader.onload = () => {
    app.state.source = "file";
    app.state.selected = null;
    /** @type {HTMLSelectElement} */ (el("example")).value = "";
    el("example-note").textContent = "";
    load(String(reader.result), `file ${file.name}`, false);
  };
  reader.onerror = () => (el("status").textContent = `could not read ${file.name}`);
  reader.readAsText(file);
}

// ------------------------------------------------------------------ render

function updateUrl() {
  const q = encodeState(app.state);
  history.replaceState(null, "", q ? `?${q}` : location.pathname);
}

function recompute() {
  const ctx = app.report?.context;
  if (!ctx) {
    app.poses = new Map();
    app.com = null;
    return;
  }
  app.poses = forwardKinematics(ctx.tree, app.state.q);
  app.com = massProperties(ctx.tree, app.poses).com;
}

/** @param {boolean} refit */
function renderAll(refit) {
  recompute();
  renderSummary();
  renderFindings();
  renderTreePanel();
  renderSliders();
  renderDetails();
  render3d(refit);
  updateUrl();
}

function renderSummary() {
  const r = /** @type {Report} */ (app.report);
  const s = r.summary;
  const parts = [`${s.errors} error(s), ${s.warnings} warning(s), ${s.infos} note(s)`];
  if (r.robot) {
    parts.unshift(
      `robot '${s.name}': ${s.links} links, ${s.joints} joints, root ${s.root ?? "none"}`,
    );
    if (s.com) parts.push(`mass ${g(s.totalMass)} kg`);
  }
  el("summary").textContent = parts.join(" · ");
}

function renderFindings() {
  const r = /** @type {Report} */ (app.report);
  const list = el("findings");
  const show = {
    error: input("f-error").checked,
    warning: input("f-warning").checked,
    info: input("f-info").checked,
  };
  const items = r.findings.filter((x) => show[x.severity]);
  const sel = app.state.selected;
  list.replaceChildren(
    ...items.map((fd) => {
      const c = CHECKS[fd.id];
      const isSel = sel !== null && fd.target.kind === sel.kind && fd.target.name === sel.name;
      const li = h(
        "li",
        { class: `finding ${fd.severity}${isSel ? " selected" : ""}`, tabindex: "0" },
        [
          h("div", { class: "head" }, [
            h("span", { class: "sev" }, [fd.severity === "info" ? "note" : fd.severity]),
            h("span", { class: "id" }, [fd.id]),
            h("span", { class: "muted small" }, [c.title]),
          ]),
          h("p", { class: "msg" }, [fd.message]),
          h("div", { class: "path" }, [fd.path + (fd.line ? `  (line ${fd.line})` : "")]),
          h("details", {}, [h("summary", {}, ["Why it matters"]), h("p", {}, [c.why])]),
        ],
      );
      const go = () => {
        if (fd.target.kind === "link" || fd.target.kind === "joint")
          select({ kind: fd.target.kind, name: fd.target.name });
      };
      li.addEventListener("click", (e) => {
        if (/** @type {HTMLElement} */ (e.target).closest("details")) return;
        go();
      });
      li.addEventListener("keydown", (e) => {
        if (e.key === "Enter") go();
      });
      return li;
    }),
  );
  if (items.length === 0)
    list.append(
      h("li", { class: "muted" }, [
        r.findings.length ? "All findings are filtered out." : "No findings.",
      ]),
    );
}

/** Worst severity per link and per joint ("joint:name"). */
function badges() {
  /** @type {Map<string, "error" | "warning" | "info">} */
  const m = new Map();
  for (const fd of app.report?.findings ?? []) {
    const key =
      fd.target.kind === "link"
        ? fd.target.name
        : fd.target.kind === "joint"
          ? `joint:${fd.target.name}`
          : "";
    if (!key) continue;
    const prev = m.get(key);
    if (!prev || SEVERITY_RANK[fd.severity] < SEVERITY_RANK[prev]) m.set(key, fd.severity);
  }
  return m;
}

function renderTreePanel() {
  const ctx = app.report?.context;
  const host = el("tree");
  if (!ctx) {
    host.replaceChildren(
      h("p", { class: "muted" }, ["No tree: fix the errors listed under Findings first."]),
    );
    return;
  }
  renderTree(host, ctx.tree, {
    collapsed: app.collapsed,
    selected: app.state.selected,
    badges: badges(),
    onSelect: select,
    onToggle: (link) => {
      if (app.collapsed.has(link)) app.collapsed.delete(link);
      else app.collapsed.add(link);
      renderTreePanel();
    },
  });
}

function renderSliders() {
  const host = el("sliders");
  const ctx = app.report?.context;
  if (!ctx) {
    host.replaceChildren();
    return;
  }
  const movable = ctx.tree.order
    .map((l) => ctx.tree.parentJoint.get(l))
    .filter((j) => j !== undefined && isMovable(j));
  if (movable.length === 0) {
    host.replaceChildren(h("p", { class: "muted small" }, ["No movable joints."]));
    return;
  }
  const resolved = resolvePositions(ctx.tree, app.state.q);
  const reset = h("button", { type: "button" }, ["Zero all joints"]);
  reset.addEventListener("click", () => {
    app.state.q = {};
    renderSliders();
    onPose();
  });
  host.replaceChildren(
    h("div", { class: "sliders-head" }, [h("strong", {}, ["Joint positions"]), reset]),
    ...movable.map((jj) => {
      const j = /** @type {import("../core/urdf.js").Joint} */ (jj);
      const unit = j.type === "prismatic" ? "m" : "rad";
      const val = h("span", { class: "val" }, [`${f(resolved.get(j.name) ?? 0, 3)} ${unit}`]);
      if (j.mimic && ctx.tree.joints.has(j.mimic.joint) && j.mimic.joint !== j.name) {
        return h("div", { class: "slider mimic" }, [
          h("span", { class: "name", title: j.name }, [`${j.name} follows ${j.mimic.joint}`]),
          val,
        ]);
      }
      const { min, max } = sliderRange(j);
      const r = /** @type {HTMLInputElement} */ (
        h("input", {
          type: "range",
          min: String(min),
          max: String(max),
          step: String((max - min) / 500),
          value: String(app.state.q[j.name] ?? 0),
          "aria-label": `${j.name} position`,
          "data-joint": j.name,
        })
      );
      r.addEventListener("input", () => {
        app.state.q[j.name] = Number(r.value);
        const now = resolvePositions(ctx.tree, app.state.q);
        for (const span of host.querySelectorAll("[data-val]")) {
          const name = /** @type {HTMLElement} */ (span).dataset.val ?? "";
          const jt = ctx.tree.joints.get(name);
          span.textContent = `${f(now.get(name) ?? 0, 3)} ${jt?.type === "prismatic" ? "m" : "rad"}`;
        }
        onPose();
      });
      return h("label", { class: "slider" }, [
        h("span", { class: "name", title: j.name }, [j.name]),
        val,
        r,
      ]);
    }),
  );
  // Tag value spans so slider moves can refresh mimic followers too.
  const labels = host.querySelectorAll(".slider");
  let k = 0;
  for (const jj of movable) {
    const span = labels[k++]?.querySelector(".val");
    if (span && jj) /** @type {HTMLElement} */ (span).dataset.val = jj.name;
  }
}

let urlTimer = 0;
function onPose() {
  recompute();
  render3d(false);
  if (app.state.selected) renderDetails();
  clearTimeout(urlTimer);
  urlTimer = window.setTimeout(updateUrl, 250);
}

/** @param {boolean} refit */
function render3d(refit) {
  const ctx = app.report?.context;
  if (!app.view) return;
  if (!ctx) {
    app.view.set(
      {
        tree: {
          links: new Map(),
          joints: new Map(),
          parentJoint: new Map(),
          childJoints: new Map(),
          root: null,
          order: [],
          roots: [],
        },
        poses: new Map(),
        materials: new Map(),
        com: null,
        selectedLink: null,
        view: app.state.view,
        geometry: app.state.geometry,
        frames: app.state.frames,
        showCom: app.state.com,
      },
      true,
    );
    return;
  }
  const sel = app.state.selected;
  const selectedLink =
    sel === null
      ? null
      : sel.kind === "link"
        ? sel.name
        : (ctx.tree.joints.get(sel.name)?.child ?? null);
  app.view.set(
    {
      tree: ctx.tree,
      poses: app.poses,
      materials: ctx.robot.materials,
      com: app.com,
      selectedLink,
      view: app.state.view,
      geometry: app.state.geometry,
      frames: app.state.frames,
      showCom: app.state.com,
    },
    refit,
  );
}

/** @param {Selection} s */
function select(s) {
  app.state.selected = s;
  const tree = app.report?.context?.tree;
  if (tree) {
    const link = s.kind === "link" ? s.name : tree.joints.get(s.name)?.child;
    if (link) for (const a of ancestors(tree, link)) app.collapsed.delete(a);
  }
  renderFindings();
  renderTreePanel();
  renderDetails();
  render3d(false);
  updateUrl();
}

/**
 * @param {[string, Node | string][]} rows
 */
const table = (rows) =>
  h(
    "table",
    {},
    rows.map(([k, v]) => h("tr", {}, [h("th", { scope: "row" }, [k]), h("td", {}, [v])])),
  );

/** @param {Selection} s @param {string} [label] */
function chip(s, label) {
  const c = h("span", { class: "chip", role: "button", tabindex: "0" }, [label ?? s.name]);
  c.addEventListener("click", () => select(s));
  c.addEventListener("keydown", (e) => {
    if (e.key === "Enter") select(s);
  });
  return c;
}

/** @param {import("../core/urdf.js").Geometry | null} geo */
function geomText(geo) {
  if (!geo) return "none";
  if (geo.type === "box") return `box ${vec(geo.size, 3)} m`;
  if (geo.type === "cylinder") return `cylinder r ${g(geo.radius)} m, length ${g(geo.length)} m`;
  if (geo.type === "sphere") return `sphere r ${g(geo.radius)} m`;
  if (geo.type === "mesh")
    return `mesh ${geo.filename} (scale ${geo.scale.map(g).join(" ")}; not loaded)`;
  return `<${geo.tag}> (not supported)`;
}

/** @param {import("../core/urdf.js").InertiaTensor} t */
const tensorText = (t) =>
  `ixx ${g(t.ixx)}  iyy ${g(t.iyy)}  izz ${g(t.izz)}  ixy ${g(t.ixy)}  ixz ${g(t.ixz)}  iyz ${g(t.iyz)}`;

function renderDetails() {
  const host = el("details");
  const title = el("details-title");
  const ctx = app.report?.context;
  const sel = app.state.selected;
  if (!ctx || !sel) {
    title.textContent = "Details";
    host.replaceChildren(
      h("p", { class: "muted" }, ["Select a link or joint in the tree, the 3D view or a finding."]),
    );
    return;
  }
  const tree = ctx.tree;
  const related = (app.report?.findings ?? []).filter(
    (x) => x.target.kind === sel.kind && x.target.name === sel.name,
  );
  /** @type {Node[]} */
  const out = [];
  if (sel.kind === "link") {
    const link = tree.links.get(sel.name);
    if (!link) return;
    title.textContent = `Link ${link.name}`;
    const pose = app.poses.get(link.name);
    const pj = tree.parentJoint.get(link.name);
    const kids = tree.childJoints.get(link.name) ?? [];
    out.push(
      table([
        [`Position in ${tree.root}`, pose ? `${vec(pose.p)} m` : "not connected"],
        ["Orientation (rpy)", pose ? `${vec(matrixToRpy(pose.R))} rad` : "-"],
        ["Parent joint", pj ? chip({ kind: "joint", name: pj.name }) : "none (root)"],
        [
          "Child joints",
          kids.length
            ? h(
                "span",
                {},
                kids.map((j) => chip({ kind: "joint", name: j.name })),
              )
            : "none",
        ],
      ]),
    );
    out.push(h("h3", {}, ["Inertial"]));
    const inert = link.inertial;
    if (!inert) out.push(h("p", { class: "muted" }, ["No <inertial> block."]));
    else {
      /** @type {[string, Node | string][]} */
      const rows = [
        ["Mass", inert.mass === null ? "missing" : `${g(inert.mass)} kg`],
        ["Origin xyz / rpy", `${vec(inert.origin.xyz)} m / ${vec(inert.origin.rpy)} rad`],
      ];
      if (inert.inertia) {
        rows.push(["Tensor (kg m²)", tensorText(inert.inertia)]);
        const v = inertiaValidity(inert.inertia);
        rows.push([
          "Principal moments",
          `${v.principal.map(g).join(", ")} (${v.positiveDefinite ? "positive definite" : "NOT positive definite"}, triangle inequality ${v.triangle ? "holds" : "FAILS"})`,
        ]);
        const shape = singlePrimitive(link);
        const ref =
          shape?.geometry && inert.mass !== null && inert.mass > 0
            ? primitiveInertia(shape.geometry, inert.mass)
            : null;
        if (shape && ref)
          rows.push([
            `Solid ${shape.geometry?.type} reference`,
            tensorText(rotateTensor(ref, shape.origin.rpy, inert.origin.rpy)),
          ]);
      }
      out.push(table(rows));
    }
    out.push(h("h3", {}, ["Geometry"]));
    const shapes = [...link.visuals, ...link.collisions];
    if (shapes.length === 0)
      out.push(h("p", { class: "muted" }, ["No visual or collision shapes."]));
    else
      out.push(
        table(
          shapes.map((s, k) => [
            `${s.kind} ${s.kind === "visual" ? link.visuals.indexOf(s) + 1 : k - link.visuals.length + 1}`,
            `${geomText(s.geometry)}; origin ${vec(s.origin.xyz, 3)} / ${vec(s.origin.rpy, 3)}`,
          ]),
        ),
      );
  } else {
    const j = tree.joints.get(sel.name);
    if (!j) return;
    title.textContent = `Joint ${j.name}`;
    const q = resolvePositions(tree, app.state.q).get(j.name) ?? 0;
    /** @type {[string, Node | string][]} */
    const rows = [
      ["Type", j.type],
      [
        "Parent → child",
        h("span", {}, [
          chip({ kind: "link", name: j.parent }),
          "→ ",
          chip({ kind: "link", name: j.child }),
        ]),
      ],
      ["Origin xyz", `${vec(j.origin.xyz)} m`],
      ["Origin rpy", `${vec(j.origin.rpy)} rad`],
    ];
    if (isMovable(j) || j.type === "planar")
      rows.push(["Axis", `${vec(j.axis, 3)}${j.axisGiven ? "" : " (default)"}`]);
    if (isMovable(j))
      rows.push(["Position now", `${f(q)} ${j.type === "prismatic" ? "m" : "rad"}`]);
    if (j.limit) {
      const l = j.limit;
      rows.push([
        "Limit",
        `lower ${l.lower === null ? "-" : g(l.lower)}, upper ${l.upper === null ? "-" : g(l.upper)}, effort ${l.effort === null ? "-" : g(l.effort)}, velocity ${l.velocity === null ? "-" : g(l.velocity)}`,
      ]);
    } else rows.push(["Limit", "none"]);
    if (j.dynamics)
      rows.push([
        "Dynamics",
        `damping ${g(j.dynamics.damping)}, friction ${g(j.dynamics.friction)}`,
      ]);
    if (j.mimic)
      rows.push([
        "Mimic",
        h("span", {}, [
          chip({ kind: "joint", name: j.mimic.joint }),
          ` × ${g(j.mimic.multiplier)} + ${g(j.mimic.offset)}`,
        ]),
      ]);
    out.push(table(rows));
  }
  out.push(h("h3", {}, [`Findings (${related.length})`]));
  if (related.length === 0) out.push(h("p", { class: "muted" }, ["None for this element."]));
  else
    out.push(
      h(
        "ul",
        {},
        related.map((x) =>
          h("li", {}, [
            h("strong", {}, [`${x.severity === "info" ? "note" : x.severity} ${x.id}: `]),
            x.message,
          ]),
        ),
      ),
    );
  host.replaceChildren(...out);
}

// ------------------------------------------------------------------ setup

function applyTheme() {
  const t = app.state.theme;
  if (t === "auto") document.documentElement.removeAttribute("data-theme");
  else document.documentElement.dataset.theme = t;
  el("btn-theme").textContent = `Theme: ${t}`;
}

function init() {
  const sel = /** @type {HTMLSelectElement} */ (el("example"));
  sel.replaceChildren(
    h("option", { value: "" }, ["(choose)"]),
    ...EXAMPLES.map((e) => h("option", { value: e.id }, [e.title])),
  );
  sel.addEventListener("change", () => {
    if (!sel.value) return;
    app.state.source = sel.value;
    app.state.selected = null;
    app.state.q = {};
    loadExample();
  });

  const drop = el("drop");
  input("file").addEventListener("change", () => {
    const file = input("file").files?.[0];
    if (file) loadFile(file);
  });
  drop.addEventListener("dragover", (e) => {
    e.preventDefault();
    drop.classList.add("over");
  });
  drop.addEventListener("dragleave", () => drop.classList.remove("over"));
  drop.addEventListener("drop", (e) => {
    e.preventDefault();
    drop.classList.remove("over");
    const file = e.dataTransfer?.files?.[0];
    if (file) loadFile(file);
  });
  el("btn-paste").addEventListener("click", () => {
    const text = /** @type {HTMLTextAreaElement} */ (el("paste")).value;
    if (!text.trim()) return;
    app.state.source = "paste";
    app.state.selected = null;
    sel.value = "";
    el("example-note").textContent = "";
    load(text, "pasted XML", false);
  });

  for (const id of ["f-error", "f-warning", "f-info"])
    input(id).addEventListener("change", renderFindings);

  const viewMode = /** @type {HTMLSelectElement} */ (el("view-mode"));
  const geomMode = /** @type {HTMLSelectElement} */ (el("geom-mode"));
  viewMode.value = app.state.view;
  geomMode.value = app.state.geometry;
  input("show-frames").checked = app.state.frames;
  input("show-com").checked = app.state.com;
  viewMode.addEventListener("change", () => {
    app.state.view = viewMode.value === "wire" ? "wire" : "solid";
    render3d(false);
    updateUrl();
  });
  geomMode.addEventListener("change", () => {
    app.state.geometry = geomMode.value === "collision" ? "collision" : "visual";
    render3d(false);
    updateUrl();
  });
  input("show-frames").addEventListener("change", () => {
    app.state.frames = input("show-frames").checked;
    render3d(false);
    updateUrl();
  });
  input("show-com").addEventListener("change", () => {
    app.state.com = input("show-com").checked;
    render3d(false);
    updateUrl();
  });
  el("btn-reset-view").addEventListener("click", () => app.view?.reset());
  el("btn-expand").addEventListener("click", () => {
    app.collapsed.clear();
    renderTreePanel();
  });

  el("btn-link").addEventListener("click", async () => {
    updateUrl();
    const btn = el("btn-link");
    try {
      await navigator.clipboard.writeText(location.href);
      btn.textContent =
        app.state.source === "file" || app.state.source === "paste"
          ? "Copied (model not included)"
          : "Copied";
    } catch {
      btn.textContent = "Copy failed";
    }
    setTimeout(() => (btn.textContent = "Copy link"), 2000);
  });
  el("btn-theme").addEventListener("click", () => {
    const t = app.state.theme;
    app.state.theme = t === "auto" ? "light" : t === "light" ? "dark" : "auto";
    applyTheme();
    render3d(false);
    renderTreePanel();
    updateUrl();
  });
  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => render3d(false));
  applyTheme();

  app.view = createView(/** @type {HTMLCanvasElement} */ (el("view3d")), {
    onPick: (link) => select({ kind: "link", name: link }),
  });

  if (app.state.source === "file" || app.state.source === "paste") {
    el("status").textContent =
      "This link was made with a local model, which is never put in a URL. Drop the file again or paste it.";
    sel.value = "";
  } else loadExample();
}

init();
