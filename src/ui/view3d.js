// Canvas 2D renderer for the 3D view: painter's algorithm over the
// tessellated primitives from src/core/scene.js, flat shading, link frame
// axes, joint skeleton, centre-of-mass marker and an orbit camera driven by
// mouse and touch. All geometry and projection math lives in the core.

import {
  buildSolids,
  bounds,
  faceNormal,
  fitCamera,
  frameAxes,
  projector,
  cameraBasis,
} from "../core/scene.js";
import { dot } from "../core/math.js";

/** @typedef {import("../core/math.js").Vec3} Vec3 */
/** @typedef {import("../core/math.js").Transform} Transform */
/** @typedef {import("../core/scene.js").Camera} Camera */

/**
 * @typedef {object} SceneInput
 * @property {import("../core/tree.js").KinTree} tree
 * @property {Map<string, Transform>} poses
 * @property {Map<string, { rgba: number[] | null }>} materials
 * @property {Vec3 | null} com
 * @property {string | null} selectedLink
 * @property {"solid" | "wire"} view
 * @property {"visual" | "collision"} geometry
 * @property {boolean} frames
 * @property {boolean} showCom
 */

/** @param {string} name */
const cssVar = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

/**
 * @param {HTMLCanvasElement} canvas
 * @param {{ onPick: (link: string) => void }} hooks
 */
export function createView(canvas, hooks) {
  const ctx = /** @type {CanvasRenderingContext2D} */ (canvas.getContext("2d"));
  /** @type {SceneInput | null} */
  let scene = null;
  /** @type {Camera} */
  let cam = fitCamera({ centre: [0, 0, 0], radius: 0.5 });
  let fitted = false;
  /** @type {{ link: string, x: number, y: number }[]} */
  let pickable = [];

  const size = () => {
    const r = canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(r.width * dpr));
    const h = Math.max(1, Math.round(r.height * dpr));
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    return { w, h, dpr };
  };

  /** Bounds of everything drawn at the zero configuration. @param {SceneInput} s */
  const sceneBounds = (s) => {
    const { solids } = buildSolids(s.tree, s.poses, s.geometry, s.materials);
    /** @type {Vec3[]} */
    const pts = [];
    for (const so of solids) pts.push(...so.vertices);
    for (const p of s.poses.values()) pts.push(p.p);
    return bounds(pts);
  };

  const reset = () => {
    if (!scene) return;
    cam = fitCamera(sceneBounds(scene));
    render();
  };

  const render = () => {
    const { w, h, dpr } = size();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = cssVar("--view-bg");
    ctx.fillRect(0, 0, w, h);
    pickable = [];
    if (!scene) return;
    const s = scene;
    const project = projector(cam, w, h);
    const { eye, f, u } = cameraBasis(cam);
    const b = sceneBounds(s);
    const axisLen = Math.max(b.radius * 0.18, 0.02);

    // Ground grid on the root frame's z = 0 plane.
    const step = niceStep(b.radius / 4);
    const half = Math.ceil((b.radius * 1.4) / step) * step;
    ctx.strokeStyle = cssVar("--grid");
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    for (let v = -half; v <= half + 1e-9; v += step) {
      line(
        project,
        [b.centre[0] - half, b.centre[1] + v, 0],
        [b.centre[0] + half, b.centre[1] + v, 0],
      );
      line(
        project,
        [b.centre[0] + v, b.centre[1] - half, 0],
        [b.centre[0] + v, b.centre[1] + half, 0],
      );
    }
    ctx.stroke();

    // Solids: collect faces, sort far to near.
    const { solids, meshes } = buildSolids(s.tree, s.poses, s.geometry, s.materials);
    const light = /** @type {Vec3} */ ([
      -f[0] + 0.4 * u[0],
      -f[1] + 0.4 * u[1],
      -f[2] + 0.4 * u[2],
    ]);
    const ll = Math.hypot(...light);
    const L = /** @type {Vec3} */ (light.map((v) => v / ll));
    const solidColor = parseColor(cssVar("--solid"));
    const edge = cssVar("--edge");
    const accent = cssVar("--accent");
    /** @type {{ depth: number, pts: { x: number, y: number }[], fill: string, stroke: string, sel: boolean }[]} */
    const faces = [];
    for (const so of solids) {
      const sel = so.link === s.selectedLink;
      const base = so.rgba ? so.rgba.slice(0, 3).map((c) => Math.round(c * 255)) : solidColor;
      const alpha = so.rgba ? Math.max(0.25, so.rgba[3] ?? 1) : 1;
      if (s.view === "solid") {
        for (const face of so.faces) {
          const wpts = face.map((k) => so.vertices[k]);
          const n = faceNormal(wpts);
          const c = wpts
            .reduce((a, p) => [a[0] + p[0], a[1] + p[1], a[2] + p[2]], [0, 0, 0])
            .map((v) => v / wpts.length);
          const toEye = [eye[0] - c[0], eye[1] - c[1], eye[2] - c[2]];
          if (dot(n, /** @type {Vec3} */ (toEye)) <= 0) continue; // back face
          const proj = wpts.map(project);
          if (proj.some((p) => p === null)) continue;
          const shade = 0.45 + 0.55 * Math.max(0, dot(n, L));
          const rgb = base.map((v) => Math.round(Math.min(255, v * shade)));
          faces.push({
            depth: Math.max(...proj.map((p) => /** @type {{ depth: number }} */ (p).depth)),
            pts: /** @type {{ x: number, y: number }[]} */ (proj),
            fill: `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${alpha})`,
            stroke: sel ? accent : "",
            sel,
          });
        }
      }
    }
    faces.sort((a, b2) => b2.depth - a.depth);
    for (const fc of faces) {
      ctx.beginPath();
      fc.pts.forEach((p, k) => (k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)));
      ctx.closePath();
      ctx.fillStyle = fc.fill;
      ctx.fill();
      ctx.strokeStyle = fc.fill;
      ctx.lineWidth = 0.6 * dpr;
      ctx.stroke();
    }
    // Outlines: always in wireframe, and for the selected link in solid mode.
    for (const so of solids) {
      const sel = so.link === s.selectedLink;
      if (s.view === "solid" && !sel) continue;
      ctx.strokeStyle = sel ? accent : edge;
      ctx.lineWidth = (sel ? 2 : 1) * dpr;
      ctx.beginPath();
      for (const [a, c] of so.edges) line(project, so.vertices[a], so.vertices[c]);
      ctx.stroke();
    }

    // Meshes are not loaded: a dashed diamond marks where they would be.
    ctx.strokeStyle = cssVar("--neutral");
    ctx.setLineDash([3 * dpr, 3 * dpr]);
    for (const m of meshes) {
      const p = project(m.at);
      if (!p) continue;
      const r = 7 * dpr;
      ctx.beginPath();
      ctx.moveTo(p.x, p.y - r);
      ctx.lineTo(p.x + r, p.y);
      ctx.lineTo(p.x, p.y + r);
      ctx.lineTo(p.x - r, p.y);
      ctx.closePath();
      ctx.stroke();
    }
    ctx.setLineDash([]);

    // Joint skeleton: parent link origin to child link origin.
    ctx.strokeStyle = cssVar("--neutral");
    ctx.lineWidth = 1.2 * dpr;
    ctx.setLineDash([4 * dpr, 3 * dpr]);
    ctx.beginPath();
    for (const j of s.tree.joints.values()) {
      const a = s.poses.get(j.parent);
      const c = s.poses.get(j.child);
      if (a && c && s.tree.parentJoint.get(j.child) === j) line(project, a.p, c.p);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // Link frames and pick points.
    const cx = cssVar("--x");
    const cy = cssVar("--y");
    const cz = cssVar("--z");
    for (const name of s.tree.order) {
      const pose = s.poses.get(name);
      if (!pose) continue;
      const o = project(pose.p);
      if (o) pickable.push({ link: name, x: o.x, y: o.y });
      const sel = name === s.selectedLink;
      if (!s.frames && !sel) continue;
      const ax = frameAxes(pose, sel ? axisLen * 1.4 : axisLen);
      ctx.lineWidth = (sel ? 2.5 : 1.5) * dpr;
      for (const [end, col] of /** @type {[Vec3, string][]} */ ([
        [ax.x, cx],
        [ax.y, cy],
        [ax.z, cz],
      ])) {
        ctx.strokeStyle = col;
        ctx.beginPath();
        line(project, ax.o, end);
        ctx.stroke();
      }
      if (sel && o) {
        ctx.fillStyle = accent;
        ctx.font = `${12 * dpr}px system-ui, sans-serif`;
        ctx.fillText(name, o.x + 6 * dpr, o.y - 6 * dpr);
      }
    }

    // Centre of mass.
    if (s.showCom && s.com) {
      const p = project(s.com);
      if (p) {
        const r = 7 * dpr;
        ctx.strokeStyle = cssVar("--com");
        ctx.fillStyle = cssVar("--com");
        ctx.lineWidth = 2 * dpr;
        ctx.beginPath();
        ctx.arc(p.x, p.y, r, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(p.x - r * 1.6, p.y);
        ctx.lineTo(p.x + r * 1.6, p.y);
        ctx.moveTo(p.x, p.y - r * 1.6);
        ctx.lineTo(p.x, p.y + r * 1.6);
        ctx.stroke();
        ctx.font = `${11 * dpr}px system-ui, sans-serif`;
        ctx.fillText("COM", p.x + r + 4 * dpr, p.y + r + 8 * dpr);
      }
    }

    /**
     * @param {(p: Vec3) => { x: number, y: number } | null} pr
     * @param {Vec3} a
     * @param {Vec3} c
     */
    function line(pr, a, c) {
      const pa = pr(a);
      const pc = pr(c);
      if (!pa || !pc) return;
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pc.x, pc.y);
    }
  };

  // ------------------------------------------------------------ interaction
  /** @type {Map<number, { x: number, y: number }>} */
  const pointers = new Map();
  let moved = 0;
  let pinch = 0;
  canvas.addEventListener("pointerdown", (e) => {
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    moved = 0;
    if (pointers.size === 2) pinch = pinchDistance();
  });
  canvas.addEventListener("pointermove", (e) => {
    const prev = pointers.get(e.pointerId);
    if (!prev) return;
    const dx = e.clientX - prev.x;
    const dy = e.clientY - prev.y;
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    moved += Math.abs(dx) + Math.abs(dy);
    if (pointers.size === 2) {
      const d = pinchDistance();
      if (pinch > 0 && d > 0) zoom(pinch / d);
      pinch = d;
      return;
    }
    if (e.shiftKey || e.buttons === 4 || e.buttons === 2) {
      pan(dx, dy);
    } else {
      cam.yaw -= dx * 0.01;
      cam.pitch = Math.max(-1.5, Math.min(1.5, cam.pitch + dy * 0.01));
    }
    render();
  });
  /** @param {PointerEvent} e */
  const up = (e) => {
    pointers.delete(e.pointerId);
    if (pointers.size < 2) pinch = 0;
    if (e.type === "pointerup" && moved < 4) pick(e);
  };
  canvas.addEventListener("pointerup", up);
  canvas.addEventListener("pointercancel", up);
  canvas.addEventListener("contextmenu", (e) => e.preventDefault());
  canvas.addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      zoom(Math.exp(e.deltaY * 0.0015));
    },
    { passive: false },
  );
  canvas.addEventListener("dblclick", reset);

  function pinchDistance() {
    const ps = [...pointers.values()];
    return ps.length === 2 ? Math.hypot(ps[0].x - ps[1].x, ps[0].y - ps[1].y) : 0;
  }
  /** @param {number} factor */
  function zoom(factor) {
    cam.dist = Math.max(0.01, Math.min(1e3, cam.dist * factor));
    render();
  }
  /** @param {number} dx @param {number} dy */
  function pan(dx, dy) {
    const { r, u } = cameraBasis(cam);
    const k = (cam.dist * Math.tan(cam.fov / 2) * 2) / canvas.getBoundingClientRect().height;
    cam.target = [
      cam.target[0] - (dx * r[0] - dy * u[0]) * k,
      cam.target[1] - (dx * r[1] - dy * u[1]) * k,
      cam.target[2] - (dx * r[2] - dy * u[2]) * k,
    ];
  }
  /** @param {PointerEvent} e */
  function pick(e) {
    const rect = canvas.getBoundingClientRect();
    const dpr = canvas.width / Math.max(1, rect.width);
    const x = (e.clientX - rect.left) * dpr;
    const y = (e.clientY - rect.top) * dpr;
    let best = null;
    let bestD = 14 * dpr;
    for (const p of pickable) {
      const d = Math.hypot(p.x - x, p.y - y);
      if (d < bestD) {
        bestD = d;
        best = p.link;
      }
    }
    if (best) hooks.onPick(best);
  }

  new ResizeObserver(() => render()).observe(canvas);

  return {
    /** @param {SceneInput} s @param {boolean} refit */
    set(s, refit) {
      scene = s;
      if (refit || !fitted) {
        cam = fitCamera(sceneBounds(s));
        fitted = true;
      }
      render();
    },
    reset,
    render,
  };
}

/** @param {number} x */
function niceStep(x) {
  const p = 10 ** Math.floor(Math.log10(Math.max(x, 1e-6)));
  const m = x / p;
  return (m < 2 ? 1 : m < 5 ? 2 : 5) * p;
}

/** @param {string} css @returns {number[]} */
function parseColor(css) {
  const m = /^#([0-9a-f]{6})$/i.exec(css);
  if (!m) return [150, 155, 160];
  const v = parseInt(m[1], 16);
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255];
}
