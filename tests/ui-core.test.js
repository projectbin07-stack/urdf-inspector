// DOM-free pieces of the browser app: scene tessellation and projection,
// tree layout, slider ranges and URL state.

import assert from "node:assert/strict";
import { test } from "node:test";
import { EXAMPLES } from "../src/core/examples.js";
import { forwardKinematics, sliderRange } from "../src/core/fk.js";
import { inspectText } from "../src/core/inspect.js";
import { ancestors, layoutTree } from "../src/core/layout.js";
import {
  bounds,
  buildSolids,
  cameraBasis,
  faceNormal,
  fitCamera,
  frameAxes,
  projector,
  tessellate,
} from "../src/core/scene.js";
import { decodeState, defaultState, encodeState } from "../src/ui/state.js";
import { example } from "./helpers.js";

/** @param {string} name */
const ctxOf = (name) => {
  const c = inspectText(example(name)).context;
  assert.ok(c);
  return c;
};

test("tessellated primitives are closed with outward normals", () => {
  for (const geo of /** @type {import("../src/core/urdf.js").Geometry[]} */ ([
    { type: "box", size: [0.4, 0.2, 0.1] },
    { type: "cylinder", radius: 0.1, length: 0.3 },
    { type: "sphere", radius: 0.2 },
  ])) {
    const m = tessellate(geo);
    assert.ok(m);
    for (const face of m.faces) {
      const pts = face.map((k) => m.vertices[k]);
      const n = faceNormal(pts);
      const c = pts.reduce((a, p) => [a[0] + p[0], a[1] + p[1], a[2] + p[2]], [0, 0, 0]);
      // The face centre and its normal point the same way from the shape centre.
      assert.ok(n[0] * c[0] + n[1] * c[1] + n[2] * c[2] > 0, `${geo.type} face ${face}`);
    }
    // Every edge appears in each face list at most twice (closed surface).
    /** @type {Map<string, number>} */
    const count = new Map();
    for (const face of m.faces)
      face.forEach((a, k) => {
        const b = face[(k + 1) % face.length];
        const key = a < b ? `${a}-${b}` : `${b}-${a}`;
        count.set(key, (count.get(key) ?? 0) + 1);
      });
    for (const v of count.values()) assert.equal(v, 2);
  }
  const box = tessellate({ type: "box", size: [0.4, 0.2, 0.1] });
  assert.deepEqual(box?.vertices[7], [0.2, 0.1, 0.05]);
  assert.equal(tessellate({ type: "mesh", filename: "a.stl", scale: [1, 1, 1] }), null);
});

test("solids are placed by link pose and shape origin; meshes become markers", () => {
  const ctx = ctxOf("mecanum_base.urdf");
  const { solids, meshes } = buildSolids(ctx.tree, ctx.poses, "visual", ctx.robot.materials);
  assert.equal(meshes.length, 0);
  assert.equal(solids.length, 7);
  const chassis = solids.find((s) => s.link === "base_link");
  assert.ok(chassis);
  // base_link is 0.04 above base_footprint and the box is centred 0.03 above base_link.
  const b = bounds(chassis.vertices);
  assert.deepEqual(
    b.min.map((v) => Number(v.toFixed(9))),
    [-0.15, -0.11, 0.04],
  );
  assert.deepEqual(chassis.rgba, [0.55, 0.57, 0.6, 1]);
  const broken = ctxOf("broken_robot.urdf");
  const bm = buildSolids(broken.tree, broken.poses, "collision");
  assert.equal(bm.meshes.length, 1);
  assert.equal(bm.meshes[0].filename, "https://example.com/meshes/pan.stl");
});

test("camera basis is orthonormal and projection centres the target", () => {
  const cam = fitCamera({ centre: [1, 2, 3], radius: 0.5 });
  const { r, u, f } = cameraBasis(cam);
  /** @param {number[]} a @param {number[]} b */
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  for (const [a, b] of [
    [r, u],
    [u, f],
    [r, f],
  ])
    assert.ok(Math.abs(dot(a, b)) < 1e-12);
  assert.ok(u[2] > 0, "world z projects upwards");
  const project = projector(cam, 400, 300);
  const p = project([1, 2, 3]);
  assert.ok(p);
  assert.ok(Math.abs(p.x - 200) < 1e-9 && Math.abs(p.y - 150) < 1e-9);
  assert.ok(Math.abs(p.depth - cam.dist) < 1e-9);
  // A point above the target appears higher on the screen.
  const above = project([1, 2, 3.1]);
  assert.ok(above && above.y < 150);
  // Behind the camera: not drawn.
  const { eye } = cameraBasis(cam);
  assert.equal(
    project([eye[0] + (eye[0] - 1), eye[1] + (eye[1] - 2), eye[2] + (eye[2] - 3)]),
    null,
  );
  // Looking straight down still gives a valid basis.
  const top = cameraBasis({ ...cam, pitch: Math.PI / 2 });
  assert.ok(Math.abs(Math.hypot(...top.r) - 1) < 1e-12);
  // The fitted sphere fits inside the vertical field of view.
  assert.ok(Math.sin(cam.fov / 2) * cam.dist > 0.5);
});

test("bounds and frame axes", () => {
  assert.equal(bounds([]).radius, 0.5);
  const b = bounds([
    [0, 0, 0],
    [2, 0, 0],
  ]);
  assert.deepEqual(b.centre, [1, 0, 0]);
  assert.equal(b.radius, 1);
  const ax = frameAxes({ R: [0, -1, 0, 1, 0, 0, 0, 0, 1], p: [1, 1, 1] }, 0.5);
  assert.deepEqual(ax.x, [1, 1.5, 1]);
  assert.deepEqual(ax.y, [0.5, 1, 1]);
  assert.deepEqual(ax.z, [1, 1, 1.5]);
});

test("tree layout rows, indentation and collapsing", () => {
  const ctx = ctxOf("mecanum_base.urdf");
  const rows = layoutTree(ctx.tree, new Set());
  assert.equal(rows.length, 10);
  assert.deepEqual(
    rows.slice(0, 3).map((r) => [r.link, r.depth, r.joint, r.parentRow]),
    [
      ["base_footprint", 0, null, null],
      ["base_link", 1, "base_joint", 0],
      ["front_left_wheel", 2, "front_left_wheel_joint", 1],
    ],
  );
  const cam = rows.find((r) => r.link === "camera_optical_frame");
  assert.equal(cam?.depth, 3);
  const collapsed = layoutTree(ctx.tree, new Set(["base_link"]));
  assert.equal(collapsed.length, 2);
  assert.equal(collapsed[1].collapsed, true);
  assert.equal(collapsed[1].children, 7);
  // Collapsing a leaf has no effect.
  assert.equal(layoutTree(ctx.tree, new Set(["imu_link"])).length, 10);
  assert.deepEqual(ancestors(ctx.tree, "camera_optical_frame"), [
    "base_footprint",
    "base_link",
    "camera_link",
  ]);
});

test("tree layout lists links outside the main tree", () => {
  const ctx = ctxOf("broken_robot.urdf");
  const rows = layoutTree(ctx.tree, new Set());
  const spare = rows.find((r) => r.link === "spare_link");
  assert.equal(spare?.connected, false);
  assert.equal(rows.length, ctx.tree.links.size);
});

test("slider ranges follow limits, with fallbacks", () => {
  const ctx = ctxOf("broken_robot.urdf");
  const j = (/** @type {string} */ n) => /** @type {any} */ (ctx.tree.joints.get(n));
  assert.deepEqual(sliderRange(j("lift_joint")), { min: 0, max: 0.1 });
  // lower > upper: fall back to one turn.
  assert.deepEqual(sliderRange(j("pan_joint")), { min: -Math.PI, max: Math.PI });
  // continuous ignores its (invalid) limits.
  assert.deepEqual(sliderRange(j("right_wheel_joint")), { min: -Math.PI, max: Math.PI });
  assert.deepEqual(sliderRange({ ...j("lift_joint"), limit: null }), { min: -0.5, max: 0.5 });
  // Poses follow the slider values.
  const poses = forwardKinematics(ctx.tree, { lift_joint: 0.05 });
  assert.ok(poses.get("lift"));
});

test("URL state round trip, defaults and rejection of bad values", () => {
  assert.equal(encodeState(defaultState()), "");
  const st = defaultState();
  st.source = "arm";
  st.selected = { kind: "joint", name: "elbow_joint" };
  st.view = "wire";
  st.geometry = "collision";
  st.frames = false;
  st.com = false;
  st.q = { shoulder_joint: 0.5, elbow_joint: -0.25, zero: 0 };
  st.theme = "dark";
  const qs = encodeState(st);
  assert.equal(
    qs,
    "s=arm&sel=j%3Aelbow_joint&v=wire&g=collision&ax=0&com=0&q=elbow_joint%3A-0.25%2Cshoulder_joint%3A0.5&theme=dark",
  );
  const back = decodeState(`?${qs}`);
  assert.deepEqual(back, { ...st, q: { shoulder_joint: 0.5, elbow_joint: -0.25 } });
  // Joint positions are not written for local files (names would not match anything).
  assert.equal(encodeState({ ...st, source: "file", q: { a: 1 } }).includes("q="), false);
  const bad = decodeState("s=nope&sel=x:y&v=3d&q=a:1,b:x,:2,c,d:1e9,%3Cscript%3E:1&theme=pink");
  assert.deepEqual(bad, { ...defaultState(), q: { a: 1 } });
  assert.equal(decodeState("sel=l:base_link").selected?.kind, "link");
  for (const e of EXAMPLES) assert.equal(decodeState(`s=${e.id}`).source, e.id);
  assert.equal(decodeState("s=paste").source, "paste");
});
