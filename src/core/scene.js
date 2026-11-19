// 3D scene for the browser view, independent of any drawing API: primitive
// shapes tessellated into world-space triangles and edges, link frames,
// the joint skeleton and an orbit camera with perspective projection.
// World frame is the URDF root frame, z up (REP 103).

import {
  add,
  apply,
  column,
  compose,
  cross,
  dot,
  fromXyzRpy,
  norm,
  normalize,
  scale,
  sub,
} from "./math.js";

/** @typedef {import("./math.js").Vec3} Vec3 */
/** @typedef {import("./math.js").Transform} Transform */
/** @typedef {import("./tree.js").KinTree} KinTree */

/**
 * @typedef {object} Solid
 * @property {string} link
 * @property {"box" | "cylinder" | "sphere"} type
 * @property {Vec3[]} vertices world coordinates
 * @property {number[][]} faces vertex indices of each planar polygon, counter-clockwise from outside
 * @property {[number, number][]} edges outline edges (no tessellation seams on smooth surfaces)
 * @property {number[] | null} rgba material colour, if any
 */

/**
 * Unit primitives in their own frame.
 * @param {import("./urdf.js").Geometry} g
 * @param {number} [segments] around a cylinder or sphere
 * @returns {{ vertices: Vec3[], faces: number[][], edges: [number, number][] } | null}
 */
export function tessellate(g, segments = 16) {
  if (g.type === "box") {
    const [a, b, c] = g.size.map((v) => v / 2);
    /** @type {Vec3[]} */
    const vertices = [];
    for (const z of [-c, c])
      for (const y of [-b, b]) for (const x of [-a, a]) vertices.push([x, y, z]);
    // index = ix + 2 iy + 4 iz
    const faces = [
      [0, 2, 3, 1], // -z
      [4, 5, 7, 6], // +z
      [0, 1, 5, 4], // -y
      [2, 6, 7, 3], // +y
      [0, 4, 6, 2], // -x
      [1, 3, 7, 5], // +x
    ];
    /** @type {[number, number][]} */
    const edges = [
      [0, 1],
      [2, 3],
      [4, 5],
      [6, 7],
      [0, 2],
      [1, 3],
      [4, 6],
      [5, 7],
      [0, 4],
      [1, 5],
      [2, 6],
      [3, 7],
    ];
    return { vertices, faces, edges };
  }
  if (g.type === "cylinder") {
    const n = segments;
    const h = g.length / 2;
    /** @type {Vec3[]} */
    const vertices = [];
    for (let k = 0; k < n; k++) {
      const t = (2 * Math.PI * k) / n;
      vertices.push([g.radius * Math.cos(t), g.radius * Math.sin(t), -h]);
    }
    for (let k = 0; k < n; k++) {
      const t = (2 * Math.PI * k) / n;
      vertices.push([g.radius * Math.cos(t), g.radius * Math.sin(t), h]);
    }
    /** @type {number[][]} */
    const faces = [];
    /** @type {[number, number][]} */
    const edges = [];
    for (let k = 0; k < n; k++) {
      const k2 = (k + 1) % n;
      faces.push([k, k2, n + k2, n + k]);
      edges.push([k, k2], [n + k, n + k2]);
      if (k % (n / 4) === 0) edges.push([k, n + k]);
    }
    faces.push([...Array(n).keys()].reverse());
    faces.push([...Array(n).keys()].map((k) => n + k));
    return { vertices, faces, edges };
  }
  if (g.type === "sphere") {
    const n = segments;
    const rings = Math.max(4, n / 2);
    /** @type {Vec3[]} */
    const vertices = [[0, 0, -g.radius]];
    for (let i = 1; i < rings; i++) {
      const phi = -Math.PI / 2 + (Math.PI * i) / rings;
      for (let k = 0; k < n; k++) {
        const t = (2 * Math.PI * k) / n;
        vertices.push([
          g.radius * Math.cos(phi) * Math.cos(t),
          g.radius * Math.cos(phi) * Math.sin(t),
          g.radius * Math.sin(phi),
        ]);
      }
    }
    vertices.push([0, 0, g.radius]);
    const top = vertices.length - 1;
    /** @param {number} i @param {number} k */
    const at = (i, k) => 1 + (i - 1) * n + (k % n);
    /** @type {number[][]} */
    const faces = [];
    /** @type {[number, number][]} */
    const edges = [];
    for (let k = 0; k < n; k++) {
      faces.push([0, at(1, k + 1), at(1, k)]);
      faces.push([top, at(rings - 1, k), at(rings - 1, k + 1)]);
      for (let i = 1; i < rings - 1; i++)
        faces.push([at(i, k), at(i, k + 1), at(i + 1, k + 1), at(i + 1, k)]);
      // Equator and meridians every quarter turn as the outline.
      const eq = Math.floor(rings / 2);
      edges.push([at(eq, k), at(eq, k + 1)]);
      if (k % (n / 4) === 0) {
        edges.push([0, at(1, k)], [at(rings - 1, k), top]);
        for (let i = 1; i < rings - 1; i++) edges.push([at(i, k), at(i + 1, k)]);
      }
    }
    return { vertices, faces, edges };
  }
  return null;
}

/**
 * World-space solids for every link pose.
 * @param {KinTree} tree
 * @param {Map<string, Transform>} poses
 * @param {"visual" | "collision"} which
 * @param {Map<string, { rgba: number[] | null }>} [materials] top-level materials
 * @returns {{ solids: Solid[], meshes: { link: string, at: Vec3, filename: string }[] }}
 */
export function buildSolids(tree, poses, which, materials = new Map()) {
  /** @type {Solid[]} */
  const solids = [];
  /** @type {{ link: string, at: Vec3, filename: string }[]} */
  const meshes = [];
  for (const name of tree.order) {
    const link = tree.links.get(name);
    const pose = poses.get(name);
    if (!link || !pose) continue;
    for (const s of which === "visual" ? link.visuals : link.collisions) {
      const g = s.geometry;
      if (!g) continue;
      const T = compose(pose, fromXyzRpy(s.origin.xyz, s.origin.rpy));
      if (g.type === "mesh") {
        meshes.push({ link: name, at: T.p, filename: g.filename });
        continue;
      }
      if (g.type === "unknown") continue;
      const dims =
        g.type === "box" ? g.size : g.type === "cylinder" ? [g.radius, g.length] : [g.radius];
      if (!dims.every((d) => d > 0 && d < 1e3)) continue;
      const mesh = /** @type {NonNullable<ReturnType<typeof tessellate>>} */ (tessellate(g));
      let rgba = s.material?.rgba ?? null;
      if (!rgba && s.material?.name) rgba = materials.get(s.material.name)?.rgba ?? null;
      solids.push({
        link: name,
        type: g.type,
        vertices: mesh.vertices.map((v) => apply(T, v)),
        faces: mesh.faces,
        edges: mesh.edges,
        rgba,
      });
    }
  }
  return { solids, meshes };
}

/**
 * Axis-aligned bounds of solids, link origins and extra points.
 * @param {Vec3[]} points
 * @returns {{ min: Vec3, max: Vec3, centre: Vec3, radius: number }}
 */
export function bounds(points) {
  if (points.length === 0)
    return { min: [0, 0, 0], max: [0, 0, 0], centre: [0, 0, 0], radius: 0.5 };
  /** @type {Vec3} */
  const min = [Infinity, Infinity, Infinity];
  /** @type {Vec3} */
  const max = [-Infinity, -Infinity, -Infinity];
  for (const p of points)
    for (let k = 0; k < 3; k++) {
      min[k] = Math.min(min[k], p[k]);
      max[k] = Math.max(max[k], p[k]);
    }
  const centre = scale(add(min, max), 0.5);
  return { min, max, centre, radius: Math.max(norm(sub(max, centre)), 0.05) };
}

/**
 * @typedef {object} Camera
 * @property {number} yaw rad, about world z
 * @property {number} pitch rad, elevation above the xy plane
 * @property {number} dist m from the target
 * @property {Vec3} target
 * @property {number} fov vertical field of view, rad
 */

/**
 * Camera basis: eye position, right, up and forward unit vectors.
 * @param {Camera} c
 */
export function cameraBasis(c) {
  const dir = [
    Math.cos(c.pitch) * Math.cos(c.yaw),
    Math.cos(c.pitch) * Math.sin(c.yaw),
    Math.sin(c.pitch),
  ];
  const eye = add(c.target, scale(/** @type {Vec3} */ (dir), c.dist));
  const f = normalize(sub(c.target, eye));
  let r = cross(f, [0, 0, 1]);
  if (norm(r) < 1e-9) r = [0, 1, 0];
  r = normalize(r);
  const u = cross(r, f);
  return { eye, r, u, f };
}

/**
 * Perspective projection to pixels; null behind the near plane.
 * @param {Camera} c
 * @param {number} width
 * @param {number} height
 * @returns {(p: Vec3) => { x: number, y: number, depth: number } | null}
 */
export function projector(c, width, height) {
  const { eye, r, u, f } = cameraBasis(c);
  const focal = height / (2 * Math.tan(c.fov / 2));
  const near = c.dist * 1e-3;
  return (p) => {
    const d = sub(p, eye);
    const z = dot(d, f);
    if (z < near) return null;
    return {
      x: width / 2 + (focal * dot(d, r)) / z,
      y: height / 2 - (focal * dot(d, u)) / z,
      depth: z,
    };
  };
}

/**
 * A camera that frames a sphere of the given radius, looking from the
 * front-left and slightly above, like the default RViz orbit view.
 * @param {{ centre: Vec3, radius: number }} b
 * @returns {Camera}
 */
export function fitCamera(b) {
  const fov = (40 * Math.PI) / 180;
  return {
    yaw: (35 * Math.PI) / 180,
    pitch: (25 * Math.PI) / 180,
    dist: (b.radius / Math.sin(fov / 2)) * 1.1,
    target: b.centre,
    fov,
  };
}

/**
 * Outward unit normal of a planar polygon (Newell's method).
 * @param {Vec3[]} pts
 * @returns {Vec3}
 */
export function faceNormal(pts) {
  /** @type {Vec3} */
  let n = [0, 0, 0];
  for (let k = 0; k < pts.length; k++) {
    const a = pts[k];
    const b = pts[(k + 1) % pts.length];
    n = add(n, [
      (a[1] - b[1]) * (a[2] + b[2]),
      (a[2] - b[2]) * (a[0] + b[0]),
      (a[0] - b[0]) * (a[1] + b[1]),
    ]);
  }
  return normalize(n);
}

/**
 * Link frame axes (origin and unit x, y, z in the world) for drawing.
 * @param {Transform} pose
 * @param {number} length
 * @returns {{ o: Vec3, x: Vec3, y: Vec3, z: Vec3 }}
 */
export function frameAxes(pose, length) {
  return {
    o: pose.p,
    x: add(pose.p, scale(column(pose.R, 0), length)),
    y: add(pose.p, scale(column(pose.R, 1), length)),
    z: add(pose.p, scale(column(pose.R, 2), length)),
  };
}
