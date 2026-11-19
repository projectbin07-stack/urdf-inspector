// URDF model reader: XmlElement tree -> plain robot model plus input
// findings (missing attributes, malformed numbers, unknown types, xacro).
// Written from the URDF XML specification (ROS wiki, urdfdom's urdf.xsd);
// defaults follow the specification: origin 0, axis (1 0 0), mimic
// multiplier 1 and offset 0, mesh scale (1 1 1).

import { finding } from "./findings.js";

/** @typedef {import("./xml.js").XmlElement} XmlElement */
/** @typedef {import("./findings.js").Finding} Finding */
/** @typedef {import("./math.js").Vec3} Vec3 */

/** @typedef {{ xyz: Vec3, rpy: Vec3 }} Origin */
/**
 * @typedef {{ type: "box", size: Vec3 }
 *   | { type: "cylinder", radius: number, length: number }
 *   | { type: "sphere", radius: number }
 *   | { type: "mesh", filename: string, scale: Vec3 }
 *   | { type: "unknown", tag: string }} Geometry
 */
/**
 * @typedef {object} Shape
 * @property {"visual" | "collision"} kind
 * @property {string} name
 * @property {Origin} origin
 * @property {Geometry | null} geometry
 * @property {{ name: string, rgba: number[] | null } | null} material
 * @property {string} path
 */
/** @typedef {{ ixx: number, ixy: number, ixz: number, iyy: number, iyz: number, izz: number }} InertiaTensor */
/**
 * @typedef {object} Inertial
 * @property {Origin} origin
 * @property {number | null} mass
 * @property {InertiaTensor | null} inertia
 * @property {string} path
 */
/**
 * @typedef {object} Link
 * @property {string} name
 * @property {string} path
 * @property {number} line
 * @property {Inertial | null} inertial
 * @property {Shape[]} visuals
 * @property {Shape[]} collisions
 */
/**
 * @typedef {object} Limit
 * @property {number | null} lower null when the attribute is absent
 * @property {number | null} upper
 * @property {number | null} effort
 * @property {number | null} velocity
 * @property {string} path
 */
/**
 * @typedef {object} Joint
 * @property {string} name
 * @property {JointType | string} type
 * @property {string} parent
 * @property {string} child
 * @property {Origin} origin
 * @property {Vec3} axis as written, not normalised
 * @property {boolean} axisGiven
 * @property {Limit | null} limit
 * @property {{ damping: number, friction: number } | null} dynamics
 * @property {{ joint: string, multiplier: number, offset: number } | null} mimic
 * @property {string} path
 * @property {number} line
 */
/** @typedef {"revolute" | "continuous" | "prismatic" | "fixed" | "floating" | "planar"} JointType */
/**
 * @typedef {object} Ros2ControlJoint
 * @property {string} name
 * @property {string[]} command
 * @property {string[]} state
 * @property {string} path
 * @property {number} line
 */
/**
 * @typedef {object} Ros2Control
 * @property {string} name
 * @property {string} type system, actuator or sensor
 * @property {string} plugin
 * @property {Ros2ControlJoint[]} joints
 * @property {string[]} sensors
 * @property {string} path
 * @property {number} line
 */
/**
 * @typedef {object} GazeboTag
 * @property {string} reference empty for robot-level tags
 * @property {string[]} elements child element names
 * @property {{ name: string, filename: string }[]} plugins
 * @property {string} path
 * @property {number} line
 */
/**
 * @typedef {object} Robot
 * @property {string} name
 * @property {string} version
 * @property {Link[]} links
 * @property {Joint[]} joints
 * @property {Map<string, { rgba: number[] | null, path: string }>} materials top-level materials
 * @property {Ros2Control[]} ros2Control
 * @property {GazeboTag[]} gazebo
 * @property {{ name: string, joints: string[], path: string, line: number }[]} transmissions
 */

export const JOINT_TYPES = ["revolute", "continuous", "prismatic", "fixed", "floating", "planar"];

/** Escape a name for use inside an element path. @param {string} s */
const q = (s) => s.replace(/'/g, "&apos;");

/**
 * @param {string} tag
 * @param {string} name
 */
export const namedPath = (tag, name) => `/robot/${tag}[@name='${q(name)}']`;

/**
 * Find unexpanded xacro: xacro: elements, ${...} expressions and $(...)
 * substitutions in attribute values or text.
 * @param {XmlElement} root
 * @returns {Finding[]}
 */
export function detectXacro(root) {
  /** @type {Finding[]} */
  const out = [];
  let more = 0;
  const LIMIT = 25;
  /** @param {string} message @param {string} path @param {number} line */
  const add = (message, path, line) => {
    if (out.length < LIMIT) out.push(finding("xacro", "error", message, path, el0, line));
    else more++;
  };
  /** @type {Finding["target"]} */
  const el0 = { kind: "element", name: "" };
  /** @param {XmlElement} el @param {string} path */
  const walk = (el, path) => {
    if (el.name.startsWith("xacro:")) {
      const what = el.attrs.name ? ` '${el.attrs.name}'` : "";
      add(`<${el.name}${what}> is a xacro element`, path, el.line);
    }
    for (const [k, v] of Object.entries(el.attrs)) {
      if (/\$\{[^}]*\}/.test(v) || /\$\([^)]*\)/.test(v))
        add(`${k}="${v}" contains a xacro expression`, `${path}/@${k}`, el.line);
    }
    if (/\$\{[^}]*\}|\$\([^)]*\)/.test(el.text))
      add(`text "${el.text.trim().slice(0, 60)}" contains a xacro expression`, path, el.line);
    const counts = new Map();
    for (const c of el.children) {
      const k = (counts.get(c.name) ?? 0) + 1;
      counts.set(c.name, k);
      const label = c.attrs.name !== undefined ? `[@name='${q(c.attrs.name)}']` : `[${k}]`;
      walk(c, `${path}/${c.name}${label}`);
    }
  };
  walk(root, `/${root.name}`);
  if (more > 0)
    out.push(finding("xacro", "error", `${more} more xacro constructs not listed`, "/robot", el0));
  return out;
}

/**
 * Parse the whole URDF model.
 * @param {XmlElement} root
 * @returns {{ robot: Robot | null, findings: Finding[] }}
 */
export function readUrdf(root) {
  /** @type {Finding[]} */
  const findings = [];
  if (root.name !== "robot") {
    findings.push(
      finding(
        "not-urdf",
        "error",
        `root element is <${root.name}>, expected <robot>` +
          (root.name === "sdf" ? " (this looks like SDF, which is not supported)" : ""),
        `/${root.name}`,
        { kind: "element", name: root.name },
        root.line,
      ),
    );
    return { robot: null, findings };
  }
  const xacro = detectXacro(root);
  if (xacro.length > 0) return { robot: null, findings: xacro };

  /** @type {Finding["target"]} */
  let target = { kind: "robot", name: root.attrs.name ?? "" };

  /**
   * @param {string} id
   * @param {string} message
   * @param {string} path
   * @param {number} line
   * @param {import("./findings.js").Severity} [sev]
   */
  const report = (id, message, path, line, sev = "error") =>
    findings.push(finding(id, sev, message, path, target, line));

  /**
   * @param {XmlElement} el
   * @param {string} attr
   * @param {string} path
   * @returns {string | null}
   */
  const required = (el, attr, path) => {
    const v = el.attrs[attr];
    if (v === undefined || v.trim() === "") {
      report("missing-attribute", `<${el.name}> has no ${attr} attribute`, path, el.line);
      return null;
    }
    return v;
  };

  /**
   * @param {string | undefined} raw
   * @param {number} fallback
   * @param {string} what
   * @param {string} path
   * @param {number} line
   */
  const num = (raw, fallback, what, path, line) => {
    if (raw === undefined) return fallback;
    const v = Number(raw.trim());
    if (raw.trim() === "" || !Number.isFinite(v)) {
      report("bad-number", `${what}="${raw}" is not a finite number`, path, line);
      return fallback;
    }
    return v;
  };

  /**
   * @param {string | undefined} raw
   * @param {Vec3} fallback
   * @param {string} what
   * @param {string} path
   * @param {number} line
   * @returns {Vec3}
   */
  const vec3 = (raw, fallback, what, path, line) => {
    if (raw === undefined) return [fallback[0], fallback[1], fallback[2]];
    const parts = raw.trim().split(/\s+/).filter(Boolean);
    const vals = parts.map(Number);
    if (vals.length !== 3 || !vals.every(Number.isFinite)) {
      report("bad-number", `${what}="${raw}" must be three numbers`, path, line);
      return [fallback[0], fallback[1], fallback[2]];
    }
    return [vals[0], vals[1], vals[2]];
  };

  /** @param {XmlElement} parent @param {string} tag */
  const first = (parent, tag) => parent.children.find((c) => c.name === tag);

  /**
   * @param {XmlElement | undefined} el
   * @param {string} path
   * @returns {Origin}
   */
  const origin = (el, path) => {
    if (!el) return { xyz: [0, 0, 0], rpy: [0, 0, 0] };
    const p = `${path}/origin`;
    return {
      xyz: vec3(el.attrs.xyz, [0, 0, 0], "xyz", p, el.line),
      rpy: vec3(el.attrs.rpy, [0, 0, 0], "rpy", p, el.line),
    };
  };

  /** @param {string | undefined} raw @param {string} path @param {number} line */
  const rgba = (raw, path, line) => {
    if (raw === undefined) return null;
    const v = raw.trim().split(/\s+/).map(Number);
    if (v.length !== 4 || !v.every(Number.isFinite)) {
      report("bad-number", `rgba="${raw}" must be four numbers`, path, line);
      return null;
    }
    return v;
  };

  /**
   * @param {XmlElement | undefined} el
   * @param {string} path
   * @returns {Geometry | null}
   */
  const geometry = (el, path) => {
    if (!el) return null;
    const p = `${path}/geometry`;
    const shape = el.children[0];
    if (!shape) return null;
    const sp = `${p}/${shape.name}`;
    if (shape.name === "box")
      return { type: "box", size: vec3(shape.attrs.size, [0, 0, 0], "size", sp, shape.line) };
    if (shape.name === "cylinder") {
      const r = required(shape, "radius", sp);
      const l = required(shape, "length", sp);
      return {
        type: "cylinder",
        radius: num(r ?? undefined, 0, "radius", sp, shape.line),
        length: num(l ?? undefined, 0, "length", sp, shape.line),
      };
    }
    if (shape.name === "sphere") {
      const r = required(shape, "radius", sp);
      return { type: "sphere", radius: num(r ?? undefined, 0, "radius", sp, shape.line) };
    }
    if (shape.name === "mesh") {
      const f = required(shape, "filename", sp);
      return {
        type: "mesh",
        filename: f ?? "",
        scale: vec3(shape.attrs.scale, [1, 1, 1], "scale", sp, shape.line),
      };
    }
    report(
      "unknown-geometry",
      `geometry <${shape.name}> is not box, cylinder, sphere or mesh`,
      sp,
      shape.line,
      "warning",
    );
    return { type: "unknown", tag: shape.name };
  };

  /** @type {Robot} */
  const robot = {
    name: root.attrs.name ?? "",
    version: root.attrs.version ?? "1.0",
    links: [],
    joints: [],
    materials: new Map(),
    ros2Control: [],
    gazebo: [],
    transmissions: [],
  };
  if (root.attrs.name === undefined)
    report("missing-attribute", "<robot> has no name attribute", "/robot", root.line);
  if (root.attrs.version !== undefined && !/^1\.[012]$/.test(root.attrs.version.trim()))
    report(
      "version-unsupported",
      `version="${root.attrs.version}" (urdfdom accepts 1.0 to 1.2)`,
      "/robot/@version",
      root.line,
      "warning",
    );
  const v12 = /^1\.2$/.test(robot.version.trim());

  const counters = new Map();
  for (const el of root.children) {
    const k = (counters.get(el.name) ?? 0) + 1;
    counters.set(el.name, k);
    const path =
      el.attrs.name !== undefined ? namedPath(el.name, el.attrs.name) : `/robot/${el.name}[${k}]`;
    if (el.name === "link") robot.links.push(readLink(el, path));
    else if (el.name === "joint") robot.joints.push(readJoint(el, path));
    else if (el.name === "material") {
      target = { kind: "material", name: el.attrs.name ?? "" };
      const name = required(el, "name", path);
      if (name !== null) {
        if (robot.materials.has(name))
          report("duplicate-material", `material '${name}' is defined twice`, path, el.line);
        else {
          const c = first(el, "color");
          robot.materials.set(name, {
            rgba: c ? rgba(c.attrs.rgba, `${path}/color`, c.line) : null,
            path,
          });
        }
      }
    } else if (el.name === "ros2_control") robot.ros2Control.push(readRos2Control(el, path));
    else if (el.name === "gazebo") robot.gazebo.push(readGazebo(el, path));
    else if (el.name === "transmission") {
      robot.transmissions.push({
        name: el.attrs.name ?? "",
        joints: el.children.filter((c) => c.name === "joint").map((c) => c.attrs.name ?? ""),
        path,
        line: el.line,
      });
    } else {
      target = { kind: "element", name: el.name };
      report(
        "unknown-element",
        `<${el.name}> is not a URDF element and is ignored`,
        path,
        el.line,
        "info",
      );
    }
  }

  // Materials referenced by name only must be defined at the top level.
  for (const link of robot.links) {
    for (const v of link.visuals) {
      if (v.material && v.material.rgba === null && v.material.name !== "") {
        if (!robot.materials.has(v.material.name)) {
          target = { kind: "link", name: link.name };
          report(
            "undefined-material",
            `material '${v.material.name}' is used by link '${link.name}' but not defined`,
            `${v.path}/material`,
            link.line,
            "warning",
          );
        }
      }
    }
  }
  return { robot, findings };

  /**
   * @param {XmlElement} el
   * @param {string} path
   * @returns {Link}
   */
  function readLink(el, path) {
    target = { kind: "link", name: el.attrs.name ?? "" };
    const name = required(el, "name", path) ?? "";
    /** @type {Link} */
    const link = { name, path, line: el.line, inertial: null, visuals: [], collisions: [] };
    const inert = first(el, "inertial");
    if (inert) {
      const ip = `${path}/inertial`;
      const massEl = first(inert, "mass");
      const inertiaEl = first(inert, "inertia");
      let mass = null;
      if (!massEl)
        report("missing-attribute", `inertial of '${name}' has no <mass>`, ip, inert.line);
      else {
        const raw = required(massEl, "value", `${ip}/mass`);
        if (raw !== null) mass = num(raw, NaN, "mass value", `${ip}/mass`, massEl.line);
        if (mass !== null && Number.isNaN(mass)) mass = null;
      }
      /** @type {InertiaTensor | null} */
      let tensor = null;
      if (!inertiaEl)
        report("missing-attribute", `inertial of '${name}' has no <inertia>`, ip, inert.line);
      else {
        const tp = `${ip}/inertia`;
        /** @type {Record<string, number>} */
        const t = {};
        let ok = true;
        for (const key of ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]) {
          const raw = required(inertiaEl, key, tp);
          const v = raw === null ? NaN : num(raw, NaN, key, tp, inertiaEl.line);
          if (Number.isNaN(v)) ok = false;
          t[key] = v;
        }
        if (ok) tensor = /** @type {InertiaTensor} */ (/** @type {unknown} */ (t));
      }
      link.inertial = {
        origin: origin(first(inert, "origin"), ip),
        mass,
        inertia: tensor,
        path: ip,
      };
    }
    for (const kind of /** @type {const} */ (["visual", "collision"])) {
      const list = el.children.filter((c) => c.name === kind);
      list.forEach((s, k) => {
        const sp = `${path}/${kind}[${k + 1}]`;
        const geomEl = first(s, "geometry");
        const geom = geometry(geomEl, sp);
        if (!geom)
          report(
            "missing-geometry",
            `${kind} ${k + 1} of link '${name}' has no ${geomEl ? "shape inside <geometry>" : "<geometry>"}`,
            sp,
            s.line,
          );
        let material = null;
        const m = kind === "visual" ? first(s, "material") : undefined;
        if (m) {
          const c = first(m, "color");
          material = {
            name: m.attrs.name ?? "",
            rgba: c ? rgba(c.attrs.rgba, `${sp}/material/color`, c.line) : null,
          };
        }
        const shape = {
          kind,
          name: s.attrs.name ?? "",
          origin: origin(first(s, "origin"), sp),
          geometry: geom,
          material,
          path: sp,
        };
        (kind === "visual" ? link.visuals : link.collisions).push(shape);
      });
    }
    return link;
  }

  /**
   * @param {XmlElement} el
   * @param {string} path
   * @returns {Joint}
   */
  function readJoint(el, path) {
    target = { kind: "joint", name: el.attrs.name ?? "" };
    const name = required(el, "name", path) ?? "";
    const typeRaw = el.attrs.type;
    if (typeRaw === undefined || !JOINT_TYPES.includes(typeRaw))
      report(
        "unknown-joint-type",
        typeRaw === undefined
          ? `joint '${name}' has no type`
          : `joint '${name}' has unknown type '${typeRaw}'`,
        path,
        el.line,
      );
    const parentEl = first(el, "parent");
    const childEl = first(el, "child");
    let parent = "";
    let child = "";
    if (!parentEl) report("missing-attribute", `joint '${name}' has no <parent>`, path, el.line);
    else parent = required(parentEl, "link", `${path}/parent`) ?? "";
    if (!childEl) report("missing-attribute", `joint '${name}' has no <child>`, path, el.line);
    else child = required(childEl, "link", `${path}/child`) ?? "";

    const axisEl = first(el, "axis");
    const axis = axisEl
      ? vec3(axisEl.attrs.xyz, [1, 0, 0], "axis xyz", `${path}/axis`, axisEl.line)
      : /** @type {Vec3} */ ([1, 0, 0]);

    /** @type {Limit | null} */
    let limit = null;
    const limEl = first(el, "limit");
    if (limEl) {
      const lp = `${path}/limit`;
      /** @param {string} key */
      const opt = (key) => {
        const raw = limEl.attrs[key];
        if (raw === undefined) return null;
        const v = num(raw, NaN, key, lp, limEl.line);
        return Number.isNaN(v) ? null : v;
      };
      limit = {
        lower: opt("lower"),
        upper: opt("upper"),
        effort: opt("effort"),
        velocity: opt("velocity"),
        path: lp,
      };
      if (!v12) {
        const missing = ["effort", "velocity"].filter((k) => limEl.attrs[k] === undefined);
        if (missing.length > 0)
          report(
            "limit-attr-missing",
            `limit of joint '${name}' has no ${missing.join(" and no ")} attribute`,
            lp,
            limEl.line,
          );
      }
    }

    const dynEl = first(el, "dynamics");
    const dynamics = dynEl
      ? {
          damping: num(dynEl.attrs.damping, 0, "damping", `${path}/dynamics`, dynEl.line),
          friction: num(dynEl.attrs.friction, 0, "friction", `${path}/dynamics`, dynEl.line),
        }
      : null;

    const mimEl = first(el, "mimic");
    let mimic = null;
    if (mimEl) {
      const mp = `${path}/mimic`;
      const j = required(mimEl, "joint", mp);
      mimic = {
        joint: j ?? "",
        multiplier: num(mimEl.attrs.multiplier, 1, "multiplier", mp, mimEl.line),
        offset: num(mimEl.attrs.offset, 0, "offset", mp, mimEl.line),
      };
    }

    return {
      name,
      type: typeRaw ?? "",
      parent,
      child,
      origin: origin(first(el, "origin"), path),
      axis,
      axisGiven: axisEl !== undefined,
      limit,
      dynamics,
      mimic,
      path,
      line: el.line,
    };
  }

  /**
   * @param {XmlElement} el
   * @param {string} path
   * @returns {Ros2Control}
   */
  function readRos2Control(el, path) {
    const hw = first(el, "hardware");
    const plugin = hw ? (first(hw, "plugin")?.text.trim() ?? "") : "";
    return {
      name: el.attrs.name ?? "",
      type: el.attrs.type ?? "",
      plugin,
      joints: el.children
        .filter((c) => c.name === "joint")
        .map((c) => ({
          name: c.attrs.name ?? "",
          command: c.children
            .filter((x) => x.name === "command_interface")
            .map((x) => x.attrs.name ?? ""),
          state: c.children
            .filter((x) => x.name === "state_interface")
            .map((x) => x.attrs.name ?? ""),
          path: `${path}/joint[@name='${q(c.attrs.name ?? "")}']`,
          line: c.line,
        })),
      sensors: el.children.filter((c) => c.name === "sensor").map((c) => c.attrs.name ?? ""),
      path,
      line: el.line,
    };
  }

  /**
   * @param {XmlElement} el
   * @param {string} path
   * @returns {GazeboTag}
   */
  function readGazebo(el, path) {
    /** @type {{ name: string, filename: string }[]} */
    const plugins = [];
    /** @param {XmlElement} e */
    const collect = (e) => {
      for (const c of e.children) {
        if (c.name === "plugin")
          plugins.push({ name: c.attrs.name ?? "", filename: c.attrs.filename ?? "" });
        else collect(c);
      }
    };
    collect(el);
    return {
      reference: el.attrs.reference ?? "",
      elements: el.children.map((c) => c.name),
      plugins,
      path: el.attrs.reference ? `/robot/gazebo[@reference='${q(el.attrs.reference)}']` : path,
      line: el.line,
    };
  }
}
