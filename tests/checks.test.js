// Every check has a minimal model that triggers it and a minimal model that
// does not. The table must cover every identifier in CHECKS.

import assert from "node:assert/strict";
import { test } from "node:test";
import { CHECKS } from "../src/core/findings.js";
import { inspectText } from "../src/core/inspect.js";
import {
  boxInertial,
  emptyLink,
  ids,
  joint,
  LIMIT,
  MECANUM,
  robot,
  solidLink,
  twoLink,
  wheeled,
} from "./helpers.js";

const BASE = solidLink("base_link");
/** @param {string} extra */
const withArm = (extra) => robot(BASE + solidLink("arm") + extra);
/** @param {string} type @param {string} extra */
const armJoint = (type, extra) => withArm(joint("j", type, "base_link", "arm", extra));
/** @param {string} inner */
const linkWith = (inner) => robot(`<link name="base_link">${inner}</link>`);
const BOX_SHAPES =
  '<visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>';
/** @param {string} inertia */
const armInertia = (inertia) =>
  robot(
    BASE +
      `<link name="arm"><inertial><mass value="1"/><inertia ${inertia}/></inertial>${BOX_SHAPES}</link>` +
      joint("j", "revolute", "base_link", "arm", `<axis xyz="0 0 1"/>${LIMIT}`),
  );
const YAWED_BOX =
  '<origin rpy="0 0 1.5707963267948966"/><geometry><box size="0.4 0.2 0.1"/></geometry>';
/** @param {string} inertia */
const yawedBox = (inertia) =>
  robot(
    BASE +
      `<link name="arm"><inertial><mass value="2"/><inertia ${inertia}/></inertial>` +
      `<visual>${YAWED_BOX}</visual><collision>${YAWED_BOX}</collision></link>` +
      joint("j", "revolute", "base_link", "arm", `<axis xyz="0 0 1"/>${LIMIT}`),
  );
/** @param {string} geometry */
const meshLink = (geometry) =>
  linkWith(
    `<visual><geometry>${geometry}</geometry></visual><collision><geometry>${geometry}</geometry></collision>`,
  );
/** @param {string} body */
const ros2 = (body) =>
  `<ros2_control name="S" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>${body}</ros2_control>`;
const camera = (/** @type {string} */ rpy) =>
  robot(
    BASE +
      emptyLink("camera_link") +
      emptyLink("camera_optical_frame") +
      joint("cj", "fixed", "base_link", "camera_link", '<origin xyz="0.1 0 0"/>') +
      joint("oj", "fixed", "camera_link", "camera_optical_frame", `<origin rpy="${rpy}"/>`),
  );
const OPTICAL_RPY = "-1.5707963267948966 0 -1.5707963267948966";

/** @type {Record<string, { fail: string, pass: string }>} */
const CASES = {
  "xml-syntax": { fail: '<robot name="t"><link name="a"></robot>', pass: robot(BASE) },
  "not-urdf": { fail: '<sdf version="1.6"><model name="m"/></sdf>', pass: robot(BASE) },
  xacro: {
    fail: robot(
      '<xacro:property name="r" value="0.1"/>' + BASE,
      'name="t" xmlns:xacro="http://www.ros.org/wiki/xacro"',
    ),
    pass: robot(BASE, 'name="t" xmlns:xacro="http://www.ros.org/wiki/xacro"'),
  },
  "version-unsupported": {
    fail: robot(BASE, 'name="t" version="2.0"'),
    pass: robot(BASE, 'name="t" version="1.0"'),
  },
  "missing-attribute": { fail: robot("<link/>" + BASE), pass: robot(BASE) },
  "bad-number": {
    fail: armJoint("revolute", `<origin xyz="0 0"/><axis xyz="0 0 1"/>${LIMIT}`),
    pass: armJoint("revolute", `<origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>${LIMIT}`),
  },
  "unknown-joint-type": {
    fail: armJoint("hinge", `<axis xyz="0 0 1"/>${LIMIT}`),
    pass: armJoint("revolute", `<axis xyz="0 0 1"/>${LIMIT}`),
  },
  "unknown-geometry": {
    fail: linkWith('<visual><geometry><capsule radius="0.1" length="0.2"/></geometry></visual>'),
    pass: linkWith('<visual><geometry><sphere radius="0.1"/></geometry></visual>'),
  },
  "missing-geometry": {
    fail: linkWith("<visual><origin/></visual>"),
    pass: linkWith('<visual><geometry><sphere radius="0.1"/></geometry></visual>'),
  },
  "unknown-element": { fail: robot(BASE + '<sensor name="x"/>'), pass: robot(BASE) },
  "undefined-material": {
    fail: linkWith(
      '<visual><geometry><sphere radius="0.1"/></geometry><material name="nope"/></visual>',
    ),
    pass: robot(
      '<material name="red"><color rgba="1 0 0 1"/></material>' +
        '<link name="base_link"><visual><geometry><sphere radius="0.1"/></geometry><material name="red"/></visual></link>',
    ),
  },
  "duplicate-material": {
    fail: robot('<material name="a"/><material name="a"/>' + BASE),
    pass: robot('<material name="a"/><material name="b"/>' + BASE),
  },
  "no-links": { fail: robot(""), pass: robot(BASE) },
  "duplicate-link": { fail: robot(BASE + BASE), pass: robot(BASE) },
  "duplicate-joint": {
    fail: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j", "fixed", "base_link", "a") +
        joint("j", "fixed", "base_link", "b"),
    ),
    pass: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j1", "fixed", "base_link", "a") +
        joint("j2", "fixed", "base_link", "b"),
    ),
  },
  "missing-link": {
    fail: robot(BASE + joint("j", "fixed", "base_link", "armm")),
    pass: withArm(joint("j", "fixed", "base_link", "arm")),
  },
  "self-joint": {
    fail: robot(BASE + joint("j", "fixed", "base_link", "base_link")),
    pass: withArm(joint("j", "fixed", "base_link", "arm")),
  },
  "multiple-parents": {
    fail: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j1", "fixed", "base_link", "b") +
        joint("j2", "fixed", "a", "b"),
    ),
    pass: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j1", "fixed", "base_link", "b") +
        joint("j2", "fixed", "base_link", "a"),
    ),
  },
  cycle: {
    fail: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j1", "fixed", "a", "b") +
        joint("j2", "fixed", "b", "a"),
    ),
    pass: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j1", "fixed", "base_link", "a") +
        joint("j2", "fixed", "a", "b"),
    ),
  },
  "orphan-link": { fail: withArm(""), pass: withArm(joint("j", "fixed", "base_link", "arm")) },
  "no-base-link": { fail: robot(solidLink("chassis")), pass: robot(BASE) },
  "footprint-relation": {
    fail: robot(
      emptyLink("base_footprint") +
        BASE +
        joint(
          "bj",
          "fixed",
          "base_footprint",
          "base_link",
          '<origin xyz="0 0 0.05" rpy="0.2 0 0"/>',
        ),
    ),
    pass: robot(
      emptyLink("base_footprint") +
        BASE +
        joint(
          "bj",
          "fixed",
          "base_footprint",
          "base_link",
          '<origin xyz="0 0 0.05" rpy="0 0 0.3"/>',
        ),
    ),
  },
  "footprint-height": { fail: wheeled(MECANUM, { z: 0.08 }), pass: wheeled(MECANUM, { z: 0.05 }) },
  "odom-in-urdf": {
    fail: robot(emptyLink("odom") + BASE + joint("oj", "fixed", "odom", "base_link")),
    pass: robot(BASE),
  },
  "root-inertia": {
    fail: robot(BASE),
    pass: robot(
      emptyLink("base_footprint") + BASE + joint("bj", "fixed", "base_footprint", "base_link"),
    ),
  },
  "kdl-unsupported-joint": {
    fail: armJoint("floating", ""),
    pass: armJoint("fixed", ""),
  },
  "optical-frame": { fail: camera("0 0 0"), pass: camera(OPTICAL_RPY) },
  "camera-no-optical": {
    fail: robot(BASE + emptyLink("camera_link") + joint("cj", "fixed", "base_link", "camera_link")),
    pass: camera(OPTICAL_RPY),
  },
  "axis-zero": {
    fail: armJoint("revolute", `<axis xyz="0 0 0"/>${LIMIT}`),
    pass: armJoint("revolute", `<axis xyz="0 0 1"/>${LIMIT}`),
  },
  "axis-not-unit": {
    fail: armJoint("prismatic", `<axis xyz="0 0 2"/>${LIMIT}`),
    pass: armJoint("prismatic", `<axis xyz="0 0 1"/>${LIMIT}`),
  },
  "axis-ignored": {
    fail: armJoint("fixed", '<axis xyz="0 0 1"/>'),
    pass: armJoint("fixed", ""),
  },
  "limit-missing": {
    fail: armJoint("revolute", '<axis xyz="0 0 1"/>'),
    pass: armJoint("revolute", `<axis xyz="0 0 1"/>${LIMIT}`),
  },
  "limit-attr-missing": {
    fail: armJoint("revolute", '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/>'),
    pass: robot(
      BASE +
        solidLink("arm") +
        joint(
          "j",
          "revolute",
          "base_link",
          "arm",
          '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/>',
        ),
      'name="t" version="1.2"',
    ),
  },
  "limit-order": {
    fail: armJoint("revolute", '<limit lower="1" upper="-1" effort="1" velocity="1"/>'),
    pass: armJoint("revolute", LIMIT),
  },
  "limit-equal": {
    fail: armJoint("prismatic", '<limit effort="1" velocity="1"/>'),
    pass: armJoint("prismatic", '<limit lower="0" upper="0.2" effort="1" velocity="1"/>'),
  },
  "limit-zero": {
    fail: armJoint("revolute", '<limit lower="-1" upper="1" effort="0" velocity="1"/>'),
    pass: armJoint("revolute", LIMIT),
  },
  "limit-negative": {
    fail: armJoint("revolute", '<limit lower="-1" upper="1" effort="1" velocity="-1"/>'),
    pass: armJoint("revolute", LIMIT),
  },
  "limit-degrees": {
    fail: armJoint("revolute", '<limit lower="-90" upper="90" effort="1" velocity="1"/>'),
    pass: armJoint("revolute", '<limit lower="-1.5708" upper="1.5708" effort="1" velocity="1"/>'),
  },
  "continuous-limits": {
    fail: armJoint("continuous", '<limit lower="-3" upper="3" effort="1" velocity="1"/>'),
    pass: armJoint("continuous", '<limit effort="1" velocity="1"/>'),
  },
  "full-range-revolute": {
    fail: armJoint("revolute", '<limit lower="-3.1416" upper="3.1416" effort="1" velocity="1"/>'),
    pass: armJoint("revolute", LIMIT),
  },
  "mimic-unknown": {
    fail: armJoint("revolute", `${LIMIT}<mimic joint="nope"/>`),
    pass: armJoint("revolute", LIMIT),
  },
  "mimic-self": {
    fail: armJoint("revolute", `${LIMIT}<mimic joint="j"/>`),
    pass: armJoint("revolute", LIMIT),
  },
  "mimic-chain": {
    fail: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        solidLink("c") +
        joint("j1", "revolute", "base_link", "a", LIMIT) +
        joint("j2", "revolute", "a", "b", `${LIMIT}<mimic joint="j1"/>`) +
        joint("j3", "revolute", "b", "c", `${LIMIT}<mimic joint="j2"/>`),
    ),
    pass: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        solidLink("c") +
        joint("j1", "revolute", "base_link", "a", LIMIT) +
        joint("j2", "revolute", "a", "b", `${LIMIT}<mimic joint="j1"/>`) +
        joint("j3", "revolute", "b", "c", `${LIMIT}<mimic joint="j1" multiplier="-1"/>`),
    ),
  },
  "mimic-zero": {
    fail: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j1", "revolute", "base_link", "a", LIMIT) +
        joint("j2", "revolute", "a", "b", `${LIMIT}<mimic joint="j1" multiplier="0"/>`),
    ),
    pass: robot(
      BASE +
        solidLink("a") +
        solidLink("b") +
        joint("j1", "revolute", "base_link", "a", LIMIT) +
        joint(
          "j2",
          "revolute",
          "a",
          "b",
          `${LIMIT}<mimic joint="j1" multiplier="0.5" offset="0.1"/>`,
        ),
    ),
  },
  "dynamics-negative": {
    fail: armJoint("revolute", `${LIMIT}<dynamics damping="-0.1"/>`),
    pass: armJoint("revolute", `${LIMIT}<dynamics damping="0.1" friction="0.01"/>`),
  },
  "inertial-missing": {
    fail: robot(
      BASE +
        `<link name="arm">${BOX_SHAPES}</link>` +
        joint("j", "revolute", "base_link", "arm", LIMIT),
    ),
    pass: twoLink(),
  },
  "mass-nonpositive": {
    fail: robot(
      BASE +
        `<link name="arm"><inertial><mass value="0"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>` +
        joint("j", "fixed", "base_link", "arm"),
    ),
    pass: twoLink(),
  },
  "inertia-not-pd": {
    fail: armInertia('ixx="0.001" ixy="0.002" ixz="0" iyy="0.001" iyz="0" izz="0.001"'),
    pass: armInertia('ixx="0.0016667" ixy="0" ixz="0" iyy="0.0016667" iyz="0" izz="0.0016667"'),
  },
  "inertia-triangle": {
    fail: armInertia('ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.003"'),
    pass: armInertia('ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"'),
  },
  "inertia-implausible": {
    fail: armInertia('ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"'),
    pass: armInertia('ixx="0.0016667" ixy="0" ixz="0" iyy="0.0016667" iyz="0" izz="0.0016667"'),
  },
  "inertia-axes": {
    // A 0.4 x 0.2 x 0.1 box yawed 90 deg in its shapes; the tensor must follow it.
    fail: yawedBox('ixx="0.0083333" ixy="0" ixz="0" iyy="0.028333" iyz="0" izz="0.033333"'),
    pass: yawedBox('ixx="0.028333" ixy="0" ixz="0" iyy="0.0083333" iyz="0" izz="0.033333"'),
  },
  "density-implausible": {
    fail: robot(
      BASE +
        `<link name="arm">${boxInertial(1000, [0.1, 0.1, 0.1])}${BOX_SHAPES}</link>` +
        joint("j", "revolute", "base_link", "arm", LIMIT),
    ),
    pass: twoLink(),
  },
  "gazebo-dropped": {
    fail: robot(BASE + emptyLink("arm") + joint("j", "revolute", "base_link", "arm", LIMIT)),
    pass: robot(BASE + emptyLink("arm") + joint("j", "fixed", "base_link", "arm")),
  },
  "units-mm": {
    fail: linkWith('<collision><geometry><box size="300 200 50"/></geometry></collision>'),
    pass: linkWith('<collision><geometry><box size="0.3 0.2 0.05"/></geometry></collision>'),
  },
  "rpy-degrees": {
    fail: armJoint("fixed", '<origin rpy="0 0 90"/>'),
    pass: armJoint("fixed", '<origin rpy="0 0 1.5708"/>'),
  },
  "geometry-nonpositive": {
    fail: linkWith(
      '<collision><geometry><cylinder radius="0" length="0.1"/></geometry></collision>',
    ),
    pass: linkWith(
      '<collision><geometry><cylinder radius="0.05" length="0.1"/></geometry></collision>',
    ),
  },
  "collision-missing": {
    fail: linkWith('<visual><geometry><sphere radius="0.1"/></geometry></visual>'),
    pass: linkWith(
      '<visual><geometry><sphere radius="0.1"/></geometry></visual><collision><geometry><sphere radius="0.1"/></geometry></collision>',
    ),
  },
  "mesh-scheme": {
    fail: meshLink('<mesh filename="meshes/base.stl"/>'),
    pass: meshLink('<mesh filename="package://my_robot/meshes/base.stl"/>'),
  },
  "mesh-format": {
    fail: meshLink('<mesh filename="package://my_robot/meshes/base.step"/>'),
    pass: meshLink('<mesh filename="package://my_robot/meshes/base.dae"/>'),
  },
  "mesh-scale": {
    fail: meshLink('<mesh filename="package://r/m.stl" scale="0.001 0 0.001"/>'),
    pass: meshLink('<mesh filename="package://r/m.stl" scale="0.001 0.001 0.001"/>'),
  },
  "ros2-control-joint-unknown": {
    fail: twoLink().replace(
      "</robot>",
      ros2('<joint name="jj"><state_interface name="position"/></joint>') + "</robot>",
    ),
    pass: twoLink().replace(
      "</robot>",
      ros2('<joint name="j"><state_interface name="position"/></joint>') + "</robot>",
    ),
  },
  "ros2-control-no-interface": {
    fail: twoLink().replace(
      "</robot>",
      ros2('<joint name="j"><command_interface name="position"/></joint>') + "</robot>",
    ),
    pass: twoLink().replace(
      "</robot>",
      ros2(
        '<joint name="j"><command_interface name="position"/><state_interface name="position"/></joint>',
      ) + "</robot>",
    ),
  },
  "ros2-control-summary": {
    fail: twoLink().replace(
      "</robot>",
      ros2('<joint name="j"><state_interface name="position"/></joint>') + "</robot>",
    ),
    pass: twoLink(),
  },
  "gazebo-reference-unknown": {
    fail: robot(BASE + '<gazebo reference="base_lnk"><material>Gazebo/Grey</material></gazebo>'),
    pass: robot(BASE + '<gazebo reference="base_link"><material>Gazebo/Grey</material></gazebo>'),
  },
  "gazebo-summary": {
    fail: robot(BASE + '<gazebo><plugin name="p" filename="libp.so"/></gazebo>'),
    pass: robot(BASE),
  },
  "transmission-joint-unknown": {
    fail: twoLink().replace(
      "</robot>",
      '<transmission name="t1"><type>transmission_interface/SimpleTransmission</type><joint name="jx"/></transmission></robot>',
    ),
    pass: twoLink().replace(
      "</robot>",
      '<transmission name="t1"><type>transmission_interface/SimpleTransmission</type><joint name="j"/></transmission></robot>',
    ),
  },
  "wheel-axis": {
    fail: wheeled(MECANUM.map((w, k) => (k === 0 ? { ...w, axis: "1 0 0" } : w))),
    pass: wheeled(MECANUM),
  },
  "wheel-cylinder-axis": {
    fail: wheeled(MECANUM.map((w, k) => (k === 2 ? { ...w, cylRpy: "0 0 0" } : w))),
    pass: wheeled(MECANUM),
  },
  "wheel-direction-mixed": {
    fail: wheeled(MECANUM.map((w) => (w.name.includes("right") ? { ...w, axis: "0 -1 0" } : w))),
    pass: wheeled(MECANUM),
  },
  "wheel-radius-mismatch": {
    fail: wheeled(MECANUM.map((w, k) => (k === 3 ? { ...w, r: 0.055 } : w))),
    pass: wheeled(MECANUM),
  },
  "wheel-layout": {
    fail: wheeled(MECANUM.map((w, k) => (k === 1 ? { ...w, xyz: "0.13 -0.12 0" } : w))),
    pass: wheeled(MECANUM),
  },
  "wheel-base-summary": { fail: wheeled(MECANUM), pass: twoLink() },
};

test("the table covers every check in the catalogue", () => {
  assert.deepEqual(Object.keys(CASES).sort(), Object.keys(CHECKS).sort());
});

for (const [id, c] of Object.entries(CASES)) {
  test(`${id}: reported on the failing model, absent from the passing one`, () => {
    const failIds = ids(c.fail);
    assert.ok(failIds.includes(id), `${id} missing; got ${failIds.join(", ")}`);
    const passIds = ids(c.pass);
    assert.ok(!passIds.includes(id), `${id} reported on the passing model`);
    const f = inspectText(c.fail).findings.find((x) => x.id === id);
    assert.ok(f && f.message.length > 0 && f.path.startsWith("/"));
    // Catalogue severity is the most severe level a check uses.
    const rank = { error: 0, warning: 1, info: 2 };
    assert.ok(rank[f.severity] >= rank[CHECKS[id].severity]);
  });
}

test("the passing base fixtures are completely clean", () => {
  const arm = robot(
    emptyLink("world") +
      solidLink("base_link") +
      solidLink("arm") +
      joint("w", "fixed", "world", "base_link") +
      joint("j", "revolute", "base_link", "arm", `<axis xyz="0 0 1"/>${LIMIT}`),
  );
  for (const text of [arm, wheeled(MECANUM)]) {
    const r = inspectText(text);
    const bad = r.findings.filter((f) => f.severity !== "info");
    assert.deepEqual(bad, [], JSON.stringify(bad, null, 1));
  }
});

test("footprint-relation also fires for a movable joint between base_footprint and base_link", () => {
  const text = robot(
    emptyLink("base_footprint") +
      BASE +
      joint("bj", "revolute", "base_footprint", "base_link", `<axis xyz="0 0 1"/>${LIMIT}`),
  );
  const f = inspectText(text).findings.find((x) => x.id === "footprint-relation");
  assert.match(f?.message ?? "", /non-fixed joint\(s\) 'bj'/);
});

test("base_footprint may also be the child of base_link", () => {
  const text = robot(
    BASE +
      emptyLink("base_footprint") +
      joint("fj", "fixed", "base_link", "base_footprint", '<origin xyz="0 0 -0.05"/>'),
  );
  assert.ok(!ids(text).includes("footprint-relation"));
});

test("gazebo-dropped respects preserveFixedJoint and counts dropped descendants", () => {
  const preserved = robot(
    BASE +
      emptyLink("arm") +
      joint("j", "fixed", "base_link", "arm") +
      '<gazebo reference="j"><preserveFixedJoint>true</preserveFixedJoint></gazebo>',
  );
  assert.ok(ids(preserved).includes("gazebo-dropped"));
  const chain = robot(
    BASE +
      emptyLink("a") +
      solidLink("b") +
      solidLink("c") +
      joint("ja", "revolute", "base_link", "a", LIMIT) +
      joint("jb", "revolute", "a", "b", LIMIT) +
      joint("jc", "revolute", "b", "c", LIMIT),
  );
  const f = inspectText(chain).findings.find((x) => x.id === "gazebo-dropped");
  assert.match(f?.message ?? "", /1 child joint\(s\) and 2 descendant link\(s\)/);
  // A massless root whose fixed child has mass is lumped, not dropped.
  const lumped = robot(
    emptyLink("base_footprint") + BASE + joint("bj", "fixed", "base_footprint", "base_link"),
  );
  assert.ok(!ids(lumped).includes("gazebo-dropped"));
  // A link named world is Gazebo's fixed anchor and never dropped.
  const world = robot(emptyLink("world") + BASE + joint("wj", "fixed", "world", "base_link"));
  assert.ok(!ids(world).includes("gazebo-dropped"));
});

test("inertia-implausible gives the reference tensor in the inertial frame", () => {
  // A 0.4 x 0.2 x 0.1 box rotated 90 deg about z in its visual, inertial unrotated.
  const text = robot(
    `<link name="base_link"><inertial><mass value="2"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>` +
      `<collision><origin rpy="0 0 1.5707963267948966"/><geometry><box size="0.4 0.2 0.1"/></geometry></collision></link>`,
  );
  const f = inspectText(text).findings.find((x) => x.id === "inertia-implausible");
  const ref = /** @type {Record<string, number>} */ (f?.data?.reference);
  // In the box frame: ixx = 2(0.04+0.01)/12, iyy = 2(0.16+0.01)/12, izz = 2(0.16+0.04)/12.
  // Rotated 90 deg about z, x and y swap.
  assert.ok(Math.abs(ref.ixx - (2 * 0.17) / 12) < 1e-12);
  assert.ok(Math.abs(ref.iyy - (2 * 0.05) / 12) < 1e-12);
  assert.ok(Math.abs(ref.izz - (2 * 0.2) / 12) < 1e-12);
  assert.equal(ref.ixy, 0);
});

test("mesh scheme variants", () => {
  /** @param {string} f */
  const msg = (f) =>
    inspectText(meshLink(`<mesh filename="${f}"/>`)).findings.find((x) => x.id === "mesh-scheme")
      ?.message ?? "";
  assert.match(msg("https://example.com/a.stl"), /network/);
  assert.match(msg("model://robot/a.stl"), /only Gazebo/);
  assert.match(msg("/home/me/a.stl"), /absolute path/);
  assert.match(msg("C:\\robots\\a.stl"), /absolute path/);
  assert.match(msg("s3://bucket/a.stl"), /unsupported scheme s3/);
  assert.equal(msg("file:///home/me/a.stl"), "");
  const neg = inspectText(
    meshLink('<mesh filename="package://r/m.stl" scale="-1 1 1"/>'),
  ).findings.find((x) => x.id === "mesh-scale");
  assert.equal(neg?.severity, "warning");
  const noext = inspectText(meshLink('<mesh filename="package://r/mesh"/>')).findings.find(
    (x) => x.id === "mesh-format",
  );
  assert.match(noext?.message ?? "", /without an extension/);
});

test("ros2_control joint without any interface", () => {
  const text = twoLink().replace("</robot>", ros2('<joint name="j"/>') + "</robot>");
  const f = inspectText(text).findings.find((x) => x.id === "ros2-control-no-interface");
  assert.match(f?.message ?? "", /no command or state interfaces/);
});

test("differential layout problems are reported", () => {
  const off = wheeled([
    { name: "left_wheel", xyz: "0 0.12 0" },
    { name: "right_wheel", xyz: "0.05 -0.10 0.01" },
  ]);
  const f = inspectText(off).findings.find((x) => x.id === "wheel-layout");
  assert.match(f?.message ?? "", /not on one axle/);
  assert.match(f?.message ?? "", /not centred/);
  assert.match(f?.message ?? "", /heights differ/);
});

test("four wheels not one per corner", () => {
  const text = wheeled([
    { name: "a_wheel", xyz: "0.1 0.12 0" },
    { name: "b_wheel", xyz: "0.1 0.10 0" },
    { name: "c_wheel", xyz: "-0.1 0.12 0" },
    { name: "d_wheel", xyz: "-0.1 -0.3 0" },
  ]);
  const f = inspectText(text).findings.find((x) => x.id === "wheel-layout");
  assert.match(f?.message ?? "", /not one per corner/);
});
