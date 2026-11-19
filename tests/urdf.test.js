import assert from "node:assert/strict";
import { test } from "node:test";
import { inspectText } from "../src/core/inspect.js";
import { detectXacro, readUrdf } from "../src/core/urdf.js";
import { parseXml } from "../src/core/xml.js";
import { robot } from "./helpers.js";

/** @param {string} text */
const read = (text) => readUrdf(parseXml(text));

const FULL = robot(
  `<material name="red"><color rgba="1 0 0 1"/></material>
  <link name="base_link">
    <inertial>
      <origin xyz="0.1 0 0.2" rpy="0 0.1 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.1" ixy="0.01" ixz="0.02" iyy="0.2" iyz="0.03" izz="0.3"/>
    </inertial>
    <visual name="body">
      <origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
      <geometry><box size="0.3 0.2 0.1"/></geometry>
      <material name="red"/>
    </visual>
    <visual>
      <geometry><cylinder radius="0.05" length="0.2"/></geometry>
      <material name="inline"><color rgba="0 1 0 0.5"/></material>
    </visual>
    <collision>
      <geometry><sphere radius="0.4"/></geometry>
    </collision>
    <collision>
      <geometry><mesh filename="package://p/m.stl" scale="0.001 0.001 0.001"/></geometry>
    </collision>
    <collision>
      <geometry><mesh filename="package://p/n.dae"/></geometry>
    </collision>
  </link>
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/><link name="e"/><link name="f"/>
  <joint name="rev" type="revolute">
    <parent link="base_link"/><child link="a"/>
    <origin xyz="0 0 0.1" rpy="0 0 1.5"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="2" effort="3" velocity="4"/>
    <dynamics damping="0.5" friction="0.25"/>
    <safety_controller k_velocity="1"/>
  </joint>
  <joint name="cont" type="continuous"><parent link="base_link"/><child link="b"/><limit effort="1" velocity="2"/></joint>
  <joint name="pri" type="prismatic">
    <parent link="base_link"/><child link="c"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.5" effort="10" velocity="0.1"/>
    <mimic joint="rev" multiplier="0.5" offset="0.1"/>
  </joint>
  <joint name="fix" type="fixed"><parent link="base_link"/><child link="d"/></joint>
  <joint name="flo" type="floating"><parent link="base_link"/><child link="e"/></joint>
  <joint name="pla" type="planar"><parent link="base_link"/><child link="f"/><axis xyz="0 0 1"/><dynamics/><mimic joint="rev"/></joint>
  <transmission name="t"><type>x</type><joint name="rev"/><actuator name="m"/></transmission>
  <ros2_control name="Sys" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin><param name="x">1</param></hardware>
    <joint name="rev"><command_interface name="position"/><state_interface name="position"/><state_interface name="velocity"/></joint>
    <sensor name="imu"/>
  </ros2_control>
  <ros2_control name="Empty" type="actuator"/>
  <gazebo reference="base_link"><material>Gazebo/Red</material><mu1>0.5</mu1></gazebo>
  <gazebo><plugin name="ctrl" filename="libgazebo_ros2_control.so"><parameters>x.yaml</parameters></plugin></gazebo>
  <gazebo reference="a"><sensor name="s" type="camera"><plugin name="cam" filename="libcam.so"/></sensor></gazebo>`,
  'name="full" version="1.0"',
);

test("parses every URDF element type with its values", () => {
  const { robot: r, findings } = read(FULL);
  assert.ok(r);
  assert.deepEqual(findings, []);
  assert.equal(r.name, "full");
  assert.equal(r.version, "1.0");
  assert.equal(r.links.length, 7);
  const base = r.links[0];
  assert.deepEqual(base.inertial?.origin, { xyz: [0.1, 0, 0.2], rpy: [0, 0.1, 0] });
  assert.equal(base.inertial?.mass, 2.5);
  assert.deepEqual(base.inertial?.inertia, {
    ixx: 0.1,
    ixy: 0.01,
    ixz: 0.02,
    iyy: 0.2,
    iyz: 0.03,
    izz: 0.3,
  });
  assert.equal(base.visuals.length, 2);
  assert.equal(base.visuals[0].name, "body");
  assert.deepEqual(base.visuals[0].origin, { xyz: [1, 2, 3], rpy: [0.1, 0.2, 0.3] });
  assert.deepEqual(base.visuals[0].geometry, { type: "box", size: [0.3, 0.2, 0.1] });
  assert.deepEqual(base.visuals[0].material, { name: "red", rgba: null });
  assert.deepEqual(base.visuals[1].geometry, { type: "cylinder", radius: 0.05, length: 0.2 });
  assert.deepEqual(base.visuals[1].material, { name: "inline", rgba: [0, 1, 0, 0.5] });
  assert.equal(base.visuals[1].path, "/robot/link[@name='base_link']/visual[2]");
  assert.deepEqual(base.collisions[0].geometry, { type: "sphere", radius: 0.4 });
  assert.deepEqual(base.collisions[1].geometry, {
    type: "mesh",
    filename: "package://p/m.stl",
    scale: [0.001, 0.001, 0.001],
  });
  assert.deepEqual(base.collisions[2].geometry, {
    type: "mesh",
    filename: "package://p/n.dae",
    scale: [1, 1, 1],
  });
  assert.deepEqual(base.collisions[0].origin, { xyz: [0, 0, 0], rpy: [0, 0, 0] });
  assert.deepEqual(r.materials.get("red")?.rgba, [1, 0, 0, 1]);

  const j = Object.fromEntries(r.joints.map((x) => [x.name, x]));
  assert.deepEqual(
    r.joints.map((x) => x.type),
    ["revolute", "continuous", "prismatic", "fixed", "floating", "planar"],
  );
  assert.equal(j.rev.parent, "base_link");
  assert.equal(j.rev.child, "a");
  assert.deepEqual(j.rev.origin, { xyz: [0, 0, 0.1], rpy: [0, 0, 1.5] });
  assert.deepEqual(j.rev.axis, [0, 0, 1]);
  assert.equal(j.rev.axisGiven, true);
  assert.deepEqual(
    { ...j.rev.limit, path: undefined },
    { lower: -1, upper: 2, effort: 3, velocity: 4, path: undefined },
  );
  assert.deepEqual(j.rev.dynamics, { damping: 0.5, friction: 0.25 });
  // Defaults from the specification.
  assert.deepEqual(j.cont.axis, [1, 0, 0]);
  assert.equal(j.cont.axisGiven, false);
  assert.equal(j.cont.limit?.lower, null);
  assert.deepEqual(j.pri.mimic, { joint: "rev", multiplier: 0.5, offset: 0.1 });
  assert.deepEqual(j.pla.mimic, { joint: "rev", multiplier: 1, offset: 0 });
  assert.deepEqual(j.pla.dynamics, { damping: 0, friction: 0 });
  assert.equal(j.fix.limit, null);

  assert.deepEqual(r.transmissions, [
    { name: "t", joints: ["rev"], path: "/robot/transmission[@name='t']", line: 47 },
  ]);
  assert.equal(r.ros2Control.length, 2);
  assert.equal(r.ros2Control[0].plugin, "mock_components/GenericSystem");
  assert.deepEqual(r.ros2Control[0].joints[0].command, ["position"]);
  assert.deepEqual(r.ros2Control[0].joints[0].state, ["position", "velocity"]);
  assert.deepEqual(r.ros2Control[0].sensors, ["imu"]);
  assert.equal(r.ros2Control[1].plugin, "");
  assert.equal(r.gazebo.length, 3);
  assert.equal(r.gazebo[0].reference, "base_link");
  assert.deepEqual(r.gazebo[0].elements, ["material", "mu1"]);
  assert.deepEqual(r.gazebo[1].plugins, [{ name: "ctrl", filename: "libgazebo_ros2_control.so" }]);
  assert.deepEqual(r.gazebo[2].plugins, [{ name: "cam", filename: "libcam.so" }]);
  assert.equal(r.gazebo[2].path, "/robot/gazebo[@reference='a']");
});

test("missing required attributes and elements are reported", () => {
  const { findings } = read(
    `<robot>
      <link name="a"><inertial><inertia ixx="1"/></inertial></link>
      <link name="b"><inertial><mass/></inertial>
        <visual><geometry><cylinder/></geometry></visual>
        <visual><geometry><sphere/></geometry></visual>
        <visual><geometry><mesh/></geometry></visual>
        <visual><geometry/></visual>
      </link>
      <joint name="j" type="fixed"/>
      <joint type="fixed"><parent/><child/></joint>
      <joint name="m" type="revolute"><parent link="a"/><child link="b"/><mimic/><limit effort="1" velocity="1"/></joint>
      <material/>
    </robot>`,
  );
  const msgs = findings.map((f) => `${f.id}: ${f.message}`);
  for (const expect of [
    "missing-attribute: <robot> has no name attribute",
    "missing-attribute: <inertia> has no ixy attribute",
    "missing-attribute: inertial of 'a' has no <mass>",
    "missing-attribute: <mass> has no value attribute",
    "missing-attribute: inertial of 'b' has no <inertia>",
    "missing-attribute: <cylinder> has no radius attribute",
    "missing-attribute: <cylinder> has no length attribute",
    "missing-attribute: <sphere> has no radius attribute",
    "missing-attribute: <mesh> has no filename attribute",
    "missing-geometry: visual 4 of link 'b' has no shape inside <geometry>",
    "missing-attribute: joint 'j' has no <parent>",
    "missing-attribute: joint 'j' has no <child>",
    "missing-attribute: <joint> has no name attribute",
    "missing-attribute: <parent> has no link attribute",
    "missing-attribute: <mimic> has no joint attribute",
    "missing-attribute: <material> has no name attribute",
  ])
    assert.ok(msgs.includes(expect), `${expect}\n in ${msgs.join("\n")}`);
});

test("malformed numbers are reported and replaced by defaults", () => {
  const { robot: r, findings } = read(
    robot(`<link name="a"><inertial><mass value="1kg"/><inertia ixx="x" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
      <visual><origin xyz="1 2 3 4"/><geometry><box size="a b c"/></geometry><material name="m"><color rgba="1 0 0"/></material></visual></link>
      <link name="b"/>
      <joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0"/>
        <limit lower="" upper="1" effort="1" velocity="1"/><dynamics damping="lots"/><mimic joint="j" multiplier="two"/></joint>
      <material name="g"><color rgba="1"/></material>`),
  );
  const bad = findings.filter((f) => f.id === "bad-number").map((f) => f.message);
  assert.deepEqual(bad.sort(), [
    'axis xyz="0 0" must be three numbers',
    'damping="lots" is not a finite number',
    'ixx="x" is not a finite number',
    'lower="" is not a finite number',
    'mass value="1kg" is not a finite number',
    'multiplier="two" is not a finite number',
    'rgba="1 0 0" must be four numbers',
    'rgba="1" must be four numbers',
    'size="a b c" must be three numbers',
    'xyz="1 2 3 4" must be three numbers',
  ]);
  assert.ok(r);
  assert.deepEqual(r.joints[0].axis, [1, 0, 0]);
  assert.equal(r.links[0].inertial?.mass, null);
  assert.equal(r.links[0].inertial?.inertia, null);
  assert.equal(r.joints[0].limit?.lower, null);
});

test("unknown joint types and missing type", () => {
  const { findings } = read(
    robot(
      '<link name="a"/><link name="b"/><joint name="j"><parent link="a"/><child link="b"/></joint>',
    ),
  );
  assert.equal(findings[0].id, "unknown-joint-type");
  assert.match(findings[0].message, /has no type/);
});

test("xacro detection: elements, ${} expressions and $() substitutions", () => {
  const src = `<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="r" value="0.05"/>
  <xacro:include filename="$(find my_pkg)/urdf/wheel.xacro"/>
  <link name="a"><visual><geometry><cylinder radius="\${r}" length="0.1"/></geometry></visual></link>
  <gazebo><plugin name="p" filename="x"><parameters>$(find pkg)/config.yaml</parameters></plugin></gazebo>
</robot>`;
  const f = detectXacro(parseXml(src));
  assert.deepEqual(
    f.map((x) => x.message),
    [
      "<xacro:property 'r'> is a xacro element",
      "<xacro:include> is a xacro element",
      'filename="$(find my_pkg)/urdf/wheel.xacro" contains a xacro expression',
      'radius="${r}" contains a xacro expression',
      'text "$(find pkg)/config.yaml" contains a xacro expression',
    ],
  );
  assert.equal(f[3].path, "/robot/link[@name='a']/visual[1]/geometry[1]/cylinder[1]/@radius");
  assert.equal(f[3].line, 4);
  // inspectText stops after xacro: no structural findings on unexpanded input.
  const r = inspectText(src);
  assert.equal(r.robot, null);
  assert.ok(r.findings.every((x) => x.id === "xacro"));
});

test("xacro findings are capped with a count of the rest", () => {
  const many = Array.from(
    { length: 30 },
    (_, k) => `<xacro:property name="p${k}" value="1"/>`,
  ).join("");
  const f = detectXacro(parseXml(robot(many)));
  assert.equal(f.length, 26);
  assert.equal(f[25].message, "5 more xacro constructs not listed");
});

test("not-urdf identifies SDF", () => {
  const r = inspectText('<sdf version="1.7"><model name="m"/></sdf>');
  assert.match(r.findings[0].message, /looks like SDF/);
  assert.equal(r.summary.errors, 1);
});

test("element paths quote names with apostrophes", () => {
  const { robot: r } = read(robot(`<link name="it's"/>`));
  assert.equal(r?.links[0].path, "/robot/link[@name='it&apos;s']");
});
