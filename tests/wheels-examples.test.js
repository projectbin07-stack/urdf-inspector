// Wheel measurement, mecanum-kinematics export and the bundled examples.

import assert from "node:assert/strict";
import { test } from "node:test";
import { inspectText } from "../src/core/inspect.js";
import { chainTo } from "../src/core/tree.js";
import { mecanumToml } from "../src/core/wheels.js";
import { example, MECANUM, wheeled } from "./helpers.js";

test("four-wheel base: labels, radius, lx and ly", () => {
  const ctx = inspectText(wheeled(MECANUM)).context;
  const wb = ctx?.wheels;
  assert.ok(wb);
  assert.equal(wb.kind, "four-wheel");
  assert.equal(wb.baseFrame, "base_link");
  assert.deepEqual(
    wb.wheels.map((w) => `${w.label}:${w.joint}`),
    [
      "FL:front_left_wheel_joint",
      "FR:front_right_wheel_joint",
      "RL:rear_left_wheel_joint",
      "RR:rear_right_wheel_joint",
    ],
  );
  assert.equal(wb.radius, 0.05);
  assert.ok(Math.abs(/** @type {number} */ (wb.lx) - 0.1) < 1e-12);
  assert.ok(Math.abs(/** @type {number} */ (wb.ly) - 0.12) < 1e-12);
});

test("mecanum TOML export matches the mecanum-kinematics config fields", () => {
  const ctx = inspectText(example("mecanum_base.urdf")).context;
  const res = mecanumToml(ctx?.wheels ?? null, "example_mecanum_base");
  assert.ok(res.ok);
  const toml = res.ok ? res.toml : "";
  // Only keys that mecanum-kinematics' RobotConfig accepts may appear.
  const keys = toml
    .split("\n")
    .filter((l) => /^[a-z_]+ =/.test(l))
    .map((l) => l.split(" ")[0]);
  assert.deepEqual(keys, [
    "name",
    "description",
    "wheel_radius",
    "half_length",
    "half_width",
    "roller_angle_deg",
    "max_wheel_speed",
    "wheel_order",
  ]);
  assert.match(toml, /^wheel_radius = 0\.04 /m);
  assert.match(toml, /^half_length = 0\.1 /m);
  assert.match(toml, /^half_width = 0\.13 /m);
  assert.match(toml, /^max_wheel_speed = 20 /m);
  assert.match(toml, /^wheel_order = \["FL", "FR", "RL", "RR"\]$/m);
  assert.match(toml, /FL: front_left_wheel_joint \(\+y axis\)/);
});

test("export notes flipped axes and an offset wheel centroid", () => {
  const shifted = MECANUM.map((w) => {
    const [x, y, z] = w.xyz.split(" ").map(Number);
    return {
      ...w,
      xyz: `${x + 0.02} ${y} ${z}`,
      axis: w.name.includes("right") ? "0 -1 0" : "0 1 0",
    };
  });
  const ctx = inspectText(wheeled(shifted)).context;
  const res = mecanumToml(ctx?.wheels ?? null, "my robot");
  assert.ok(res.ok);
  const toml = res.ok ? res.toml : "";
  assert.match(toml, /^name = "my-robot"$/m);
  assert.match(toml, /FR: front_right_wheel_joint \(-y axis: negate its commands/);
  assert.match(toml, /wheel centroid is at x=0\.02, y=0 m in base_link/);
  // The wheel velocity limits are all 10 rad/s.
  assert.match(toml, /^max_wheel_speed = 10 /m);
});

test("export refuses what it cannot measure", () => {
  /** @param {string} text */
  const reason = (text) => {
    const ctx = inspectText(text).context;
    const r = mecanumToml(ctx?.wheels ?? null, "x");
    return r.ok ? "" : r.reason;
  };
  assert.match(reason(example("arm_2dof.urdf")), /no wheel joints/);
  assert.match(reason(example("diff_drive_base.urdf")), /found 2 wheel\(s\)/);
  assert.match(
    reason(
      wheeled([
        { name: "a_wheel", xyz: "0.1 0.12 0" },
        { name: "b_wheel", xyz: "0.1 0.10 0" },
        { name: "c_wheel", xyz: "-0.1 0.12 0" },
        { name: "d_wheel", xyz: "-0.1 -0.3 0" },
      ]),
    ),
    /not one per corner/,
  );
  const sphereless = wheeled(MECANUM).replace(
    /<visual><origin xyz="0 0 0" rpy="1.5707963267948966 0 0"\/><geometry><cylinder radius="0.05" length="0.03"\/><\/geometry><\/visual><collision>.*?<\/collision><\/link>/,
    "</link>",
  );
  assert.match(reason(sphereless), /not every wheel has a cylinder/);
});

test("differential example: wheel separation and labels", () => {
  const wb = inspectText(example("diff_drive_base.urdf")).context?.wheels;
  assert.equal(wb?.kind, "differential");
  assert.deepEqual(
    wb?.wheels.map((w) => w.label),
    ["L", "R"],
  );
  assert.ok(Math.abs(/** @type {number} */ (wb?.ly) - 0.115) < 1e-12);
});

test("wheels are found by cylinder shape when the name says nothing, casters are not", () => {
  const text = wheeled([
    { name: "drive_a", xyz: "0 0.12 0" },
    { name: "drive_b", xyz: "0 -0.12 0" },
    { name: "caster", xyz: "-0.1 0 0" },
  ]);
  const wb = inspectText(text).context?.wheels;
  assert.deepEqual(
    wb?.wheels.map((w) => w.joint),
    ["drive_a_joint", "drive_b_joint"],
  );
});

test("clean examples have no errors or warnings", () => {
  for (const name of ["mecanum_base.urdf", "arm_2dof.urdf", "diff_drive_base.urdf"]) {
    const r = inspectText(example(name));
    const bad = r.findings.filter((f) => f.severity !== "info").map((f) => `${f.id}: ${f.message}`);
    assert.deepEqual(bad, [], name);
  }
});

test("the broken example reports every mistake its comments describe", () => {
  const r = inspectText(example("broken_robot.urdf"));
  const found = new Set(r.findings.map((f) => f.id));
  for (const id of [
    "odom-in-urdf",
    "inertia-implausible",
    "wheel-cylinder-axis",
    "wheel-direction-mixed",
    "continuous-limits",
    "limit-order",
    "inertia-triangle",
    "mesh-scheme",
    "limit-degrees",
    "limit-zero",
    "inertial-missing",
    "gazebo-dropped",
    "collision-missing",
    "rpy-degrees",
    "optical-frame",
    "limit-missing",
    "axis-zero",
    "mimic-unknown",
    "orphan-link",
    "ros2-control-joint-unknown",
  ])
    assert.ok(found.has(id), id);
  assert.equal(r.summary.errors, 7);
  // Every MISTAKE comment block is reflected: no unexpected check ids.
  assert.deepEqual([...found].sort(), [
    "axis-zero",
    "collision-missing",
    "continuous-limits",
    "gazebo-dropped",
    "inertia-implausible",
    "inertia-triangle",
    "inertial-missing",
    "limit-degrees",
    "limit-missing",
    "limit-order",
    "limit-zero",
    "mesh-scheme",
    "mimic-unknown",
    "odom-in-urdf",
    "optical-frame",
    "orphan-link",
    "ros2-control-joint-unknown",
    "ros2-control-summary",
    "rpy-degrees",
    "wheel-base-summary",
    "wheel-cylinder-axis",
    "wheel-direction-mixed",
  ]);
});

test("inspection is deterministic", () => {
  const text = example("broken_robot.urdf");
  const a = JSON.stringify(inspectText(text).findings);
  const b = JSON.stringify(inspectText(text).findings);
  assert.equal(a, b);
});

test("chainTo lists joints from the root", () => {
  const ctx = inspectText(example("arm_2dof.urdf")).context;
  assert.ok(ctx);
  assert.deepEqual(
    chainTo(ctx.tree, "tool0").map((j) => j.name),
    ["world_joint", "shoulder_joint", "elbow_joint", "tool_joint"],
  );
  assert.deepEqual(chainTo(ctx.tree, "world"), []);
});
