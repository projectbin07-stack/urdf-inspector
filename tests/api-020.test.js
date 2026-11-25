// Library additions in 0.2.0, requested by Robot Studio: joint frames,
// a file lookup for meshes, source files on findings, a location formatter
// for the text report, and a lone file argument on the CLI.

import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import { fileURLToPath } from "node:url";
import { forwardKinematics, jointFrames } from "../src/core/fk.js";
import { inspectText, inspectXml } from "../src/core/inspect.js";
import { checkText, lineLocation } from "../src/core/report.js";
import { buildTree } from "../src/core/tree.js";
import { readUrdf } from "../src/core/urdf.js";
import { parseXml } from "../src/core/xml.js";
import { apply } from "../src/core/math.js";
import { EXIT, main } from "../src/cli.js";
import { joint, LIMIT, robot, solidLink } from "./helpers.js";

/** @param {number} a @param {number} b */
const close = (a, b, eps = 1e-12) => assert.ok(Math.abs(a - b) <= eps, `${a} vs ${b}`);

/** @param {string} text */
const treeOf = (text) => {
  const { robot: r } = readUrdf(parseXml(text));
  return buildTree(/** @type {import("../src/core/urdf.js").Robot} */ (r)).tree;
};

test("jointFrames: parent pose times the joint origin, unaffected by the joint's own position", () => {
  // base -> j1 (revolute about z at x = 1) -> arm -> j2 (fixed, 0.5 m along the arm's x)
  const text = robot(
    solidLink("base_link") +
      solidLink("arm") +
      solidLink("tip") +
      joint(
        "j1",
        "revolute",
        "base_link",
        "arm",
        `<origin xyz="1 0 0"/><axis xyz="0 0 1"/>${LIMIT}`,
      ) +
      joint("j2", "fixed", "arm", "tip", '<origin xyz="0.5 0 0" rpy="0 0 0.3"/>'),
  );
  const tree = treeOf(text);
  const q = { j1: Math.PI / 2 };
  const frames = jointFrames(tree, q);
  const f1 = /** @type {import("../src/core/math.js").Transform} */ (frames.get("j1"));
  // j1's frame is at (1, 0, 0) with the base's orientation, whatever q is.
  assert.deepEqual(f1.p, [1, 0, 0]);
  close(f1.R[0], 1);
  // j2's frame: the arm is turned 90 degrees, so 0.5 m along it is +y.
  const f2 = /** @type {import("../src/core/math.js").Transform} */ (frames.get("j2"));
  close(f2.p[0], 1);
  close(f2.p[1], 0.5);
  // The child link pose of a fixed joint is its joint frame.
  const tip = /** @type {import("../src/core/math.js").Transform} */ (
    forwardKinematics(tree, q).get("tip")
  );
  const a = apply(tip, [0.1, 0, 0]);
  const b = apply(f2, [0.1, 0, 0]);
  for (let k = 0; k < 3; k++) close(a[k], b[k]);
  // Precomputed poses give the same frames.
  assert.deepEqual(jointFrames(tree, q, forwardKinematics(tree, q)), frames);
  assert.equal(frames.size, 2);
});

const MESHES = robot(
  `<link name="base_link"><visual><geometry><mesh filename="package://r/base.stl"/></geometry></visual>` +
    `<collision><geometry><mesh filename="package://r/gone.stl"/></geometry></collision></link>`,
);

test("meshExists: each missing mesh is a warning with the caller's reason; no lookup without it", () => {
  /** @type {string[]} */
  const asked = [];
  const r = inspectText(MESHES, {
    meshExists: (f) => {
      asked.push(f);
      return f.endsWith("base.stl") || "not in the opened folder";
    },
  });
  assert.deepEqual(asked, ["package://r/base.stl", "package://r/gone.stl"]);
  const missing = r.findings.filter((f) => f.id === "mesh-missing");
  assert.equal(missing.length, 1);
  assert.equal(
    missing[0].message,
    "collision mesh 'package://r/gone.stl' of link 'base_link' was not found: not in the opened folder",
  );
  assert.equal(missing[0].severity, "warning");
  assert.deepEqual(missing[0].target, { kind: "link", name: "base_link" });
  assert.ok(!inspectText(MESHES).findings.some((f) => f.id === "mesh-missing"));
});

test("source: findings carry the file of their top-level element when the input elements have one", () => {
  const root = parseXml(
    robot(
      solidLink("base_link") +
        solidLink("arm") +
        joint("j", "revolute", "base_link", "arm", '<axis xyz="0 0 0"/>' + LIMIT),
    ),
  );
  // As a xacro expander would: the joint came from arm.xacro, the rest from robot.xacro.
  root.source = "robot.xacro";
  for (const c of root.children) if (c.name === "joint") c.source = "arm.xacro";
  const r = inspectXml(root);
  const axis = r.findings.find((f) => f.id === "axis-zero");
  assert.equal(axis?.source, "arm.xacro");
  assert.ok(
    r.findings.filter((f) => f.target.kind !== "joint").every((f) => f.source === "robot.xacro"),
  );
  // Without sources nothing is added.
  assert.ok(inspectText(robot(solidLink("base_link"))).findings.every((f) => !("source" in f)));
});

test("checkText: a custom location formatter, the default unchanged", () => {
  const r = inspectText(MESHES, { meshExists: () => "absent" });
  const def = checkText(r, "m.urdf");
  assert.equal(def, checkText(r, "m.urdf", { location: lineLocation }));
  assert.match(def, /WARN {2}mesh-missing {2}\(line \d+\)/);
  const custom = checkText(r, "m.urdf", { location: (f) => ` m.urdf:${f.line}` });
  assert.match(custom, /WARN {2}mesh-missing m\.urdf:\d+/);
  assert.equal(lineLocation({ ...r.findings[0], line: 0 }), "");
});

test("CLI: a lone .urdf or path argument runs check; anything else is still an unknown command", () => {
  let out = "";
  let err = "";
  const io = {
    out: (/** @type {string} */ t) => (out += t),
    err: (/** @type {string} */ t) => (err += t),
    readFile: (/** @type {string} */ p) =>
      readFileSync(fileURLToPath(new URL(`../${p}`, import.meta.url)), "utf8"),
    writeFile: () => {},
  };
  assert.equal(main(["examples/arm_2dof.urdf"], io), EXIT.ok);
  assert.match(out, /^examples\/arm_2dof\.urdf\n {2}robot 'example_arm_2dof'/);
  out = "";
  assert.equal(main(["chek"], io), EXIT.usage);
  assert.match(err, /unknown command 'chek'/);
});
