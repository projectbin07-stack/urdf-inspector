import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import { fileURLToPath } from "node:url";
import { EXIT, main } from "../src/cli.js";
import { CHECKS } from "../src/core/findings.js";
import { VERSION } from "../src/core/version.js";
import { robot, solidLink, joint } from "./helpers.js";

/** In-memory files for the CLI. */
const FILES = {
  "warn.urdf": robot(
    '<link name="base_footprint"/>' +
      solidLink("base_link") +
      joint("bj", "fixed", "base_footprint", "base_link") +
      solidLink("arm") +
      joint(
        "j",
        "revolute",
        "base_link",
        "arm",
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="0" velocity="1"/>',
      ),
  ),
  "error.urdf": robot(solidLink("base_link") + joint("j", "fixed", "base_link", "nope")),
  "xacro.urdf": robot('<xacro:property name="a" value="1"/>'),
  "broken.xml": "<robot><link></robot>",
  "notree.urdf": robot(
    '<link name="a"/><link name="b"/>' +
      joint("x", "fixed", "a", "b") +
      joint("y", "fixed", "b", "a"),
  ),
};

/**
 * @param {string[]} argv
 * @returns {{ code: number, out: string, err: string, written: Record<string, string> }}
 */
function cli(argv) {
  let out = "";
  let err = "";
  /** @type {Record<string, string>} */
  const written = {};
  const code = main(argv, {
    out: (t) => (out += t),
    err: (t) => (err += t),
    readFile: (p) => {
      if (p.startsWith("examples/"))
        return readFileSync(fileURLToPath(new URL(`../${p}`, import.meta.url)), "utf8");
      if (p in FILES) return FILES[/** @type {keyof typeof FILES} */ (p)];
      throw Object.assign(new Error(`ENOENT: no such file '${p}'`), { code: "ENOENT" });
    },
    writeFile: (p, d) => (written[p] = d),
  });
  return { code, out, err, written };
}

test("check exit codes: 0 clean, 0 or 1 for warnings depending on --strict, 2 for errors", () => {
  assert.equal(cli(["check", "examples/arm_2dof.urdf"]).code, EXIT.ok);
  assert.equal(cli(["check", "examples/arm_2dof.urdf", "--strict"]).code, EXIT.ok);
  assert.equal(cli(["check", "warn.urdf"]).code, EXIT.ok);
  assert.equal(cli(["check", "warn.urdf", "--strict"]).code, EXIT.warnings);
  assert.equal(cli(["check", "error.urdf"]).code, EXIT.errors);
  assert.equal(cli(["check", "error.urdf", "--strict"]).code, EXIT.errors);
  assert.equal(cli(["check", "xacro.urdf"]).code, EXIT.errors);
  assert.equal(cli(["check", "broken.xml"]).code, EXIT.errors);
  assert.equal(cli(["check", "examples/broken_robot.urdf"]).code, EXIT.errors);
});

test("check --json shape", () => {
  const r = cli(["check", "warn.urdf", "--json"]);
  const j = JSON.parse(r.out);
  assert.deepEqual(Object.keys(j), ["tool", "version", "file", "summary", "findings"]);
  assert.equal(j.tool, "urdf-inspector");
  assert.equal(j.version, VERSION);
  assert.equal(j.file, "warn.urdf");
  assert.deepEqual(Object.keys(j.summary).sort(), [
    "com",
    "errors",
    "infos",
    "jointTypes",
    "joints",
    "links",
    "name",
    "root",
    "totalMass",
    "warnings",
  ]);
  assert.equal(j.summary.warnings, 1);
  assert.equal(j.summary.totalMass, 2);
  const f = j.findings[0];
  assert.deepEqual(Object.keys(f), ["id", "severity", "message", "path", "target", "line"]);
  assert.equal(f.id, "limit-zero");
  assert.deepEqual(f.target, { kind: "joint", name: "j" });
  assert.ok(f.line > 0);
  for (const x of j.findings) assert.ok(x.id in CHECKS);
});

test("check output is byte-identical across runs", () => {
  const a = cli(["check", "examples/broken_robot.urdf", "--json"]).out;
  const b = cli(["check", "examples/broken_robot.urdf", "--json"]).out;
  assert.equal(a, b);
  const t1 = cli(["check", "examples/broken_robot.urdf"]).out;
  assert.equal(t1, cli(["check", "examples/broken_robot.urdf"]).out);
  assert.match(t1, /7 error\(s\), 14 warning\(s\), 2 note\(s\)/);
  assert.match(t1, /^ERROR orphan-link {2}\(line \d+\)$/m);
});

test("check text for xacro and XML errors", () => {
  assert.match(
    cli(["check", "xacro.urdf"]).out,
    /ERROR xacro[\s\S]*<xacro:property 'a'> is a xacro element/,
  );
  assert.match(
    cli(["check", "broken.xml"]).out,
    /ERROR xml-syntax {2}\(line 1\)[\s\S]*<\/robot> does not match <link>/,
  );
});

test("tree", () => {
  const r = cli(["tree", "examples/arm_2dof.urdf"]);
  assert.equal(r.code, 0);
  assert.equal(
    r.out,
    [
      "world",
      "`-- world_joint (fixed) -> base_link",
      "    `-- shoulder_joint (revolute, axis 0 0 1, [-2.6, 2.6]) -> upper_arm",
      "        `-- elbow_joint (revolute, axis 0 1 0, [-1.57, 1.57]) -> forearm",
      "            `-- tool_joint (fixed) -> tool0",
      "",
    ].join("\n"),
  );
  const broken = cli(["tree", "examples/broken_robot.urdf"]).out;
  assert.match(broken, /^spare_link {2}\(not connected\)$/m);
  assert.match(broken, /mimics flap_motor_joint/);
});

test("fk text and JSON", () => {
  const r = cli([
    "fk",
    "examples/arm_2dof.urdf",
    "--joint",
    "shoulder_joint=0.5",
    "-j",
    "elbow_joint=0.3",
  ]);
  assert.equal(r.code, 0);
  assert.match(r.out, /^tool0 +0\.1677 +0\.0916 +0\.2409 +0\.0000 +0\.3000 +0\.5000$/m);
  const j = JSON.parse(
    cli(["fk", "examples/arm_2dof.urdf", "--joint", "elbow_joint=-1.5707963267948966", "--json"])
      .out,
  );
  assert.equal(j.frame, "world");
  assert.deepEqual(j.joints, { shoulder_joint: 0, elbow_joint: -1.57079632679 });
  const tool = j.links.find((/** @type {any} */ l) => l.name === "tool0");
  assert.deepEqual(tool.xyz, [0, 0, 0.5]);
  assert.equal(j.mass, 2.8);
  assert.deepEqual(j.com, [0, 0, 0.091964]);
});

test("fk argument errors", () => {
  assert.equal(cli(["fk", "examples/arm_2dof.urdf", "--joint", "x"]).code, EXIT.usage);
  assert.match(
    cli(["fk", "examples/arm_2dof.urdf", "--joint", "elbow_joint=abc"]).err,
    /not a number/,
  );
  assert.match(
    cli(["fk", "examples/arm_2dof.urdf", "--joint", "nope=1"]).err,
    /no joint named 'nope'/,
  );
  assert.match(
    cli(["fk", "examples/arm_2dof.urdf", "--joint", "tool_joint=1"]).err,
    /fixed joint and has no position/,
  );
});

test("export-mecanum to stdout and to a file", () => {
  const r = cli(["export-mecanum", "examples/mecanum_base.urdf"]);
  assert.equal(r.code, 0);
  assert.match(r.out, /^half_width = 0\.13 /m);
  const w = cli(["export-mecanum", "examples/mecanum_base.urdf", "--out", "cfg.toml"]);
  assert.equal(w.written["cfg.toml"], r.out);
  assert.match(w.err, /wrote cfg\.toml/);
  const no = cli(["export-mecanum", "examples/arm_2dof.urdf"]);
  assert.equal(no.code, EXIT.errors);
  assert.match(no.err, /no wheel joints/);
});

test("commands that need a tree refuse models without one", () => {
  for (const cmd of ["tree", "fk", "export-mecanum"]) {
    const r = cli([cmd, "xacro.urdf"]);
    assert.equal(r.code, EXIT.errors);
    assert.match(r.err, /could not be read as a URDF tree[\s\S]*xacro:/);
  }
  assert.equal(cli(["tree", "notree.urdf"]).code, EXIT.errors);
});

test("help, version, checks and usage errors", () => {
  assert.match(cli(["--help"]).out, /^urdf-inspector \d+\.\d+\.\d+/);
  assert.equal(cli(["--version"]).out, `${VERSION}\n`);
  const none = cli([]);
  assert.equal(none.code, EXIT.usage);
  assert.match(none.out, /Usage:/);
  const list = cli(["checks"]);
  assert.equal(list.out.trim().split("\n").length, Object.keys(CHECKS).length);
  assert.match(list.out, /^error {3}inertia {5}inertia-triangle/m);
  assert.equal(cli(["frobnicate", "x"]).code, EXIT.usage);
  assert.equal(cli(["check"]).code, EXIT.usage);
  assert.equal(cli(["check", "a", "b"]).code, EXIT.usage);
  assert.equal(cli(["check", "a", "--bogus"]).code, EXIT.usage);
  const missing = cli(["check", "missing.urdf"]);
  assert.equal(missing.code, EXIT.usage);
  assert.match(missing.err, /cannot read file/);
});

test("other exceptions propagate", () => {
  assert.throws(() =>
    main(["check", "x.urdf"], {
      out: () => {},
      err: () => {},
      readFile: () => {
        throw new TypeError("boom");
      },
      writeFile: () => {},
    }),
  );
});

test("the bin script sets the process exit code", () => {
  const bin = fileURLToPath(new URL("../bin/urdf-inspector.js", import.meta.url));
  const cwd = fileURLToPath(new URL("..", import.meta.url));
  const ok = spawnSync(process.execPath, [bin, "check", "examples/mecanum_base.urdf"], {
    cwd,
    encoding: "utf8",
  });
  assert.equal(ok.status, 0);
  assert.match(ok.stdout, /0 error\(s\), 0 warning\(s\), 3 note\(s\)/);
  const bad = spawnSync(process.execPath, [bin, "check", "examples/broken_robot.urdf", "--json"], {
    cwd,
    encoding: "utf8",
  });
  assert.equal(bad.status, 2);
  assert.equal(JSON.parse(bad.stdout).summary.errors, 7);
});
