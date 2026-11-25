// Headless command-line interface. Runs the same src/core code as the
// browser app, with the built-in XML reader instead of DOMParser.

import { parseArgs } from "node:util";
import { CHECKS } from "./core/findings.js";
import { inspectText } from "./core/inspect.js";
import { checkJson, checkText, fkData, fkText, treeText } from "./core/report.js";
import { VERSION } from "./core/version.js";
import { mecanumToml } from "./core/wheels.js";

/**
 * @typedef {object} CliIo
 * @property {(text: string) => void} out
 * @property {(text: string) => void} err
 * @property {(path: string) => string} readFile
 * @property {(path: string, data: string) => void} writeFile
 */

export const EXIT = { ok: 0, warnings: 1, errors: 2, usage: 3 };

const HELP = `urdf-inspector ${VERSION}: find the mistakes in a URDF that make robots misbehave
in RViz, Gazebo and ros2_control, and show its structure.

Usage:
  urdf-inspector check <robot.urdf> [--json] [--strict]
  urdf-inspector <robot.urdf>      the same as check
  urdf-inspector tree <robot.urdf>
  urdf-inspector fk <robot.urdf> [--joint name=value ...] [--json]
  urdf-inspector export-mecanum <robot.urdf> [--out config.toml]
  urdf-inspector checks            list every check with its severity
  urdf-inspector --help | --version

check     run every check; findings sorted by severity
          --json     machine-readable report
          --strict   exit 1 when there are warnings (and no errors)
tree      print the kinematic tree
fk        link poses in the root frame for the given joint positions
          (rad for revolute/continuous, m for prismatic; others are 0;
          mimic joints follow their source), plus mass and centre of mass
export-mecanum
          write a mecanum-kinematics TOML config (wheel radius, lx, ly)
          measured from a four-wheel base

Exit codes: 0 no errors (and no warnings with --strict), 1 warnings with
--strict, 2 errors in the model (or nothing to export), 3 usage or file error.

The input must be plain URDF: run xacro first. Nothing is uploaded anywhere.
`;

class UsageError extends Error {}

const COMMANDS = ["check", "tree", "fk", "export-mecanum"];

/**
 * @param {string[]} argv
 * @param {CliIo} io
 * @returns {number} exit code
 */
export function main(argv, io) {
  try {
    return run(argv, io);
  } catch (e) {
    if (
      e instanceof UsageError ||
      (e instanceof Error && "code" in e && String(e.code).startsWith("ERR_PARSE_ARGS"))
    ) {
      io.err(`urdf-inspector: ${e.message}\nRun 'urdf-inspector --help' for usage.\n`);
      return EXIT.usage;
    }
    if (
      e instanceof Error &&
      "code" in e &&
      (e.code === "ENOENT" || e.code === "EISDIR" || e.code === "EACCES")
    ) {
      io.err(`urdf-inspector: cannot read file: ${e.message}\n`);
      return EXIT.usage;
    }
    throw e;
  }
}

/**
 * @param {string[]} argv
 * @param {CliIo} io
 */
function run(argv, io) {
  const { values, positionals } = parseArgs({
    args: argv,
    allowPositionals: true,
    options: {
      help: { type: "boolean", short: "h" },
      version: { type: "boolean", short: "v" },
      json: { type: "boolean" },
      strict: { type: "boolean" },
      joint: { type: "string", short: "j", multiple: true },
      out: { type: "string", short: "o" },
    },
  });
  if (values.help) {
    io.out(HELP);
    return EXIT.ok;
  }
  if (values.version) {
    io.out(`${VERSION}\n`);
    return EXIT.ok;
  }
  // A lone file argument (a .urdf or .xml file, or a path) means "check it".
  if (positionals.length === 1 && /\.(urdf|xml)$|[\\/]/i.test(positionals[0]))
    positionals.unshift("check");
  const [cmd, file, ...extra] = positionals;
  if (cmd === undefined) {
    io.out(HELP);
    return EXIT.usage;
  }
  if (cmd === "checks") {
    for (const [id, c] of Object.entries(CHECKS))
      io.out(`${c.severity.padEnd(7)} ${c.group.padEnd(11)} ${id.padEnd(27)} ${c.title}\n`);
    return EXIT.ok;
  }
  if (!COMMANDS.includes(cmd)) throw new UsageError(`unknown command '${cmd}'`);
  if (file === undefined) throw new UsageError(`${cmd} needs a URDF file`);
  if (extra.length > 0) throw new UsageError(`unexpected argument '${extra[0]}'`);

  const report = inspectText(io.readFile(file));

  if (cmd === "check") {
    if (values.json) io.out(JSON.stringify(checkJson(report, file), null, 2) + "\n");
    else io.out(checkText(report, file));
    if (report.summary.errors > 0) return EXIT.errors;
    if (values.strict && report.summary.warnings > 0) return EXIT.warnings;
    return EXIT.ok;
  }

  const ctx = report.context;
  if (!ctx || ctx.tree.root === null) {
    io.err(`urdf-inspector: ${file} could not be read as a URDF tree:\n`);
    for (const f of report.findings.filter((x) => x.severity === "error").slice(0, 10))
      io.err(`  ${f.id}: ${f.message}\n`);
    return EXIT.errors;
  }

  if (cmd === "tree") {
    io.out(treeText(ctx.tree));
    return EXIT.ok;
  }

  if (cmd === "fk") {
    /** @type {Record<string, number>} */
    const q = {};
    for (const spec of values.joint ?? []) {
      const m = /^([^=]+)=(.+)$/.exec(spec);
      if (!m) throw new UsageError(`--joint expects name=value, got '${spec}'`);
      const v = Number(m[2]);
      if (!Number.isFinite(v)) throw new UsageError(`--joint ${m[1]}: '${m[2]}' is not a number`);
      const j = ctx.tree.joints.get(m[1]);
      if (!j) throw new UsageError(`--joint: no joint named '${m[1]}'`);
      if (j.type !== "revolute" && j.type !== "continuous" && j.type !== "prismatic")
        throw new UsageError(`--joint: '${m[1]}' is a ${j.type} joint and has no position`);
      q[m[1]] = v;
    }
    const d = fkData(ctx.tree, q);
    io.out(values.json ? JSON.stringify(d, null, 2) + "\n" : fkText(d));
    return EXIT.ok;
  }

  // export-mecanum
  const res = mecanumToml(ctx.wheels, ctx.robot.name);
  if (!res.ok) {
    io.err(`urdf-inspector: cannot export a mecanum config: ${res.reason}\n`);
    return EXIT.errors;
  }
  if (values.out) {
    io.writeFile(values.out, res.toml);
    io.err(`wrote ${values.out}\n`);
  } else io.out(res.toml);
  return EXIT.ok;
}
