// Joint checks: axis, limits, mimic, dynamics.

import { finding } from "../findings.js";
import { norm } from "../math.js";

/** @typedef {import("./context.js").Context} Context */
/** @typedef {import("../findings.js").Finding} Finding */

const TWO_PI = 2 * Math.PI;

/** @param {number} v */
const f = (v) => String(Number(v.toPrecision(6)));

/**
 * @param {Context} ctx
 * @returns {Finding[]}
 */
export function checkJoints(ctx) {
  /** @type {Finding[]} */
  const out = [];
  for (const j of ctx.tree.joints.values()) {
    const t = /** @type {const} */ ({ kind: "joint", name: j.name });
    /** @param {string} id @param {import("../findings.js").Severity} sev @param {string} msg @param {string} [path] */
    const add = (id, sev, msg, path = j.path) => out.push(finding(id, sev, msg, path, t, j.line));
    const movable = j.type === "revolute" || j.type === "continuous" || j.type === "prismatic";
    const n = norm(j.axis);

    if (movable || j.type === "planar") {
      if (n === 0)
        add("axis-zero", "error", `joint '${j.name}' has axis (0 0 0)`, `${j.path}/axis`);
      else if (Math.abs(n - 1) > 1e-3)
        add(
          "axis-not-unit",
          "warning",
          `joint '${j.name}' axis (${j.axis.map(f).join(" ")}) has length ${f(n)}, not 1`,
          `${j.path}/axis`,
        );
    } else if (j.axisGiven && (j.type === "fixed" || j.type === "floating"))
      add(
        "axis-ignored",
        "info",
        `axis of ${j.type} joint '${j.name}' is ignored`,
        `${j.path}/axis`,
      );

    const lim = j.limit;
    if ((j.type === "revolute" || j.type === "prismatic") && !lim)
      add("limit-missing", "error", `${j.type} joint '${j.name}' has no <limit>`);

    if (lim) {
      const lp = lim.path;
      const lower = lim.lower ?? 0;
      const upper = lim.upper ?? 0;
      const bounded = j.type === "revolute" || j.type === "prismatic";
      if (bounded) {
        if (lower > upper)
          add(
            "limit-order",
            "error",
            `joint '${j.name}' lower limit ${f(lower)} is greater than upper limit ${f(upper)}`,
            lp,
          );
        else if (lower === upper)
          add(
            "limit-equal",
            "warning",
            `joint '${j.name}' lower and upper limits are both ${f(lower)}` +
              (lim.lower === null || lim.upper === null ? " (a missing limit defaults to 0)" : ""),
            lp,
          );
      }
      if (j.type === "revolute") {
        const big = [lim.lower, lim.upper].filter((v) => v !== null && Math.abs(v) > TWO_PI + 1e-3);
        if (big.length > 0)
          add(
            "limit-degrees",
            "warning",
            `joint '${j.name}' limit ${big.map((v) => f(/** @type {number} */ (v))).join(", ")} rad is more than a full turn; degrees?`,
            lp,
          );
        else if (upper - lower >= TWO_PI - 1e-3)
          add(
            "full-range-revolute",
            "info",
            `joint '${j.name}' range ${f(upper - lower)} rad covers a full turn`,
            lp,
          );
      }
      if (j.type === "continuous" && (lim.lower !== null || lim.upper !== null))
        add(
          "continuous-limits",
          "warning",
          `continuous joint '${j.name}' has lower/upper limits, which are ignored`,
          lp,
        );
      for (const key of /** @type {const} */ (["effort", "velocity"])) {
        const v = lim[key];
        if (v === null) continue;
        if (v < 0)
          add(
            "limit-negative",
            "error",
            `joint '${j.name}' ${key} limit is negative (${f(v)})`,
            lp,
          );
        else if (v === 0 && movable)
          add("limit-zero", "warning", `joint '${j.name}' ${key} limit is 0`, lp);
      }
    }

    if (j.mimic) {
      const mp = `${j.path}/mimic`;
      const src = ctx.tree.joints.get(j.mimic.joint);
      if (j.mimic.joint === j.name)
        add("mimic-self", "error", `joint '${j.name}' mimics itself`, mp);
      else if (!src)
        add(
          "mimic-unknown",
          "error",
          `joint '${j.name}' mimics '${j.mimic.joint}', which does not exist`,
          mp,
        );
      else if (src.mimic)
        add(
          "mimic-chain",
          "warning",
          `joint '${j.name}' mimics '${src.name}', which itself mimics '${src.mimic.joint}'`,
          mp,
        );
      else if (src.type === "fixed")
        add("mimic-chain", "warning", `joint '${j.name}' mimics fixed joint '${src.name}'`, mp);
      if (j.mimic.multiplier === 0)
        add("mimic-zero", "warning", `joint '${j.name}' mimic multiplier is 0`, mp);
    }

    if (j.dynamics) {
      for (const key of /** @type {const} */ (["damping", "friction"])) {
        if (j.dynamics[key] < 0)
          add(
            "dynamics-negative",
            "error",
            `joint '${j.name}' ${key} is negative (${f(j.dynamics[key])})`,
            `${j.path}/dynamics`,
          );
      }
    }
  }
  return out;
}
