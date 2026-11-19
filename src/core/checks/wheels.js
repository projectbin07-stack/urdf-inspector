// Drive-wheel consistency for differential and four-wheel bases.

import { finding } from "../findings.js";
import { lateralSign, lengthTolerance, parallel } from "../wheels.js";
import { fmt } from "./context.js";

/** @typedef {import("./context.js").Context} Context */
/** @typedef {import("../findings.js").Finding} Finding */
/** @typedef {import("../wheels.js").Wheel} Wheel */

/**
 * @param {Context} ctx
 * @returns {Finding[]}
 */
export function checkWheels(ctx) {
  const wb = ctx.wheels;
  if (!wb) return [];
  /** @type {Finding[]} */
  const out = [];
  /** @param {Wheel} w */
  const jt = (w) => /** @type {const} */ ({ kind: "joint", name: w.joint });
  /** @param {Wheel} w */
  const jp = (w) => ctx.tree.joints.get(w.joint)?.path ?? "/robot";
  /** @param {Wheel} w */
  const jl = (w) => ctx.tree.joints.get(w.joint)?.line ?? 0;
  /** @param {import("../math.js").Vec3} v */
  const vec = (v) => `(${v.map((x) => fmt(Math.abs(x) < 1e-12 ? 0 : x)).join(" ")})`;

  for (const w of wb.wheels) {
    if (lateralSign(w) === 0)
      out.push(
        finding(
          "wheel-axis",
          "warning",
          `wheel joint '${w.joint}' spins about ${vec(w.axis)} in ${wb.baseFrame}; expected +-y (0 1 0)`,
          `${jp(w)}/axis`,
          jt(w),
          jl(w),
        ),
      );
    if (w.cylinderAxis && !parallel(w.cylinderAxis, w.axis)) {
      const link = ctx.tree.links.get(w.link);
      const shape =
        link?.collisions.find((s) => s.geometry?.type === "cylinder") ??
        link?.visuals.find((s) => s.geometry?.type === "cylinder");
      out.push(
        finding(
          "wheel-cylinder-axis",
          "warning",
          `wheel '${w.link}' cylinder lies along ${vec(w.cylinderAxis)} but the joint spins about ${vec(w.axis)}; the wheel is not upright`,
          shape ? `${shape.path}/origin` : (link?.path ?? "/robot"),
          { kind: "link", name: w.link },
          link?.line ?? 0,
        ),
      );
    }
  }

  const plus = wb.wheels.filter((w) => lateralSign(w) === 1);
  const minus = wb.wheels.filter((w) => lateralSign(w) === -1);
  if (plus.length > 0 && minus.length > 0) {
    const odd = minus.length <= plus.length ? minus : plus;
    out.push(
      finding(
        "wheel-direction-mixed",
        "warning",
        `wheel axes ${plus.map((w) => `'${w.joint}'`).join(", ")} point to +y and ${minus.map((w) => `'${w.joint}'`).join(", ")} to -y; a positive velocity drives them in opposite directions`,
        jp(odd[0]) + "/axis",
        jt(odd[0]),
        jl(odd[0]),
      ),
    );
  }

  const radii = wb.wheels.map((w) => w.radius).filter((r) => r !== null);
  if (radii.length >= 2) {
    const rmax = Math.max(...radii);
    const rmin = Math.min(...radii);
    if (rmax - rmin > 0.01 * rmax)
      out.push(
        finding(
          "wheel-radius-mismatch",
          "warning",
          `wheel radii range from ${fmt(rmin)} to ${fmt(rmax)} m`,
          jp(wb.wheels[0]),
          jt(wb.wheels[0]),
          jl(wb.wheels[0]),
        ),
      );
  }

  /** @type {string[]} */
  const layout = [];
  const zs = wb.wheels.map((w) => w.centre[2]);
  if (wb.wheels.length >= 2 && Math.max(...zs) - Math.min(...zs) > lengthTolerance(0, 0))
    layout.push(`wheel centre heights differ (${zs.map(fmt).join(", ")} m)`);
  if (wb.kind === "differential") {
    const [a, b] = wb.wheels;
    if (Math.abs(a.centre[0] - b.centre[0]) > lengthTolerance(a.centre[0], b.centre[0]))
      layout.push(
        `the two wheels are not on one axle (x = ${fmt(a.centre[0])}, ${fmt(b.centre[0])} m)`,
      );
    if (Math.abs(a.centre[1] + b.centre[1]) > lengthTolerance(a.centre[1], b.centre[1]))
      layout.push(
        `the wheels are not centred on ${wb.baseFrame} (y = ${fmt(a.centre[1])}, ${fmt(b.centre[1])} m)`,
      );
  } else if (wb.kind === "four-wheel") {
    if (wb.lx === null) layout.push("the four wheels are not one per corner");
    else {
      const c = wb.centroid;
      const xs = wb.wheels.map((w) => Math.abs(w.centre[0] - c[0]));
      const ys = wb.wheels.map((w) => Math.abs(w.centre[1] - c[1]));
      if (Math.max(...xs) - Math.min(...xs) > lengthTolerance(Math.max(...xs), 0))
        layout.push(`front and rear axle distances differ (|x| = ${xs.map(fmt).join(", ")} m)`);
      if (Math.max(...ys) - Math.min(...ys) > lengthTolerance(Math.max(...ys), 0))
        layout.push(`left and right track distances differ (|y| = ${ys.map(fmt).join(", ")} m)`);
    }
  }
  if (layout.length > 0)
    out.push(
      finding(
        "wheel-layout",
        "warning",
        layout.join("; "),
        jp(wb.wheels[0]),
        jt(wb.wheels[0]),
        jl(wb.wheels[0]),
      ),
    );

  const names = wb.wheels.map((w) => (w.label ? `${w.label}=${w.joint}` : w.joint)).join(", ");
  let msg = `${wb.wheels.length} drive wheel(s) in ${wb.baseFrame} (${names})`;
  if (wb.radius !== null) msg += `, radius ${fmt(wb.radius)} m`;
  if (wb.kind === "differential" && wb.ly !== null) msg += `, wheel separation ${fmt(2 * wb.ly)} m`;
  if (wb.kind === "four-wheel" && wb.lx !== null && wb.ly !== null)
    msg += `, lx ${fmt(wb.lx)} m, ly ${fmt(wb.ly)} m (export-mecanum writes a mecanum-kinematics config)`;
  const off = Math.hypot(wb.centroid[0], wb.centroid[1]);
  if (off > 0.001)
    msg += `; wheel centroid is ${fmt(off)} m from the ${wb.baseFrame} origin (x ${fmt(wb.centroid[0])}, y ${fmt(wb.centroid[1])})`;
  out.push(
    finding(
      "wheel-base-summary",
      "info",
      msg,
      jp(wb.wheels[0]),
      jt(wb.wheels[0]),
      jl(wb.wheels[0]),
      {
        kind: wb.kind,
        radius: wb.radius,
        lx: wb.lx,
        ly: wb.ly,
        centroid: wb.centroid,
      },
    ),
  );
  return out;
}
