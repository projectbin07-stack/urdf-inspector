// <ros2_control>, <gazebo> and <transmission>: recognised and summarised
// against the URDF names, never executed.

import { finding } from "../findings.js";

/** @typedef {import("./context.js").Context} Context */
/** @typedef {import("../findings.js").Finding} Finding */

/**
 * @param {Context} ctx
 * @returns {Finding[]}
 */
export function checkIntegration(ctx) {
  const { robot, tree } = ctx;
  /** @type {Finding[]} */
  const out = [];
  const robotTarget = /** @type {const} */ ({ kind: "robot", name: robot.name });
  for (const rc of robot.ros2Control) {
    for (const j of rc.joints) {
      const t = /** @type {const} */ ({ kind: "joint", name: j.name });
      if (!tree.joints.has(j.name))
        out.push(
          finding(
            "ros2-control-joint-unknown",
            "error",
            `<ros2_control name="${rc.name}"> joint '${j.name}' is not a URDF joint`,
            j.path,
            t,
            j.line,
          ),
        );
      if (j.command.length === 0 && j.state.length === 0)
        out.push(
          finding(
            "ros2-control-no-interface",
            "warning",
            `<ros2_control name="${rc.name}"> joint '${j.name}' has no command or state interfaces`,
            j.path,
            t,
            j.line,
          ),
        );
      else if (j.state.length === 0)
        out.push(
          finding(
            "ros2-control-no-interface",
            "warning",
            `<ros2_control name="${rc.name}"> joint '${j.name}' has no state_interface; it will not appear in /joint_states`,
            j.path,
            t,
            j.line,
          ),
        );
    }
    const joints = rc.joints
      .map(
        (j) => `${j.name} [cmd: ${j.command.join(",") || "-"}; state: ${j.state.join(",") || "-"}]`,
      )
      .join("; ");
    out.push(
      finding(
        "ros2-control-summary",
        "info",
        `ros2_control '${rc.name}' (${rc.type || "no type"}), plugin ${rc.plugin || "(none)"}: ${rc.joints.length} joint(s)` +
          (joints ? `: ${joints}` : "") +
          (rc.sensors.length ? `; sensors: ${rc.sensors.join(", ")}` : ""),
        rc.path,
        robotTarget,
        rc.line,
      ),
    );
  }
  const plugins = [];
  let refs = 0;
  for (const g of robot.gazebo) {
    for (const p of g.plugins)
      plugins.push(`${p.name || "(unnamed)"} (${p.filename || "no filename"})`);
    if (g.reference === "") continue;
    refs++;
    if (!tree.links.has(g.reference) && !tree.joints.has(g.reference))
      out.push(
        finding(
          "gazebo-reference-unknown",
          "warning",
          `<gazebo reference="${g.reference}"> matches no link or joint`,
          g.path,
          { kind: "element", name: g.reference },
          g.line,
        ),
      );
  }
  if (robot.gazebo.length > 0)
    out.push(
      finding(
        "gazebo-summary",
        "info",
        `${robot.gazebo.length} <gazebo> block(s), ${refs} with a reference; plugins: ${plugins.join(", ") || "none"}`,
        "/robot/gazebo",
        robotTarget,
      ),
    );
  for (const tr of robot.transmissions)
    for (const jn of tr.joints)
      if (!tree.joints.has(jn))
        out.push(
          finding(
            "transmission-joint-unknown",
            "warning",
            `transmission '${tr.name}' names joint '${jn}', which does not exist`,
            tr.path,
            { kind: "joint", name: jn },
            tr.line,
          ),
        );
  return out;
}
