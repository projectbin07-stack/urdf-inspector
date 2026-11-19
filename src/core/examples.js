// The bundled example models. All were written for this project and are
// not modelled on any commercial robot.

/** @typedef {{ id: string, file: string, title: string, note: string }} Example */

/** @type {Example[]} */
export const EXAMPLES = [
  {
    id: "mecanum",
    file: "mecanum_base.urdf",
    title: "Mecanum base with LiDAR and camera",
    note: "Example written for this tool, not a real product. Should pass with notes only.",
  },
  {
    id: "arm",
    file: "arm_2dof.urdf",
    title: "Two-joint arm",
    note: "Example written for this tool. Move the sliders to drive the forward kinematics.",
  },
  {
    id: "diff",
    file: "diff_drive_base.urdf",
    title: "Differential-drive base with caster",
    note: "Example written for this tool, not a real product. Should pass with notes only.",
  },
  {
    id: "broken",
    file: "broken_robot.urdf",
    title: "Deliberately broken robot",
    note: "Example with planted mistakes; every MISTAKE comment in the file should show up below.",
  },
];
