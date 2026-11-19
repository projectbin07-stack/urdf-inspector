// Finding records and the catalogue of every check: identifier, default
// severity, group, a one-line title and why it matters. docs/checks.md
// describes each identifier with its sources; a test keeps the two in step.

/** @typedef {"error" | "warning" | "info"} Severity */

/**
 * @typedef {object} Finding
 * @property {string} id check identifier from CHECKS
 * @property {Severity} severity
 * @property {string} message what is wrong in this model, with names and numbers
 * @property {string} path element path, e.g. /robot/joint[@name='j1']/limit
 * @property {{ kind: "robot" | "link" | "joint" | "material" | "element", name: string }} target
 * @property {number} line 1-based source line, 0 when unknown
 * @property {Record<string, unknown>} [data] machine-readable extras (reference inertia, ...)
 */

/**
 * @typedef {object} CheckInfo
 * @property {string} group
 * @property {Severity} severity the most severe level the check reports
 * @property {string} title
 * @property {string} why
 */

/** @type {Record<string, CheckInfo>} */
export const CHECKS = {
  // ---------------------------------------------------------------- input
  "xml-syntax": {
    group: "input",
    severity: "error",
    title: "XML is not well formed",
    why: "Every URDF consumer starts with an XML parser; a syntax error stops robot_state_publisher, RViz and Gazebo before they read a single link.",
  },
  "not-urdf": {
    group: "input",
    severity: "error",
    title: "Root element is not <robot>",
    why: "urdfdom looks for a <robot> root element and rejects anything else, for example an SDF <sdf> file or a launch file.",
  },
  xacro: {
    group: "input",
    severity: "error",
    title: "Unexpanded xacro",
    why: "xacro: elements, ${...} expressions and $(...) substitutions are only meaningful to the xacro preprocessor. Plain URDF parsers see literal text such as '${wheel_radius}', which is not a number. Run xacro first and inspect its output.",
  },
  "version-unsupported": {
    group: "input",
    severity: "warning",
    title: "Unsupported URDF version attribute",
    why: "urdfdom accepts version 1.0 to 1.2 on <robot>; other values make it refuse the file.",
  },
  "missing-attribute": {
    group: "input",
    severity: "error",
    title: "Required attribute or element missing",
    why: "urdfdom rejects links, joints, inertials and geometries that lack required attributes (names, parent/child links, mass value, all six inertia terms, radius/length, mesh filename).",
  },
  "bad-number": {
    group: "input",
    severity: "error",
    title: "Value is not a number or has the wrong count",
    why: "Vectors such as xyz, rpy, axis and size need exactly three numbers; a typo or a unit suffix makes urdfdom fail to load the model.",
  },
  "unknown-joint-type": {
    group: "input",
    severity: "error",
    title: "Joint type missing or unknown",
    why: "The URDF joint types are revolute, continuous, prismatic, fixed, floating and planar. Anything else is rejected by urdfdom.",
  },
  "unknown-geometry": {
    group: "input",
    severity: "warning",
    title: "Geometry this tool does not understand",
    why: "URDF 1.0 geometry is box, cylinder, sphere or mesh. Other shapes (for example capsule, only in recent urdfdom) are not read by every consumer; this tool ignores them for mass and inertia checks.",
  },
  "missing-geometry": {
    group: "input",
    severity: "error",
    title: "Visual or collision without geometry",
    why: "urdfdom requires a <geometry> with exactly one shape inside every <visual> and <collision> and rejects the link otherwise.",
  },
  "unknown-element": {
    group: "input",
    severity: "info",
    title: "Element not part of URDF",
    why: "Unknown top-level elements are ignored by urdfdom. This is often harmless (vendor extensions) but can also be a misspelt <joint> or <link> that silently disappears.",
  },
  "undefined-material": {
    group: "input",
    severity: "warning",
    title: "Material referenced but never defined",
    why: "A <material name='x'/> without a colour must refer to a top-level material of that name. urdfdom logs an error for an undefined one and RViz falls back to a default colour.",
  },
  "duplicate-material": {
    group: "input",
    severity: "error",
    title: "Top-level material defined twice",
    why: "urdfdom requires top-level material names to be unique and fails to load the model otherwise.",
  },
  // ------------------------------------------------------------ structure
  "no-links": {
    group: "structure",
    severity: "error",
    title: "Robot has no links",
    why: "A URDF needs at least one link; urdfdom fails with 'No link elements found'.",
  },
  "duplicate-link": {
    group: "structure",
    severity: "error",
    title: "Two links with the same name",
    why: "Link names are TF frame names and must be unique; urdfdom refuses the model.",
  },
  "duplicate-joint": {
    group: "structure",
    severity: "error",
    title: "Two joints with the same name",
    why: "Joint names key /joint_states, ros2_control and MoveIt; urdfdom refuses duplicates.",
  },
  "missing-link": {
    group: "structure",
    severity: "error",
    title: "Joint refers to a link that does not exist",
    why: "A parent or child name that matches no link (often a typo or a renamed link) makes tree construction fail.",
  },
  "self-joint": {
    group: "structure",
    severity: "error",
    title: "Joint connects a link to itself",
    why: "Parent and child must differ; a self-loop is a cycle of length one.",
  },
  "multiple-parents": {
    group: "structure",
    severity: "error",
    title: "Link is the child of more than one joint",
    why: "URDF describes a tree: every link except the root has exactly one parent joint. Closed chains cannot be expressed in URDF.",
  },
  cycle: {
    group: "structure",
    severity: "error",
    title: "Joints form a cycle",
    why: "A cycle leaves no root link and makes forward kinematics undefined; urdfdom fails to find the root.",
  },
  "orphan-link": {
    group: "structure",
    severity: "error",
    title: "Link not connected to the main tree",
    why: "urdfdom requires exactly one root. A second root is usually a link whose joint was forgotten or misnamed, and it would have no transform in TF.",
  },
  // --------------------------------------------------------------- frames
  "no-base-link": {
    group: "frames",
    severity: "warning",
    title: "No link called base_link",
    why: "REP 105 names the frame rigidly attached to a mobile base 'base_link', and navigation, localisation and many drivers default to it. Arms by ROS-Industrial convention use it too.",
  },
  "footprint-relation": {
    group: "frames",
    severity: "warning",
    title: "base_footprint not rigidly attached to base_link",
    why: "base_footprint is the base projected onto the floor. It must be connected to base_link by fixed joints only, with no roll or pitch between them, or the 2D pose used by navigation tilts and moves with the joints.",
  },
  "footprint-height": {
    group: "frames",
    severity: "warning",
    title: "base_footprint is not at wheel contact height",
    why: "base_footprint should lie on the floor plane. When wheels are detected, the lowest wheel contact point should be at z = 0 in base_footprint; otherwise costmaps and the robot model in RViz float or sink.",
  },
  "odom-in-urdf": {
    group: "frames",
    severity: "warning",
    title: "World frame (odom, map, earth) defined as a link",
    why: "REP 105 frames odom, map and earth are published by odometry and localisation nodes. A link with that name in the URDF makes robot_state_publisher publish a second, fixed transform into the same tree.",
  },
  "root-inertia": {
    group: "frames",
    severity: "warning",
    title: "Root link has an inertial block",
    why: "kdl_parser (used by robot_state_publisher and many solvers) warns that KDL does not support inertia on the root link and ignores it. Add a massless root such as base_footprint with a fixed joint to the massive base.",
  },
  "kdl-unsupported-joint": {
    group: "frames",
    severity: "warning",
    title: "Floating or planar joint",
    why: "kdl_parser converts floating and planar joints into fixed joints, so robot_state_publisher will never move the child. These joints are usually better published as a TF transform by another node.",
  },
  "optical-frame": {
    group: "frames",
    severity: "warning",
    title: "Optical frame not in the z-forward, x-right, y-down convention",
    why: "REP 103 defines frames with an '_optical' suffix as z forward, x right, y down relative to the body (x forward, y left, z up). Camera drivers publish images in the optical frame; a wrong rotation projects points and detections in the wrong direction.",
  },
  "camera-no-optical": {
    group: "frames",
    severity: "info",
    title: "Camera link without an optical frame",
    why: "Image and point cloud messages should carry an '_optical' frame (REP 103). Without one, a driver either uses the body-convention camera link (wrong axes) or needs a static transform from elsewhere.",
  },
  // --------------------------------------------------------------- joints
  "axis-zero": {
    group: "joints",
    severity: "error",
    title: "Joint axis is the zero vector",
    why: "A zero axis gives a rotation or translation about nothing; the joint never moves and some solvers divide by its length.",
  },
  "axis-not-unit": {
    group: "joints",
    severity: "warning",
    title: "Joint axis is not unit length",
    why: "The URDF specification asks for a normalised axis. Consumers differ in whether they normalise it, so a prismatic joint can move further than commanded in one tool and not in another.",
  },
  "axis-ignored": {
    group: "joints",
    severity: "info",
    title: "Axis on a fixed or floating joint",
    why: "fixed and floating joints have no axis; the value is ignored, which may mean the joint type is not the one intended.",
  },
  "limit-missing": {
    group: "joints",
    severity: "error",
    title: "Revolute or prismatic joint without <limit>",
    why: "urdfdom rejects revolute and prismatic joints without limits ('does not specify limits').",
  },
  "limit-attr-missing": {
    group: "joints",
    severity: "error",
    title: "<limit> without effort or velocity",
    why: "Before URDF 1.2, effort and velocity are required attributes of <limit>; urdfdom refuses the joint without them.",
  },
  "limit-order": {
    group: "joints",
    severity: "error",
    title: "Lower limit is greater than upper limit",
    why: "No position satisfies the limits. Controllers clamp every command to an empty range and MoveIt cannot sample a valid state.",
  },
  "limit-equal": {
    group: "joints",
    severity: "warning",
    title: "Lower and upper limits are equal",
    why: "A movable joint that cannot move. Often a missing lower or upper attribute (both default to 0).",
  },
  "limit-zero": {
    group: "joints",
    severity: "warning",
    title: "Effort or velocity limit is zero",
    why: "Controllers and simulators that enforce URDF limits clamp commands to zero, so the joint stays still without an obvious error.",
  },
  "limit-negative": {
    group: "joints",
    severity: "error",
    title: "Negative effort or velocity limit",
    why: "Limits are magnitudes. urdfdom rejects negative values from URDF 1.2 on, and older parsers produce undefined clamping.",
  },
  "limit-degrees": {
    group: "joints",
    severity: "warning",
    title: "Revolute limit looks like degrees",
    why: "REP 103 and URDF use radians. A limit of 90 means about 14 turns, not a quarter turn.",
  },
  "continuous-limits": {
    group: "joints",
    severity: "warning",
    title: "Continuous joint with lower/upper limits",
    why: "continuous joints have no position limits, so lower and upper are ignored. If the range matters, the joint should be revolute.",
  },
  "full-range-revolute": {
    group: "joints",
    severity: "info",
    title: "Revolute range of a full turn or more",
    why: "A revolute joint spanning 2*pi or more behaves like a continuous joint but planners treat it as bounded and may take the long way round.",
  },
  "mimic-unknown": {
    group: "joints",
    severity: "error",
    title: "Mimic refers to an unknown joint",
    why: "The mimicked joint must exist; robot_state_publisher and simulators cannot compute the mimic joint's position otherwise.",
  },
  "mimic-self": {
    group: "joints",
    severity: "error",
    title: "Joint mimics itself",
    why: "A self-referencing mimic has no defined position.",
  },
  "mimic-chain": {
    group: "joints",
    severity: "warning",
    title: "Mimic of a mimic joint, or mimic of a fixed joint",
    why: "Chained mimics and mimics of fixed joints are not resolved by every consumer; point every mimic joint at an actuated joint.",
  },
  "mimic-zero": {
    group: "joints",
    severity: "warning",
    title: "Mimic multiplier is zero",
    why: "With multiplier 0 the joint never follows its source, which is almost always a typo.",
  },
  "dynamics-negative": {
    group: "joints",
    severity: "error",
    title: "Negative damping or friction",
    why: "Negative damping injects energy; simulators that use the value become unstable.",
  },
  // ---------------------------------------------------------- mass, inertia
  "inertial-missing": {
    group: "inertia",
    severity: "warning",
    title: "Link moved by a joint has no <inertial>",
    why: "Dynamics, simulation and gravity compensation need a mass and inertia for every link that a non-fixed joint moves.",
  },
  "mass-nonpositive": {
    group: "inertia",
    severity: "error",
    title: "Mass is zero or negative",
    why: "Physical bodies have positive mass. Gazebo treats a link with mass <= 0 like one without <inertial>.",
  },
  "inertia-not-pd": {
    group: "inertia",
    severity: "error",
    title: "Inertia tensor is not positive definite",
    why: "A physical rotational inertia has three positive principal moments. A tensor with a zero or negative eigenvalue makes physics engines produce infinite or negative angular acceleration.",
  },
  "inertia-triangle": {
    group: "inertia",
    severity: "error",
    title: "Principal moments violate the triangle inequality",
    why: "For any real mass distribution each principal moment is at most the sum of the other two (I1 + I2 >= I3). A tensor that breaks this cannot come from any body, even if it is positive definite.",
  },
  "inertia-implausible": {
    group: "inertia",
    severity: "warning",
    title: "Inertia far from the value implied by mass and geometry",
    why: "For a link with a single primitive shape the solid-body inertia is known. Principal moments ten times larger or smaller than that usually mean placeholder values (all 1.0), a unit error, or the wrong formula, and make the link spin too easily or too sluggishly in simulation.",
  },
  "density-implausible": {
    group: "inertia",
    severity: "warning",
    title: "Mass implies an implausible density",
    why: "Mass divided by the volume of a single primitive shape outside 10 to 25000 kg/m^3 usually means grams instead of kilograms or a geometry in millimetres.",
  },
  "gazebo-dropped": {
    group: "inertia",
    severity: "warning",
    title: "Massless link that Gazebo drops with its children",
    why: "The URDF to SDF conversion removes a link with no <inertial> or mass <= 0 unless fixed-joint lumping merges it into a massive parent, and it drops the link's child joints and links with it ('not modeled in sdf').",
  },
  // ---------------------------------------------------------------- units
  "units-mm": {
    group: "units",
    severity: "warning",
    title: "Lengths that look like millimetres",
    why: "REP 103 specifies metres. A 300 m wheel base or a 50 m wheel radius is a CAD export in millimetres; the model is 1000 times too large.",
  },
  "rpy-degrees": {
    group: "units",
    severity: "warning",
    title: "Roll, pitch or yaw that looks like degrees",
    why: "URDF angles are radians. Values above 2*pi (for example 90) are almost always degrees.",
  },
  // ------------------------------------------------------------- geometry
  "geometry-nonpositive": {
    group: "geometry",
    severity: "error",
    title: "Box, cylinder or sphere with a non-positive dimension",
    why: "Zero or negative sizes create degenerate collision shapes that physics engines reject or treat as points.",
  },
  "collision-missing": {
    group: "geometry",
    severity: "warning",
    title: "Visual geometry without collision geometry",
    why: "Gazebo and MoveIt only use <collision>. A link that is visible but has no collision passes through the floor and through obstacles in planning.",
  },
  "mesh-scheme": {
    group: "geometry",
    severity: "warning",
    title: "Mesh filename that will not resolve everywhere",
    why: "package:// and file:// are the URI forms that both RViz and Gazebo with the ROS integration resolve. Relative paths depend on the working directory of whichever process loads the model, http(s) needs network access at load time and is not supported everywhere, and model:// is a Gazebo convention that RViz does not understand.",
  },
  "mesh-format": {
    group: "geometry",
    severity: "warning",
    title: "Mesh format not commonly supported",
    why: "RViz and Gazebo load STL, COLLADA (.dae) and OBJ reliably; other formats may load in one tool and not in the other.",
  },
  "mesh-scale": {
    group: "geometry",
    severity: "error",
    title: "Mesh scale zero or negative",
    why: "A zero scale collapses the mesh. A negative scale mirrors it and flips its normals, which breaks lighting and can turn collision meshes inside out.",
  },
  // ------------------------------------------------------- ros2_control, gazebo
  "ros2-control-joint-unknown": {
    group: "integration",
    severity: "error",
    title: "<ros2_control> names a joint that is not in the URDF",
    why: "The controller manager matches hardware joints to URDF joints by name; an unknown name makes the hardware component fail to configure.",
  },
  "ros2-control-no-interface": {
    group: "integration",
    severity: "warning",
    title: "<ros2_control> joint without command or state interfaces",
    why: "A joint with no state_interface publishes nothing to /joint_states, and one with no command_interface cannot be driven by any controller.",
  },
  "ros2-control-summary": {
    group: "integration",
    severity: "info",
    title: "ros2_control hardware summary",
    why: "Lists the hardware plugin, joints and interfaces so they can be compared with the controller configuration.",
  },
  "gazebo-reference-unknown": {
    group: "integration",
    severity: "warning",
    title: "<gazebo reference> to an unknown link or joint",
    why: "Gazebo extensions are attached by name; a wrong reference silently drops friction, material or sensor settings.",
  },
  "gazebo-summary": {
    group: "integration",
    severity: "info",
    title: "Gazebo extension summary",
    why: "Lists Gazebo plugins and per-link extensions so they can be reviewed. They are recognised, not executed.",
  },
  "transmission-joint-unknown": {
    group: "integration",
    severity: "warning",
    title: "<transmission> names a joint that is not in the URDF",
    why: "Transmission loaders match joints by name; an unknown one is skipped or fails to load.",
  },
  // --------------------------------------------------------------- wheels
  "wheel-axis": {
    group: "wheels",
    severity: "warning",
    title: "Wheel joint axis is not lateral",
    why: "For a differential or mecanum base each wheel must spin about the base's y axis (REP 103: x forward, y left). Any other axis drives the robot sideways or not at all in simulation.",
  },
  "wheel-cylinder-axis": {
    group: "wheels",
    severity: "warning",
    title: "Wheel cylinder not aligned with the joint axis",
    why: "URDF cylinders are along their local z axis. Without rpy='1.5708 0 0' (or similar) the wheel lies flat, collides with the floor on its face and rolls on nothing.",
  },
  "wheel-direction-mixed": {
    group: "wheels",
    severity: "warning",
    title: "Wheel axes point in opposite directions",
    why: "diff_drive_controller and mecanum controllers assume a positive wheel velocity moves the robot forward on every wheel. A flipped axis on one side makes the robot spin when told to drive straight.",
  },
  "wheel-radius-mismatch": {
    group: "wheels",
    severity: "warning",
    title: "Drive wheels have different radii",
    why: "Wheel radius sets the speed to distance ratio; unequal radii make simulated odometry curve on a straight command.",
  },
  "wheel-layout": {
    group: "wheels",
    severity: "warning",
    title: "Wheel positions are not symmetric",
    why: "Mecanum kinematics assume four wheels at (+-lx, +-ly) and differential drive two wheels on a common axle at +-ly. Asymmetry makes the measured lx, ly and the controller's wheel separation wrong.",
  },
  "wheel-base-summary": {
    group: "wheels",
    severity: "info",
    title: "Detected wheel base",
    why: "Measured wheel radius and spacing, for comparison with the controller parameters; a mecanum-kinematics config can be exported from them.",
  },
};

/** @type {Record<Severity, number>} */
export const SEVERITY_RANK = { error: 0, warning: 1, info: 2 };

/**
 * @param {string} id
 * @param {Severity} severity
 * @param {string} message
 * @param {string} path
 * @param {Finding["target"]} target
 * @param {number} [line]
 * @param {Record<string, unknown>} [data]
 * @returns {Finding}
 */
export function finding(id, severity, message, path, target, line = 0, data) {
  if (!(id in CHECKS)) throw new Error(`unknown check id ${id}`);
  /** @type {Finding} */
  const f = { id, severity, message, path, target, line };
  if (data) f.data = data;
  return f;
}

/**
 * Stable order: severity, then the order the checks produced them.
 * @param {Finding[]} list
 * @returns {Finding[]}
 */
export const sortFindings = (list) =>
  list
    .map((f, k) => ({ f, k }))
    .sort((a, b) => SEVERITY_RANK[a.f.severity] - SEVERITY_RANK[b.f.severity] || a.k - b.k)
    .map((x) => x.f);

/** @param {Finding[]} list */
export function countBySeverity(list) {
  const c = { error: 0, warning: 0, info: 0 };
  for (const f of list) c[f.severity]++;
  return c;
}
