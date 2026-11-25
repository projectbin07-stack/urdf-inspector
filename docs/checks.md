# Checks, conventions and sources

This document lists every check `urdf-inspector` runs, the threshold it
uses, and where the rule comes from. Identifiers match the `id` field of the
CLI's JSON output and the list printed by `urdf-inspector checks`; a test
fails if a check is missing here.

Severities:

- **error**: the model is invalid. urdfdom (and with it robot_state_publisher,
  RViz and the Gazebo URDF import) refuses it, or it describes something
  physically impossible.
- **warning**: the model loads, but something will behave strangely: a joint
  that does not move, a link that Gazebo drops, a frame pointing the wrong
  way.
- **info** (shown as "note"): a summary or a convention worth knowing about.

The checks have been written from the specifications and source code cited
below and tested on the bundled examples and minimal fixtures. They have not
been compared against every consumer's behaviour on real robot models.

## Sources

| Key                | Source                                                                                                                                                                                                                                                                                    |
| ------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| URDF-XML           | ROS wiki, "urdf/XML" specification: <http://wiki.ros.org/urdf/XML>, with the [robot](http://wiki.ros.org/urdf/XML/robot), [link](http://wiki.ros.org/urdf/XML/link) and [joint](http://wiki.ros.org/urdf/XML/joint) pages.                                                                |
| urdfdom            | The reference C++ parser, <https://github.com/ros/urdfdom>: `xsd/urdf.xsd` (attribute defaults), `urdf_parser/src/model.cpp`, `link.cpp` and `joint.cpp` (what is rejected). The wiki pages block automated access, so defaults and required attributes were checked against these files. |
| REP 103            | T. Foote, M. Purvis, "Standard Units of Measure and Coordinate Conventions", <https://www.ros.org/reps/rep-0103.html>                                                                                                                                                                     |
| REP 105            | W. Meeussen, "Coordinate Frames for Mobile Platforms", <https://www.ros.org/reps/rep-0105.html>                                                                                                                                                                                           |
| REP 120            | "Coordinate Frames for Humanoid Robots", <https://www.ros.org/reps/rep-0120.html> (the only REP that defines `base_footprint`)                                                                                                                                                            |
| kdl_parser         | <https://github.com/ros/kdl_parser>, branch `humble`, `kdl_parser/src/kdl_parser.cpp`. Used by robot_state_publisher.                                                                                                                                                                     |
| sdformat           | <https://github.com/gazebosim/sdformat>, `src/parser_urdf.cc` (URDF to SDF conversion used when Gazebo loads a URDF).                                                                                                                                                                     |
| Traversaro 2016    | S. Traversaro, S. Brossette, A. Escande, F. Nori, "Identification of fully physical consistent inertial parameters using optimization on manifolds", IEEE/RSJ IROS 2016.                                                                                                                  |
| Wensing 2018       | P. M. Wensing, S. Kim, J.-J. E. Slotine, "Linear Matrix Inequalities for Physically Consistent Inertial Parameter Identification: A Statistical Perspective on the Mass Distribution", IEEE Robotics and Automation Letters 3(1), 2018.                                                   |
| ros2_control       | ros2_control documentation, "Hardware description in URDF": <https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html>                                                                                                                     |
| mecanum-kinematics | <https://github.com/projectbin07-stack/mecanum-kinematics>, `docs/kinematics.md` (wheel order and sign conventions).                                                                                                                                                                      |

## Conventions used by the parser and the forward kinematics

From URDF-XML and urdfdom's `urdf.xsd`:

- Lengths in metres, angles in radians, masses in kilograms (REP 103).
- `<origin xyz rpy>` defaults to zero. `rpy` is roll about the parent x
  axis, then pitch about y, then yaw about z, all about **fixed** axes, so
  the rotation matrix is `R = Rz(yaw) * Ry(pitch) * Rx(roll)` (REP 103
  lists this "fixed axis roll, pitch, yaw" representation).
- A joint's origin is the child link frame in the parent link frame at
  joint position 0. The axis is expressed in the joint (= child) frame and
  defaults to `(1 0 0)` when `<axis>` is absent.
- The child frame at position `q` is `T(xyz, rpy) * M(q)`, where `M(q)` is a
  rotation by `q` about the normalised axis (revolute, continuous) or a
  translation by `q` along it (prismatic). Floating and planar joints are
  drawn at their origin; their extra coordinates are not modelled.
- `<mimic joint multiplier offset>`: position = multiplier * source +
  offset, multiplier defaults to 1 and offset to 0.
- `<limit>`: a missing `lower` or `upper` is 0 in urdfdom (it logs that it
  defaulted). Before URDF version 1.2, `effort` and `velocity` are required.
- Geometry: `box size` is the full side lengths; `cylinder` is along its
  local z axis, centred on its origin; `mesh scale` defaults to `1 1 1`.
- The `<inertial>` origin places the centre of mass, and its rpy orients the
  inertia tensor axes. urdfdom requires `<mass value>` and all six inertia
  attributes.

## Input

| id                    | severity | rule                                                                                                                                                                                                                                                                                         |
| --------------------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `xml-syntax`          | error    | The file must be well-formed XML. The CLI reports the line and column.                                                                                                                                                                                                                       |
| `not-urdf`            | error    | The root element must be `<robot>` (urdfdom `model.cpp`). An `<sdf>` root is called out because SDF is not supported.                                                                                                                                                                        |
| `xacro`               | error    | Any `xacro:` element, `${...}` expression or `$(...)` substitution in an attribute or text. Checking stops here: the file must be expanded with `xacro` first (`ros2 run xacro xacro robot.urdf.xacro > robot.urdf`). A `xmlns:xacro` declaration on its own is fine; xacro output keeps it. |
| `version-unsupported` | warning  | `<robot version>` other than 1.0, 1.1 or 1.2 (urdfdom `model.cpp` accepts 1.0 to 1.2).                                                                                                                                                                                                       |
| `missing-attribute`   | error    | Required names and elements (urdfdom): robot, link and joint names, joint `<parent link>` and `<child link>`, `<mass value>`, all six inertia terms, cylinder radius and length, sphere radius, mesh filename, mimic joint, top-level material name.                                         |
| `bad-number`          | error    | `xyz`, `rpy`, `axis`, `size`, `scale` need three finite numbers, `rgba` four; scalars must be finite numbers.                                                                                                                                                                                |
| `unknown-joint-type`  | error    | Type must be one of revolute, continuous, prismatic, fixed, floating, planar (URDF-XML joint page; urdfdom `joint.cpp` "has no known type").                                                                                                                                                 |
| `unknown-geometry`    | warning  | Geometry other than box, cylinder, sphere, mesh. Recent urdfdom also has `capsule`; this tool does not draw or weigh it.                                                                                                                                                                     |
| `missing-geometry`    | error    | `<visual>` or `<collision>` without `<geometry>` or without a shape in it (urdfdom `link.cpp`).                                                                                                                                                                                              |
| `unknown-element`     | info     | Top-level elements other than link, joint, material, transmission, gazebo, ros2_control.                                                                                                                                                                                                     |
| `undefined-material`  | warning  | A visual's `<material name>` without a colour that refers to no top-level material (urdfdom `link.cpp` logs "not defined in file").                                                                                                                                                          |
| `duplicate-material`  | error    | Two top-level materials with one name (urdfdom `model.cpp`: "material is not unique").                                                                                                                                                                                                       |

## Tree structure

URDF describes a tree: one root link, every other link the child of exactly
one joint (URDF-XML robot page; urdfdom `model.cpp` builds the tree and
finds the root).

| id                 | severity | rule                                                                                                                                               |
| ------------------ | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `no-links`         | error    | At least one `<link>`.                                                                                                                             |
| `duplicate-link`   | error    | Link names are unique.                                                                                                                             |
| `duplicate-joint`  | error    | Joint names are unique.                                                                                                                            |
| `missing-link`     | error    | A joint's parent or child names an existing link.                                                                                                  |
| `self-joint`       | error    | Parent and child differ.                                                                                                                           |
| `multiple-parents` | error    | No link is the child of two joints (closed chains cannot be written in URDF).                                                                      |
| `cycle`            | error    | Following parent joints from any link never returns to it, and some link has no parent.                                                            |
| `orphan-link`      | error    | Exactly one root. The root with the most descendants is taken as the main tree; every other parentless link is reported with its descendant count. |

## Frames (REP 103, REP 105, kdl_parser)

| id                      | severity | rule                                                                                                                                                                                                                                                                                                                                                                                                              |
| ----------------------- | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `no-base-link`          | warning  | REP 105: "The coordinate frame called `base_link` is rigidly attached to the mobile robot base." Navigation, localisation and many drivers default to it.                                                                                                                                                                                                                                                         |
| `footprint-relation`    | warning  | If both `base_footprint` and `base_link` exist, the joints between them (either may be the parent) must all be fixed, and `base_link`'s z axis must be within 0.001 rad of `base_footprint`'s (translation and yaw only). REP 105 does not define `base_footprint`; REP 120 defines it (for humanoids) as "the representation of the robot position on the floor", and mobile-robot packages use it the same way. |
| `footprint-height`      | warning  | With `base_footprint` and detected wheels, the lowest wheel contact point (wheel centre z minus radius, in `base_footprint`) must be within 5 mm of z = 0.                                                                                                                                                                                                                                                        |
| `odom-in-urdf`          | warning  | No link called `odom`, `map` or `earth`. REP 105: `odom` is computed "based on an odometry source", `map` by localisation, and the tree is `earth -> map -> odom -> base_link`, with each frame having one parent; the transforms are published by those nodes, not by robot_state_publisher.                                                                                                                     |
| `root-inertia`          | warning  | kdl_parser logs: "The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia. As a workaround, you can add an extra dummy link to your URDF."                                                                                                                                                                                                                    |
| `kdl-unsupported-joint` | warning  | kdl_parser's `toKdl` handles fixed, revolute, continuous and prismatic; every other type takes the default branch, "Converting unknown joint type of joint '%s' into a fixed joint".                                                                                                                                                                                                                              |
| `optical-frame`         | warning  | A link named `*_optical` or `*_optical_frame` must have, relative to its nearest non-optical ancestor, z along the ancestor's x, x along -y and y along -z (each within about 2.6 degrees; dot product > 0.999). REP 103: "a second frame defined with a '_optical' suffix ... z forward, x right, y down". The usual joint rpy is `-pi/2 0 -pi/2`.                                                               |
| `camera-no-optical`     | info     | A link whose name contains `camera` (and is not itself a sub-frame of another camera link) has no `*_optical(_frame)` descendant.                                                                                                                                                                                                                                                                                 |

## Joints

| id                    | severity | rule                                                                                                                                                                                           |
| --------------------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `axis-zero`           | error    | Movable and planar joints: axis length 0.                                                                                                                                                      |
| `axis-not-unit`       | warning  | Axis length differs from 1 by more than 0.001. URDF-XML asks for a normalised axis; this tool normalises it for forward kinematics, but not every consumer does.                               |
| `axis-ignored`        | info     | `<axis>` given on a fixed or floating joint.                                                                                                                                                   |
| `limit-missing`       | error    | urdfdom `joint.cpp`: "Joint [%s] is of type REVOLUTE but it does not specify limits" (and the same for PRISMATIC).                                                                             |
| `limit-attr-missing`  | error    | Before URDF 1.2 urdfdom rejects `<limit>` without `effort` or `velocity` ("limit has no effort"). With `version="1.2"` they default to infinity and the check is skipped.                      |
| `limit-order`         | error    | Revolute and prismatic: `lower > upper`. urdfdom 1.2 rejects it ("upper position limit ... cannot be smaller than lower position limit"); earlier versions accept a range nothing can satisfy. |
| `limit-equal`         | warning  | Revolute and prismatic: `lower == upper`, typically because both are missing and default to 0.                                                                                                 |
| `limit-zero`          | warning  | `effort="0"` or `velocity="0"` on a movable joint.                                                                                                                                             |
| `limit-negative`      | error    | Negative effort or velocity (urdfdom 1.2: "effort value ... is negative").                                                                                                                     |
| `limit-degrees`       | warning  | Revolute `                                                                                                                                                                                     | lower | `or` | upper | ` above 2*pi + 0.001 rad. |
| `continuous-limits`   | warning  | A continuous joint with `lower` or `upper`. Effort and velocity on a continuous joint are fine and not reported.                                                                               |
| `full-range-revolute` | info     | Revolute range `upper - lower >= 2*pi`.                                                                                                                                                        |
| `mimic-unknown`       | error    | The mimicked joint does not exist.                                                                                                                                                             |
| `mimic-self`          | error    | A joint mimics itself.                                                                                                                                                                         |
| `mimic-chain`         | warning  | The mimicked joint is itself a mimic joint, or is fixed. This tool resolves chains (and breaks cycles) for its own forward kinematics.                                                         |
| `mimic-zero`          | warning  | `multiplier="0"`.                                                                                                                                                                              |
| `dynamics-negative`   | error    | Negative `damping` or `friction`.                                                                                                                                                              |

## Mass and inertia

A rigid body's rotational inertia about its centre of mass is a symmetric
3x3 matrix whose eigenvalues (principal moments) `I1 <= I2 <= I3` satisfy

- `I1 > 0` (positive definite), and
- `I1 + I2 >= I3` (triangle inequality),

because each principal moment is a sum of the mass-weighted squared
distances along the other two principal axes, so `I1 + I2 - I3 = 2 *
sum(m z^2) >= 0` for the axis of `I3`. These are the physical-consistency
conditions used in inertial parameter identification (Traversaro 2016,
Wensing 2018). Principal moments are computed with cyclic Jacobi rotations
(`src/core/math.js`).

| id                    | severity | rule                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| --------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `inertial-missing`    | warning  | A link whose parent joint is revolute, continuous or prismatic has no `<inertial>`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `mass-nonpositive`    | error    | `mass <= 0`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| `inertia-not-pd`      | error    | Smallest principal moment `<= 1e-9 * I3`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `inertia-triangle`    | error    | `I1 + I2 < I3 - 1e-5 * I3`. The relative tolerance lets values rounded to six significant digits pass at equality (a thin plate has `I3 = I1 + I2` exactly).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| `inertia-implausible` | warning  | Only for links described by a single box, cylinder or sphere (the only collision shape, or the only visual shape when there is no collision). The solid-body principal moments for that shape and mass are compared with the given ones, sorted; if any ratio exceeds 10 or is below 0.1 the finding reports the factor and a reference tensor expressed in the `<inertial>` frame. The reference assumes the centre of mass at the shape's centre; links whose real mass distribution is far from uniform (a motor inside a light shell) can legitimately differ.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `inertia-axes`        | warning  | Same single-primitive links, when `inertia-implausible` did not fire. The reference is rotated by the shape's origin rpy into the link frame and then into the `<inertial>` origin's frame, so it and the declared tensor are compared in the `<inertial>` frame. Principal axes (unit eigenvectors, Jacobi) are paired by rank: smallest, middle and largest moment. An axis is compared only when its moment differs from each of the other two by at least 10% of the largest moment in both tensors (`AXIS_GAP`); degenerate axes (sphere, cube, a cylinder's two side axes, a cylinder with `h = sqrt(3) r`) are never compared, so they cannot false-alarm. The finding fires when a compared axis is more than 10 degrees from the reference axis (`AXIS_ANGLE_TOL_DEG`, sign ignored). Typical cause: a wheel cylinder rolled by `rpy="1.5708 0 0"` in `<visual>`/`<collision>` while `<inertial>` keeps `izz = m r^2 / 2`; the axis moment belongs on `iyy`. The message gives the declared and reference axes and the reference tensor in the `<inertial>` frame. Added in 0.1.1; 0.1.0 compared sorted moments only. |
| `density-implausible` | warning  | Same single-primitive links: `mass / volume` outside 10 to 25000 kg/m^3 (lighter than foam, denser than tungsten). Typical cause: grams entered as kilograms, or geometry in millimetres.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `gazebo-dropped`      | warning  | Mirrors sdformat's `CreateSDF`: a link with no `<inertial>` or `mass <= 0` is skipped ("not modeled in sdf") together with its child joints and links, unless fixed-joint lumping merges it. This tool computes each link's mass after lumping (own mass plus the lumped mass of children connected by fixed joints) and reports links that are the root or sit behind a non-fixed joint and still have no mass. A link named `world` is exempt, as in sdformat. Fixed joints that carry `<preserveFixedJoint>` or `<disableFixedJointLumping>` in a `<gazebo reference>` block are not lumped.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Reference inertias for solid primitives about their centroid, used by
`inertia-implausible` and `inertia-axes` and shown in the details panel (`src/core/inertia.js`):

| shape                                 | Ixx                  | Iyy                  | Izz                |
| ------------------------------------- | -------------------- | -------------------- | ------------------ |
| box `a x b x c`                       | m (b^2 + c^2) / 12   | m (a^2 + c^2) / 12   | m (a^2 + b^2) / 12 |
| cylinder radius r, length h (along z) | m (3 r^2 + h^2) / 12 | m (3 r^2 + h^2) / 12 | m r^2 / 2          |
| sphere radius r                       | 2 m r^2 / 5          | 2 m r^2 / 5          | 2 m r^2 / 5        |

Products of inertia are zero in the shape frame. If the shape's origin is
rotated relative to the `<inertial>` origin, the reference is rotated with
`I_F = R I_S R^T`, `R = R_F^T R_S`.

## Units

| id            | severity | rule                                                                                                                                |
| ------------- | -------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `units-mm`    | warning  | A box side, cylinder radius or length, or sphere radius above 10 m, or a joint or shape offset above 50 m. Reported once per model. |
| `rpy-degrees` | warning  | Any rpy component (joint, visual, collision or inertial origin) with magnitude above 2*pi + 0.001 rad.                              |

## Geometry and meshes

| id                     | severity        | rule                                                                                                                                                                                                                                                                                                |
| ---------------------- | --------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `geometry-nonpositive` | error           | A box, cylinder or sphere dimension `<= 0`.                                                                                                                                                                                                                                                         |
| `collision-missing`    | warning         | A link with `<visual>` but no `<collision>`. Gazebo and MoveIt use only collision geometry.                                                                                                                                                                                                         |
| `mesh-scheme`          | warning         | Mesh filename that is not `package://` or `file://`: relative or absolute paths, `http(s)://`, `model://` or another scheme. `package://` and `file://` are the forms resolved by RViz (through `resource_retriever`) and by Gazebo with the ROS integration. Meshes are never loaded by this tool. |
| `mesh-format`          | warning         | Extension other than `.stl`, `.dae`, `.obj`.                                                                                                                                                                                                                                                        |
| `mesh-missing`         | warning         | A mesh file that the caller could not find, with its reason. Only reported when the caller passes `meshExists` to `inspectXml` / `inspectText` (added in 0.2.0); the CLI does not look files up.                                                                                                    |
| `mesh-scale`           | error / warning | A zero scale component is an error; a negative one is a warning (mirrors the mesh).                                                                                                                                                                                                                 |

## ros2_control, Gazebo, transmissions

These blocks are recognised and summarised; nothing in them is executed or
validated against plugin parameters.

| id                           | severity | rule                                                                                                                                           |
| ---------------------------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `ros2-control-joint-unknown` | error    | A `<ros2_control>` `<joint name>` that is not a URDF joint. The controller manager matches hardware joints to the URDF by name (ros2_control). |
| `ros2-control-no-interface`  | warning  | A `<ros2_control>` joint with no `state_interface` (nothing published to `/joint_states`), or with no interface at all.                        |
| `ros2-control-summary`       | info     | Plugin, joints and their command and state interfaces, sensors.                                                                                |
| `gazebo-reference-unknown`   | warning  | `<gazebo reference>` that names neither a link nor a joint.                                                                                    |
| `gazebo-summary`             | info     | Number of `<gazebo>` blocks and the plugins they load.                                                                                         |
| `transmission-joint-unknown` | warning  | A `<transmission>` `<joint name>` that is not a URDF joint.                                                                                    |

## Wheels (differential and four-wheel bases)

Drive wheels are continuous joints whose joint or child link name contains
`wheel`, or whose child link has a cylinder shape; names containing
`caster` or `castor` are excluded. They are measured at the zero
configuration in `base_link` (or the root when there is no `base_link`):
wheel centre (the cylinder's or sphere's origin, else the link origin),
spin axis, radius (collision shape first, then visual).

Conventions follow REP 103 (x forward, y left, z up) and
mecanum-kinematics: wheels FL `(+lx, +ly)`, FR `(+lx, -ly)`, RL `(-lx,
+ly)`, RR `(-lx, -ly)` around the wheel centroid, and a positive wheel
speed drives the robot forward. For a wheel spinning about `+y`, a positive
joint velocity moves the contact point backwards and the robot forwards.

| id                      | severity | rule                                                                                                                                                                                        |
| ----------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `wheel-axis`            | warning  | Spin axis more than 5 degrees from the base y axis.                                                                                                                                         |
| `wheel-cylinder-axis`   | warning  | The wheel cylinder's z axis more than 5 degrees from the spin axis (usually a missing `rpy="1.5708 0 0"` on the shape).                                                                     |
| `wheel-direction-mixed` | warning  | Some lateral wheel axes point to `+y` and others to `-y`. diff_drive_controller and mecanum controllers command the same sign to every wheel to drive straight.                             |
| `wheel-radius-mismatch` | warning  | Radii differ by more than 1 % of the largest.                                                                                                                                               |
| `wheel-layout`          | warning  | Wheel centre heights differ by more than 2 mm. Two wheels: x differs, or the wheels are not centred on the base frame in y. Four wheels: not one per quadrant around the centroid, or the ` | x   | `or` | y   | ` distances to the centroid differ. Length tolerance: the larger of 2 mm and 2 % of the value. |
| `wheel-base-summary`    | info     | Number of wheels, radius, wheel separation (two wheels) or `lx`, `ly` (four wheels), and the offset of the wheel centroid from the base frame.                                              |

`urdf-inspector export-mecanum robot.urdf` writes these measurements as a
mecanum-kinematics `RobotConfig` TOML file: `wheel_radius`, `half_length`
(`lx`), `half_width` (`ly`), `roller_angle_deg = 45` (not in a URDF),
`max_wheel_speed` from the smallest wheel `<limit velocity>` when every
wheel has one, and `wheel_order = ["FL", "FR", "RL", "RR"]`, with comments
naming the URDF joint for each slot and flagging wheels whose axis is `-y`
(their commands and encoder counts must be negated). Encoder settings are
not in a URDF and are left to the user.
