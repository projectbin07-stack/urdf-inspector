# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-11-25

Library additions requested by Robot Studio (its upstream requests 2 to 6;
ecosystem audit findings 35 and 51). Existing results are unchanged: no
check that ran in 0.1.1 reports anything different.

### Added

- `exports` map in `package.json` naming the public core modules.
- `mesh-missing` warning (73 checks): `inspectXml` and `inspectText` take an
  optional `meshExists(filename)` callback that returns `true` or the reason
  the mesh was not found. Without it (the CLI) nothing is looked up.
- Optional `source` on `XmlElement` and on `Finding`: when the input elements
  carry the file they came from, each finding gets the file of its top-level
  element.
- `checkText(report, file, { location })` formats a finding's location
  (default `lineLocation`, the old `  (line N)`).
- `jointFrames(tree, q, poses?)` in `fk.js`: the root-frame pose of each
  joint frame.
- The CLI treats a lone `.urdf` / `.xml` file or path argument as `check`
  (`urdf-inspector robot.urdf`); other unknown words are still an unknown
  command.

## [0.1.1] - 2026-11-24

### Fixed

- The inertia plausibility check in 0.1.0 compared sorted principal moments
  only, so an inertia tensor that was not rotated together with its geometry
  passed without any finding. Example: a wheel cylinder (r = 0.04 m,
  h = 0.03 m, 1 kg) rolled with `rpy="1.5708 0 0"` in `<collision>` while
  `<inertial>` keeps `ixx = iyy = 0.00045, izz = 0.0008`; the axis moment
  belongs on `iyy`. The new `inertia-axes` warning compares principal axes
  in the `<inertial>` frame (reference rotated by the shape origin and the
  inertial origin rpy) and reports axes more than 10 degrees apart. Axes
  whose moment is within 10% of another moment (sphere, cube, a cylinder's
  side axes) are not compared.

### Added

- `symEigen` in `src/core/math.js`; `principalAxisMismatch`, `AXIS_GAP`
  and `AXIS_ANGLE_TOL_DEG` in `src/core/inertia.js`. Existing exports are
  unchanged.

## [0.1.0] - 2026-11-19

First release. The checks are tested on the bundled example models, minimal
fixtures and one public tutorial model; they have not been compared with
RViz or Gazebo across a wide set of real robot descriptions.

### Added

- XML reader for Node with line numbers, and a DOMParser adapter for the
  browser, producing the same element tree.
- URDF reader: links, joints of all six types, origins, axes, limits,
  dynamics, mimic, inertials, visual and collision box, cylinder, sphere and
  mesh geometry, materials, transmissions, `<ros2_control>` and `<gazebo>`
  blocks (summarised, not executed). Unexpanded xacro is detected and
  reported.
- 71 checks with severity, element path and explanation: input, tree
  structure, REP 103/105 frames, joints, mass and inertia (positive
  definiteness, triangle inequality, solid-primitive reference inertia,
  density), links Gazebo drops, units, geometry and meshes, ros2_control and
  Gazebo names, wheel consistency for differential and four-wheel bases.
  Documented with sources in `docs/checks.md`.
- Forward kinematics with mimic joints, per-link poses, total mass and
  centre of mass.
- Export of a mecanum-kinematics TOML config (wheel radius, lx, ly).
- CLI: `check` (text or JSON, `--strict`), `tree`, `fk`, `export-mecanum`,
  `checks`, with exit codes for CI.
- Browser app: file drop and paste, findings linked to elements,
  collapsible SVG kinematic tree, canvas 3D view of the primitives with
  orbit controls, frame axes, centre of mass and joint sliders, details
  panel, shareable links, light and dark themes, phone layout.
- Four example URDFs written for this project (mecanum base, two-joint arm,
  differential-drive base, deliberately broken robot).

[0.2.0]: https://github.com/projectbin07-stack/urdf-inspector/releases/tag/v0.2.0
[0.1.1]: https://github.com/projectbin07-stack/urdf-inspector/releases/tag/v0.1.1
[0.1.0]: https://github.com/projectbin07-stack/urdf-inspector/releases/tag/v0.1.0
