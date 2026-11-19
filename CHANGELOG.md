# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

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

[0.1.0]: https://github.com/projectbin07-stack/urdf-inspector/releases/tag/v0.1.0
