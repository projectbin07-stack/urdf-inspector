# Contributing

Bug reports with a minimal URDF, corrections to the checks and their sources,
and pull requests are welcome.

## Setup

```bash
git clone https://github.com/projectbin07-stack/urdf-inspector.git
cd urdf-inspector
npm ci            # dev tools only: TypeScript (type checking), Prettier, Node types
npm run serve     # http://127.0.0.1:8000/
```

The site has no runtime dependencies and no build step: `index.html` loads
the ES modules in `src/` directly. Any static file server works.

## Checks

All of these must pass; CI runs them on Ubuntu and Windows with Node 22 and
24, plus a CLI smoke test and a headless Chrome smoke test of the page.

```bash
npm run format:check
npm run typecheck
npm run coverage      # node:test with line, branch and function thresholds
```

`npm run check` runs all three.

## Adding or changing a check

1. Add the identifier, group, severity, title and reason to `CHECKS` in
   `src/core/findings.js`.
2. Implement it in the matching module under `src/core/checks/`. Messages
   name the element and the numbers involved; paths use the
   `/robot/joint[@name='x']/limit` form.
3. Add a minimal failing and a minimal passing model to the table in
   `tests/checks.test.js` (the test fails until every identifier has both).
4. Document it in `docs/checks.md` with the source of the rule. Do not cite
   behaviour you have not read in the source or observed.

## Guidelines

- Keep the shipped page free of runtime dependencies and network calls
  (other than its own files). Models must never leave the browser. Dev tools
  go in `devDependencies` and must not be imported from `src/`.
- Keep computation out of `src/ui/app.js`, `src/ui/view3d.js` and
  `src/ui/treeview.js`. Everything that computes a number lives in
  `src/core/` (or `src/ui/state.js`) and has tests.
- Write the parser and checks from the specifications; do not copy code from
  urdfdom, urdf-loaders or other libraries.
- Example models in `examples/` are written for this project. Do not add
  models of commercial robots unless their licence allows it and the README
  says where they came from.
- Types are JSDoc comments checked by `tsc --checkJs` in strict mode.
- Small, focused commits with imperative messages.
