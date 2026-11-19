// The documentation must describe every check, and the README examples
// must stay in step with the code.

import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import { CHECKS } from "../src/core/findings.js";

/** @param {string} p */
const read = (p) => readFileSync(new URL(`../${p}`, import.meta.url), "utf8");

test("docs/checks.md and the README describe every check", () => {
  const docs = read("docs/checks.md");
  const readme = read("README.md");
  for (const id of Object.keys(CHECKS)) {
    assert.ok(new RegExp(`\\| \`${id}\` +\\|`).test(docs), `docs/checks.md is missing ${id}`);
    assert.ok(readme.includes(`| \`${id}\` `), `README is missing ${id}`);
  }
  assert.ok(readme.includes(`**${Object.keys(CHECKS).length} checks**`));
});
