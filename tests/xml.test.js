import assert from "node:assert/strict";
import { test } from "node:test";
import { fromDomElement, parseXml, XmlError } from "../src/core/xml.js";

test("elements, attributes, text, nesting and line numbers", () => {
  const root = parseXml(
    '<?xml version="1.0" encoding="utf-8"?>\n<!-- c -->\n<robot name="r">\n  <link name=\'a\'/>\n  <x k="1">hi<y/>there</x>\n</robot>\n',
  );
  assert.equal(root.name, "robot");
  assert.deepEqual(root.attrs, { name: "r" });
  assert.equal(root.line, 3);
  assert.equal(root.children.length, 2);
  assert.equal(root.children[0].attrs.name, "a");
  assert.equal(root.children[0].line, 4);
  assert.equal(root.children[1].text, "hithere");
  assert.equal(root.children[1].children[0].name, "y");
  assert.equal(root.children[1].line, 5);
});

test("entities, character references, CDATA, comments and PIs inside elements", () => {
  const root = parseXml(
    '<a t="&lt;&gt;&amp;&quot;&apos;&#65;&#x42;">x &amp; y<!-- skip --><![CDATA[<raw & text>]]><?pi data?></a>',
  );
  assert.equal(root.attrs.t, "<>&\"'AB");
  assert.equal(root.text, "x & y<raw & text>");
});

test("BOM, DOCTYPE without subset and namespace prefixes", () => {
  const root = parseXml(
    String.fromCharCode(0xfeff) +
      '<!DOCTYPE robot>\n<robot xmlns:xacro="u"><xacro:macro name="m"/></robot>',
  );
  assert.equal(root.children[0].name, "xacro:macro");
  assert.equal(root.attrs["xmlns:xacro"], "u");
});

/** @param {string} src @param {RegExp} re @param {number} [line] */
const fails = (src, re, line) => {
  assert.throws(
    () => parseXml(src),
    (e) => {
      assert.ok(e instanceof XmlError);
      assert.match(e.message, re);
      if (line !== undefined) assert.equal(e.line, line);
      return true;
    },
  );
};

test("syntax errors carry a line number and a reason", () => {
  fails("<a>\n<b>\n</a>", /<\/a> does not match <b> from line 2/, 3);
  fails("<a>", /<a> \(line 1\) is not closed/);
  fails('<a x="1" x="2"/>', /duplicate attribute x/);
  fails("<a x=1/>", /must be quoted/);
  fails('<a x="1"y="2"/>', /expected whitespace/);
  fails("<a x/>", /has no value/);
  fails('<a x="<"/>', /'<' inside the value/);
  fails('<a x="1/>', /unterminated value/);
  fails("<a>&nope;</a>", /unknown entity/);
  fails("<a>a & b</a>", /bare '&'/);
  fails("<a>&#xFFFFFFF;</a>", /bad character reference/);
  fails("<a/><b/>", /content after the root element/);
  fails("", /no root element/);
  fails("just text", /no root element/);
  fails("<a><!-- open", /unterminated comment/);
  fails("<a><![CDATA[ open", /unterminated CDATA/);
  fails("<a></a ", /expected '>'/);
  fails('<!DOCTYPE a [<!ENTITY x "y">]><a/>', /internal subset/);
  fails("<1a/>", /expected a name/);
});

test("fromDomElement converts a DOM-like tree", () => {
  /** @param {string} tagName @param {Record<string, string>} attrs @param {any[]} kids */
  const el = (tagName, attrs, kids) => ({
    nodeType: 1,
    tagName,
    attributes: Object.entries(attrs).map(([name, value]) => ({ name, value })),
    childNodes: kids,
  });
  const text = (/** @type {string} */ v) => ({ nodeType: 3, nodeValue: v });
  const dom = el("robot", { name: "r" }, [
    text("\n "),
    el("link", { name: "a" }, []),
    { nodeType: 8, nodeValue: "comment" },
    { nodeType: 4, nodeValue: "cdata" },
    { nodeType: 3, nodeValue: null },
  ]);
  const x = fromDomElement(dom);
  assert.equal(x.name, "robot");
  assert.deepEqual(x.attrs, { name: "r" });
  assert.equal(x.children.length, 1);
  assert.equal(x.children[0].attrs.name, "a");
  assert.equal(x.text, "\n cdata");
  assert.equal(x.line, 0);
});
