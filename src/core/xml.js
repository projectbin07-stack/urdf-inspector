// A small, strict-enough XML reader for URDF files under Node (CLI and
// tests). The browser app uses DOMParser instead and converts its result
// with fromDomElement(), so both paths produce the same XmlElement tree.
//
// Supported: elements, attributes in single or double quotes, the five
// predefined entities and numeric character references, comments, CDATA,
// processing instructions and the XML declaration, and a DOCTYPE without an
// internal subset. Not supported: DTD internal subsets and custom entities
// (URDF files do not use them). Namespace prefixes are kept as part of the
// name ("xacro:macro"), which is what the xacro detector needs.

/**
 * @typedef {object} XmlElement
 * @property {string} name tag name, including any prefix
 * @property {Record<string, string>} attrs attribute values, entities decoded
 * @property {XmlElement[]} children child elements in document order
 * @property {string} text concatenated character data directly inside this element
 * @property {number} line 1-based line of the start tag, 0 when unknown (DOMParser)
 */

export class XmlError extends Error {
  /**
   * @param {string} message
   * @param {number} line
   * @param {number} col
   */
  constructor(message, line, col) {
    super(`line ${line}, column ${col}: ${message}`);
    this.line = line;
    this.col = col;
  }
}

const NAME_START = /[A-Za-z_:À-￿]/;
const NAME_CHAR = /[A-Za-z0-9_:.\-·À-￿]/;

/** @type {Record<string, string>} */
const ENTITIES = { lt: "<", gt: ">", amp: "&", quot: '"', apos: "'" };

/**
 * Parse an XML document and return its root element.
 * @param {string} src
 * @returns {XmlElement}
 */
export function parseXml(src) {
  let i = 0;
  let line = 1;
  let lineStart = 0;
  const n = src.length;
  if (src.charCodeAt(0) === 0xfeff) i = 1;

  /** @param {string} msg */
  const fail = (msg) => {
    throw new XmlError(msg, line, i - lineStart + 1);
  };

  /** Advance to position `to`, counting lines. @param {number} to */
  const advance = (to) => {
    for (; i < to; i++) {
      if (src.charCodeAt(i) === 10) {
        line++;
        lineStart = i + 1;
      }
    }
  };

  /** @param {string} s */
  const startsWith = (s) => src.startsWith(s, i);

  /** @param {string} end @param {string} what */
  const skipPast = (end, what) => {
    const j = src.indexOf(end, i);
    if (j < 0) fail(`unterminated ${what}`);
    advance(j + end.length);
  };

  const skipSpace = () => {
    while (i < n && /\s/.test(src[i])) advance(i + 1);
  };

  const readName = () => {
    if (i >= n || !NAME_START.test(src[i])) fail("expected a name");
    const s = i;
    while (i < n && NAME_CHAR.test(src[i])) i++;
    return src.slice(s, i);
  };

  /** @param {string} raw */
  const decode = (raw) =>
    raw.replace(/&(#x[0-9A-Fa-f]+|#[0-9]+|[A-Za-z]+);|&/g, (m, ent) => {
      if (ent === undefined) fail("bare '&' (write &amp;)");
      if (ent[0] === "#") {
        const code = ent[1] === "x" ? parseInt(ent.slice(2), 16) : parseInt(ent.slice(1), 10);
        if (!Number.isFinite(code) || code > 0x10ffff) fail(`bad character reference ${m}`);
        return String.fromCodePoint(code);
      }
      const v = ENTITIES[ent];
      if (v === undefined) fail(`unknown entity ${m}`);
      return v;
    });

  /** Skip comments, PIs and whitespace (and a DOCTYPE in the prolog). @param {boolean} prolog */
  const skipMisc = (prolog) => {
    for (;;) {
      skipSpace();
      if (startsWith("<!--")) skipPast("-->", "comment");
      else if (startsWith("<?")) skipPast("?>", "processing instruction");
      else if (prolog && startsWith("<!DOCTYPE")) {
        if (src.indexOf("[", i) >= 0 && src.indexOf("[", i) < src.indexOf(">", i))
          fail("DOCTYPE with an internal subset is not supported");
        skipPast(">", "DOCTYPE");
      } else return;
    }
  };

  /** @returns {XmlElement} */
  const readElement = () => {
    const elLine = line;
    advance(i + 1); // '<'
    const name = readName();
    /** @type {Record<string, string>} */
    const attrs = {};
    for (;;) {
      const before = i;
      skipSpace();
      if (startsWith("/>")) {
        advance(i + 2);
        return { name, attrs, children: [], text: "", line: elLine };
      }
      if (startsWith(">")) {
        advance(i + 1);
        break;
      }
      if (i === before) fail(`expected whitespace, '>' or '/>' in <${name}>`);
      const key = readName();
      skipSpace();
      if (src[i] !== "=") fail(`attribute ${key} has no value`);
      advance(i + 1);
      skipSpace();
      const q = src[i];
      if (q !== '"' && q !== "'") fail(`attribute ${key} value must be quoted`);
      const end = src.indexOf(q, i + 1);
      if (end < 0) fail(`unterminated value of attribute ${key}`);
      const raw = src.slice(i + 1, end);
      if (raw.includes("<")) fail(`'<' inside the value of attribute ${key}`);
      if (Object.hasOwn(attrs, key)) fail(`duplicate attribute ${key} on <${name}>`);
      attrs[key] = decode(raw);
      advance(end + 1);
    }
    /** @type {XmlElement[]} */
    const children = [];
    let text = "";
    for (;;) {
      if (i >= n) fail(`<${name}> (line ${elLine}) is not closed`);
      if (startsWith("</")) {
        advance(i + 2);
        const close = readName();
        if (close !== name) fail(`</${close}> does not match <${name}> from line ${elLine}`);
        skipSpace();
        if (src[i] !== ">") fail(`expected '>' after </${close}`);
        advance(i + 1);
        return { name, attrs, children, text, line: elLine };
      }
      if (startsWith("<!--")) skipPast("-->", "comment");
      else if (startsWith("<![CDATA[")) {
        const end = src.indexOf("]]>", i);
        if (end < 0) fail("unterminated CDATA section");
        text += src.slice(i + 9, end);
        advance(end + 3);
      } else if (startsWith("<?")) skipPast("?>", "processing instruction");
      else if (src[i] === "<") children.push(readElement());
      else {
        let end = src.indexOf("<", i);
        if (end < 0) end = n;
        text += decode(src.slice(i, end));
        advance(end);
      }
    }
  };

  skipMisc(true);
  if (i >= n || src[i] !== "<") fail("no root element");
  const root = readElement();
  skipMisc(false);
  if (i < n) fail("content after the root element");
  return root;
}

/**
 * The parts of a DOM Element that fromDomElement() reads. Declared
 * structurally so the conversion can be tested without a browser.
 * @typedef {object} DomLike
 * @property {string} tagName
 * @property {ArrayLike<{ name: string, value: string }>} attributes
 * @property {ArrayLike<DomNodeLike>} childNodes
 */
/**
 * @typedef {object} DomNodeLike
 * @property {number} nodeType 1 element, 3 text, 4 CDATA
 * @property {string | null} [nodeValue]
 * @property {string} [tagName]
 * @property {ArrayLike<{ name: string, value: string }>} [attributes]
 * @property {ArrayLike<DomNodeLike>} [childNodes]
 */

/**
 * Convert a DOM element (from DOMParser) to the XmlElement tree used by the
 * core. Line numbers are not available from the DOM and are set to 0.
 * @param {DomLike} el
 * @returns {XmlElement}
 */
export function fromDomElement(el) {
  /** @type {Record<string, string>} */
  const attrs = {};
  for (let k = 0; k < el.attributes.length; k++) {
    const a = el.attributes[k];
    attrs[a.name] = a.value;
  }
  /** @type {XmlElement[]} */
  const children = [];
  let text = "";
  for (let k = 0; k < el.childNodes.length; k++) {
    const c = el.childNodes[k];
    if (c.nodeType === 1) children.push(fromDomElement(/** @type {DomLike} */ (c)));
    else if (c.nodeType === 3 || c.nodeType === 4) text += c.nodeValue ?? "";
  }
  return { name: el.tagName, attrs, children, text, line: 0 };
}
