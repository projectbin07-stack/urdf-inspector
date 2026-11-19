# Security policy

## Supported versions

Only the latest release and the version deployed on GitHub Pages receive fixes.

## Reporting a vulnerability

Please do not open a public issue for a security problem. Use the private
vulnerability reporting of this repository on GitHub (Security tab, "Report a
vulnerability"):
https://github.com/projectbin07-stack/urdf-inspector/security/advisories/new

Include what you found, how to reproduce it and the version or commit. This is
a spare-time project maintained by one person, so there is no guaranteed
response time.

## Scope

The web page is static. Apart from its own files (HTML, CSS, JavaScript and
the bundled example URDFs) it makes no network requests and sends no data
anywhere. A URDF you drop, choose or paste is read inside the browser and is
never uploaded. Mesh files referenced by the URDF are not fetched. Names and
values from the model are inserted into the page as text nodes, never as
HTML. URL parameters are matched against fixed lists or a strict name
pattern, and the model itself is never put in the URL. The page stores
nothing; "Copy link" writes to the clipboard.

The CLI reads the URDF you name, writes a file only for
`export-mecanum --out <file>`, and opens no network connections. The XML
reader does not expand entities beyond the five predefined ones and numeric
character references, and rejects DTD internal subsets, so entity-expansion
attacks do not apply. `scripts/serve.js` is a development server bound to
127.0.0.1; do not expose it to a network.

## Not a safety component

A clean report means none of the documented checks fired, not that the model
is correct. The checks have been exercised on the bundled examples and test
fixtures only. Verify joint limits, masses and frames on the real robot.
