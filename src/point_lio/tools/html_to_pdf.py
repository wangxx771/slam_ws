#!/usr/bin/env python3
"""Convert the test-report HTML to PDF.

The web version adapts to light and dark themes; print must be pinned to light.
This also adds paper size, headers and footers, and pagination rules so tables
and figures are never split across pages.

Usage: html_to_pdf.py <in.html> <out.pdf>
Requires: pip install weasyprint
"""

import sys

from weasyprint import HTML, CSS

PRINT_CSS = """
@page {
  size: A4;
  margin: 16mm 14mm 18mm;
  @bottom-center {
    content: counter(page) " / " counter(pages);
    font-size: 9pt;
    color: #6b7280;
  }
  @bottom-left {
    content: "Point-LIO Localization and Mapping Test Report";
    font-size: 8pt;
    color: #9aa1ab;
  }
}

/* Pin print to light: the web version's dark tokens print as a black background */
:root {
  --ground: #ffffff !important;
  --surface: #ffffff !important;
  --surface-2: #f3f5f8 !important;
  --ink: #11161d !important;
  --ink-2: #4b5563 !important;
  --ink-3: #8a939f !important;
  --rule: #d7dce3 !important;
  --accent: #1f5fa8 !important;
  --accent-soft: #eef4fc !important;
  --good: #0f7a43 !important;
  --good-soft: #e9f4ee !important;
  --bad: #b3302c !important;
  --bad-soft: #fbecea !important;
  --warn: #8a5f08 !important;
  --warn-soft: #f9f2e0 !important;
  --figure-bg: #ffffff !important;
}

body { font-size: 10pt; line-height: 1.6; background: #fff !important; }
.page { max-width: none; padding: 0; }
p, ul, .measure { max-width: none; }

header.masthead { padding-top: 0; margin-bottom: 18pt; }
header.masthead h1 { font-size: 22pt; }
.subtitle { font-size: 11pt; }

/* A table of contents is useless on paper; the anchors are not clickable */
nav.toc { display: none; }

section { margin-bottom: 22pt; }
.sec-head { margin-bottom: 10pt; padding-bottom: 6pt; }
.sec-head h2 { font-size: 14pt; }
h3 { font-size: 11.5pt; margin: 14pt 0 6pt; }

/* Pagination: no orphaned headings, and tables and figures stay whole */
section { break-inside: auto; }
.sec-head, h3 { break-after: avoid; }
figure, .note { break-inside: avoid; }
.tw { break-inside: auto; }
table { break-inside: auto; }
tr { break-inside: avoid; }
thead { display: table-header-group; }

figure { margin: 12pt 0 14pt; }
figure img { max-height: 165mm; object-fit: contain; }
figcaption { font-size: 8.5pt; }

/* Tables must fit the page width: the web version scrolls horizontally, paper does not */
.tw { overflow: visible; }
table { font-size: 8.5pt; table-layout: auto; width: 100%; }
th, td { padding: 4pt 5pt; word-break: break-word; }
thead th { white-space: normal; }
.cfg { white-space: pre; }
td.num, th.num { font-size: 8pt; }
thead th { font-size: 7.5pt; }

.cfg { font-size: 8.5pt; line-height: 1.6; }
.note { padding: 8pt 12pt; margin: 10pt 0; }
.chip { font-size: 8pt; padding: 2pt 6pt; }
footer { font-size: 8.5pt; margin-top: 14pt; }
"""


def main():
    src, dst = sys.argv[1], sys.argv[2]
    # UTF-8 must be declared explicitly: the report HTML is a body fragment for
    # Artifact with no <meta charset>, and WeasyPrint falls back to Latin-1,
    # mangling every non-ASCII character.
    html = open(src, encoding="utf-8").read()
    doc = ('<!doctype html><html><head><meta charset="utf-8">'
           "</head><body>" + html + "</body></html>")
    HTML(string=doc, base_url=src).write_pdf(dst, stylesheets=[CSS(string=PRINT_CSS)])
    print(f"wrote {dst}")


if __name__ == "__main__":
    main()
