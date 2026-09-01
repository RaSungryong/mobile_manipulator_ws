#!/usr/bin/env python3
"""Render docs/GUIDE_kr.md to a paginated PDF.

    python3 docs/build_guide_pdf.py

Requires: markdown, weasyprint (pinned to pydyf==0.8.0 on Python 3.8), and the
Noto Sans CJK KR system font.
"""

import datetime
import re
import subprocess
import sys
from pathlib import Path

import markdown
import weasyprint

DOCS_DIR = Path(__file__).resolve().parent
SOURCE = DOCS_DIR / "GUIDE_kr.md"
OUTPUT = DOCS_DIR / "mobile_manipulator_guide_kr.pdf"

TITLE = "Mobile Manipulator 사용 가이드"
SUBTITLE = "Fairino FR10v6 + Navifra KU Polishing Robot"

STYLE = """
@page {
    size: A4;
    margin: 22mm 18mm 20mm 18mm;
    @top-right {
        content: "Mobile Manipulator 사용 가이드";
        font-family: "Noto Sans CJK KR", sans-serif;
        font-size: 7.5pt; color: #8a8a8a;
    }
    @bottom-center {
        content: counter(page) " / " counter(pages);
        font-family: "Noto Sans CJK KR", sans-serif;
        font-size: 8pt; color: #8a8a8a;
    }
}
@page :first { margin: 0; @top-right { content: none } @bottom-center { content: none } }

body {
    font-family: "Noto Sans CJK KR", sans-serif;
    font-size: 9.5pt; line-height: 1.65; color: #1c1c1c;
    hyphens: none;
}

/* --- cover ------------------------------------------------------------ */
.cover { height: 297mm; padding: 62mm 24mm 0 24mm; page-break-after: always; }
.cover .rule { border-top: 3px solid #1a3a5c; width: 42mm; margin-bottom: 9mm; }
.cover h1 { font-size: 27pt; color: #1a3a5c; margin: 0 0 5mm 0;
            border: none; padding: 0; line-height: 1.3;
            page-break-before: avoid; }
.cover .sub { font-size: 12pt; color: #4a5a6a; margin-bottom: 26mm; }
.cover .meta { font-size: 9pt; color: #666; line-height: 2.0; }
.cover .meta b { color: #1c1c1c; font-weight: 600; display: inline-block; min-width: 26mm; }

/* --- table of contents ------------------------------------------------ */
.toc { page-break-after: always; }
.toc h2 { font-size: 15pt; color: #1a3a5c; border-bottom: 2px solid #1a3a5c;
          padding-bottom: 2mm; margin: 0 0 6mm 0; }
.toc ul { list-style: none; padding-left: 0; margin: 0; }
.toc li { margin: 0; }
.toc a { text-decoration: none; color: #1c1c1c; }
.toc a::after { content: " " leader('.') " " target-counter(attr(href), page); color: #999; }
.toc > ul > li > a { font-weight: 600; font-size: 10pt; }
.toc > ul > li { margin-top: 3.2mm; }
.toc ul ul { padding-left: 6mm; }
.toc ul ul a { font-size: 9pt; color: #444; }

/* --- headings --------------------------------------------------------- */
h1 { font-size: 16pt; color: #1a3a5c; border-bottom: 2px solid #1a3a5c;
     padding-bottom: 1.8mm; margin: 0 0 5mm 0; page-break-before: always;
     page-break-after: avoid; }
h2 { font-size: 12pt; color: #24507a; margin: 7mm 0 2.5mm 0; page-break-after: avoid; }
h3 { font-size: 10.5pt; color: #2f2f2f; margin: 5mm 0 2mm 0; page-break-after: avoid; }
p  { margin: 0 0 2.6mm 0; }

/* --- tables ----------------------------------------------------------- */
table { border-collapse: collapse; width: 100%; margin: 3mm 0 4mm 0;
        font-size: 8.5pt; page-break-inside: avoid; }
th, td { border: 1px solid #c4ccd4; padding: 1.6mm 2.4mm;
         text-align: left; vertical-align: top; }
th { background: #eaf0f6; color: #1a3a5c; font-weight: 600; }
tr:nth-child(even) td { background: #fafbfc; }

/* --- code ------------------------------------------------------------- */
code { font-family: "DejaVu Sans Mono", monospace; font-size: 8pt;
       background: #f1f3f5; padding: 0.3mm 1mm; border-radius: 1.5px;
       color: #21456b; }
pre { background: #f7f8fa; border: 1px solid #e0e4e8; border-left: 3px solid #7a90a8;
      padding: 2.6mm 3.4mm; margin: 2.6mm 0 4mm 0; page-break-inside: avoid;
      white-space: pre-wrap; word-wrap: break-word; }
pre code { background: none; padding: 0; font-size: 8pt; color: #24303c; line-height: 1.5; }

/* --- callouts --------------------------------------------------------- */
blockquote { border-left: 3.5px solid #c0392b; background: #fdf4f3;
             padding: 2.4mm 3.4mm; margin: 3mm 0 4mm 0; page-break-inside: avoid; }
blockquote p { margin: 0 0 1.6mm 0; font-size: 9pt; }
blockquote p:last-child { margin-bottom: 0; }

ul, ol { margin: 0 0 3mm 0; padding-left: 6mm; }
li { margin-bottom: 1.1mm; }
hr { display: none; }
strong { font-weight: 600; }
"""


def build_toc(html):
    """Build a two-level TOC from the rendered body, keyed on heading ids."""
    items = re.findall(r'<h([12]) id="([^"]+)">(.*?)</h[12]>', html, re.S)
    if not items:
        return ""
    out, open_sub = ['<div class="toc"><h2>목차</h2><ul>'], False
    for level, hid, text in items:
        label = re.sub(r"<[^>]+>", "", text).strip()
        if level == "1":
            if open_sub:
                out.append("</ul></li>")
                open_sub = False
            out.append(f'<li><a href="#{hid}">{label}</a>')
            out.append("<ul>")
            open_sub = True
        else:
            out.append(f'<li><a href="#{hid}">{label}</a></li>')
    if open_sub:
        out.append("</ul></li>")
    out.append("</ul></div>")
    return "".join(out)


def source_revision():
    try:
        rev = subprocess.run(
            ["git", "-C", str(DOCS_DIR), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=10,
        )
        if rev.returncode == 0:
            return rev.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return "unversioned"


def main():
    if not SOURCE.exists():
        sys.exit(f"source not found: {SOURCE}")

    body = markdown.markdown(
        SOURCE.read_text(encoding="utf-8"),
        extensions=["tables", "fenced_code", "toc", "sane_lists"],
    )

    cover = f"""<div class="cover">
      <div class="rule"></div>
      <h1>{TITLE}</h1>
      <div class="sub">{SUBTITLE}</div>
      <div class="meta">
        <div><b>패키지</b> apriltag_nav (ROS Noetic)</div>
        <div><b>브랜치</b> real</div>
        <div><b>소스</b> docs/GUIDE_kr.md @ {source_revision()}</div>
        <div><b>생성일</b> {datetime.date.today().isoformat()}</div>
      </div>
    </div>"""

    html = (
        '<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8">'
        f"<title>{TITLE}</title></head><body>"
        + cover + build_toc(body) + body + "</body></html>"
    )

    weasyprint.HTML(string=html, base_url=str(DOCS_DIR)).write_pdf(
        OUTPUT, stylesheets=[weasyprint.CSS(string=STYLE)]
    )
    print(f"{OUTPUT}  ({OUTPUT.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
