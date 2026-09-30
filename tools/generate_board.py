#!/usr/bin/env python3
"""Generate a print-ready ChArUco board PDF at exact physical scale.

Usage:
    python tools/generate_board.py [paper] [output.pdf]

paper is letter, tabloid, a4, a3 or WIDTHxHEIGHT in mm (default tabloid).
The layout (grid, dictionary, marker to square ratio) comes from config/board.yaml,
and the squares are made as big as the sheet allows.
Print at 100% (actual size), check the 100 mm line, then measure a square and a marker
and put those lengths in config/board.yaml.
"""

import sys

from PIL import Image, ImageDraw, ImageFont

from board import load_board

PAPERS_MM = {"letter": (279.4, 215.9), "tabloid": (431.8, 279.4), "a4": (297.0, 210.0), "a3": (420.0, 297.0)}
MARGIN_MM = 12   # printers leave the edge of the sheet blank
LABEL_MM = 16    # strip under the board for the scale line and the text
DPI = 600


def mm_to_px(mm):
    return mm / 25.4 * DPI


def main():
    paper = sys.argv[1] if len(sys.argv) > 1 else "tabloid"
    output = sys.argv[2] if len(sys.argv) > 2 else f"charuco_{paper}.pdf"
    if paper in PAPERS_MM:
        page_w, page_h = PAPERS_MM[paper]
    else:
        page_w, page_h = (float(v) for v in paper.lower().split("x"))
    page_w, page_h = max(page_w, page_h), min(page_w, page_h)   # landscape

    board, _, s = load_board()
    sx, sy = s["squares_x"], s["squares_y"]
    ratio = s["marker_mm"] / s["square_mm"]

    # biggest whole-pixel square that fits inside the margins
    area_w = mm_to_px(page_w - 2 * MARGIN_MM)
    area_h = mm_to_px(page_h - 2 * MARGIN_MM - LABEL_MM)
    sq_px = int(min(area_w / sx, area_h / sy))
    square_mm = sq_px / DPI * 25.4
    img = board.generateImage((sx * sq_px, sy * sq_px), marginSize=0, borderBits=1)

    page = Image.new("L", (round(mm_to_px(page_w)), round(mm_to_px(page_h))), 255)
    x0 = (page.width - img.shape[1]) // 2
    y0 = round(mm_to_px(MARGIN_MM))
    page.paste(Image.fromarray(img), (x0, y0))

    # a 100 mm line to check the printer did not scale the page
    draw = ImageDraw.Draw(page)
    ly = y0 + img.shape[0] + round(mm_to_px(5))
    tick = round(mm_to_px(2))
    x1 = x0 + round(mm_to_px(100))
    draw.line([(x0, ly), (x1, ly)], fill=0, width=round(mm_to_px(0.4)))
    for x in (x0, x1):
        draw.line([(x, ly - tick), (x, ly + tick)], fill=0, width=round(mm_to_px(0.4)))
    font = ImageFont.load_default(size=round(mm_to_px(3)))
    text = (f"{sx} x {sy} ChArUco, {s['dictionary']}, squares {square_mm:.2f} mm, markers {square_mm * ratio:.2f} mm. "
            f"Print at 100% (actual size). The line on the left should measure 100 mm.")
    draw.text((x1 + round(mm_to_px(5)), ly - round(mm_to_px(1.8))), text, fill=0, font=font)

    # one bit per pixel keeps the edges sharp in the pdf
    page.point(lambda v: 255 if v > 127 else 0).convert("1").save(output, resolution=DPI)
    print(f"Saved {output}: {page_w:.1f} x {page_h:.1f} mm page, {sx} x {sy} squares of {square_mm:.2f} mm, "
          f"markers {square_mm * ratio:.2f} mm")


if __name__ == "__main__":
    main()
