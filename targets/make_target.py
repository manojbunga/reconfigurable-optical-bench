#!/usr/bin/env python3
"""Generate the wall reference target: a ring of ArUco markers around the
edge of the reference wall, leaving the centre free for the object.

Outputs (per layout):
  wall_target_<layout>.pdf   exact-size vector PDF (print at 100% / "Actual size")
  wall_target_<layout>.svg   same geometry in mm, for a laser cutter
  wall_board_<layout>.json   marker IDs and corner positions in mm (used by bench.py)

Wall coordinate system (all outputs in mm):
  origin = top-left corner of the printed sheet, X to the right, Y downwards,
  Z = 0 on the wall surface.
"""
import json
import sys

import cv2
import numpy as np

DICT_NAME = "DICT_4X4_50"
BITS = 6  # 4x4 data bits + 1-bit black border on each side

LAYOUTS = {
    # A4 landscape, prints on any printer. Free centre ~221 x 134 mm.
    "a4": dict(page_w=297.0, page_h=210.0, margin=8.0, size=30.0, n_top=6, n_side=2),
    # Full 300 x 300 mm wall: print on A3 and trim, or laser-cut. Free centre ~212 x 212 mm.
    "wall300": dict(page_w=300.0, page_h=300.0, margin=8.0, size=36.0, n_top=6, n_side=4),
}


def marker_bits(dictionary, marker_id):
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, BITS, borderBits=1)
    return (img < 128)  # True = black cell


def layout_markers(page_w, page_h, margin, size, n_top, n_side):
    """Return list of (x, y) top-left positions in mm, clockwise from top-left."""
    x0, x1 = margin, page_w - margin - size
    y0, y1 = margin, page_h - margin - size
    xs = np.linspace(x0, x1, n_top)
    ys = np.linspace(y0, y1, n_side + 2)[1:-1]
    pos = [(x, y0) for x in xs]                 # top row, left -> right
    pos += [(x1, y) for y in ys]                # right column, top -> bottom
    pos += [(x, y1) for x in xs[::-1]]          # bottom row, right -> left
    pos += [(x0, y) for y in ys[::-1]]          # left column, bottom -> top
    return [(round(float(x), 3), round(float(y), 3)) for x, y in pos]


def build(name):
    L = LAYOUTS[name]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    positions = layout_markers(**L)
    s = L["size"]
    cell = s / BITS
    markers = []
    for mid, (x, y) in enumerate(positions):
        corners = [[x, y], [x + s, y], [x + s, y + s], [x, y + s]]  # TL, TR, BR, BL
        markers.append(dict(id=mid, top_left=[x, y], corners_mm=corners))

    # Scale-check bars: exactly 100.0 mm long, 4 mm thick, inside the free area
    # next to the ring. Measure their length with the vernier after printing.
    inner = L["margin"] + s + 6.0
    hbar = [inner, inner, 100.0, 4.0]          # x, y, w, h
    vbar = [inner, inner + 10.0, 4.0, 100.0]

    board = dict(
        layout=name, dictionary=DICT_NAME, marker_size_mm=s,
        page_w_mm=L["page_w"], page_h_mm=L["page_h"],
        scale_x=1.0, scale_y=1.0,
        scale_bars_mm=dict(horizontal=hbar, vertical=vbar, nominal_length=100.0),
        markers=markers,
    )
    with open(f"wall_board_{name}.json", "w") as f:
        json.dump(board, f, indent=2)

    rects = []  # black rectangles in mm: x, y, w, h
    for m in markers:
        bits = marker_bits(dictionary, m["id"])
        x, y = m["top_left"]
        for r in range(BITS):
            for c in range(BITS):
                if bits[r, c]:
                    rects.append((x + c * cell, y + r * cell, cell, cell))
    rects += [tuple(hbar), tuple(vbar)]

    write_svg(name, L, rects, markers, hbar, vbar)
    write_pdf(name, L, rects, markers, hbar, vbar)
    return board


def write_svg(name, L, rects, markers, hbar, vbar):
    W, H = L["page_w"], L["page_h"]
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" '
           f'viewBox="0 0 {W} {H}">',
           f'<rect x="0" y="0" width="{W}" height="{H}" fill="none" stroke="#ff0000" stroke-width="0.1"/>']
    for x, y, w, h in rects:
        out.append(f'<rect x="{x:.4f}" y="{y:.4f}" width="{w:.4f}" height="{h:.4f}" fill="#000"/>')
    out.append('</svg>')
    with open(f"wall_target_{name}.svg", "w") as f:
        f.write("\n".join(out))


def write_pdf(name, L, rects, markers, hbar, vbar):
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    W, H = L["page_w"], L["page_h"]
    c = canvas.Canvas(f"wall_target_{name}.pdf", pagesize=(W * mm, H * mm))
    c.setFillColorRGB(0, 0, 0)
    c.setStrokeColorRGB(0, 0, 0)
    for x, y, w, h in rects:
        # PDF origin is bottom-left; our Y runs downwards.
        c.rect(x * mm, (H - y - h) * mm, w * mm, h * mm, stroke=0, fill=1)
    c.setFont("Helvetica", 6)
    for m in markers:
        x, y = m["top_left"]
        c.drawString(x * mm, (H - y - L["size"] - 3.2) * mm, f"ID {m['id']}")
    c.drawString((hbar[0]) * mm, (H - hbar[1] - hbar[3] - 3) * mm,
                 "H bar: 100.0 mm - measure with vernier")
    c.saveState()
    c.translate((vbar[0] + vbar[2] + 3.5) * mm, (H - vbar[1] - vbar[3]) * mm)
    c.rotate(90)
    c.drawString(0, 0, "V bar: 100.0 mm - measure with vernier")
    c.restoreState()
    c.drawString((W / 2 - 25) * mm, (H / 2) * mm,
                 f"{name} | {DICT_NAME} | print at 100% (Actual size)")
    c.showPage()
    c.save()


if __name__ == "__main__":
    for n in (sys.argv[1:] or LAYOUTS):
        b = build(n)
        print(f"{n}: {len(b['markers'])} markers, size {b['marker_size_mm']} mm")
