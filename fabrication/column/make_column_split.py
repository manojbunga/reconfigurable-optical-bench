#!/usr/bin/env python3
"""Column in 2 pieces (0-250 mm and 250-600 mm) + splice plate, for small laser beds.
Same switch slots, tabs and scale as the one-piece column. Join: butt the pieces,
splice plate on the BACK, 4 x M3 bolts + nuts.
Outputs column_split.svg / .dxf / .plt / _preview.png (red = cut, blue = engrave).
"""
import math
import cairosvg
import ezdxf
from ezdxf.addons import text2path
from ezdxf import path as zpath

T = 5.0
W = 50.0
SPLIT = 250.0
TOP = 600.0
SW_SPACING, SW_SLOT_W = 6.5, 2.3
HOLE_R = 2.15                     # M4 clearance (4.3 mm)
JOIN_HOLES = [220, 235, 265, 280]  # bolt heights (mm above base plate)
HOLE_X = [18.0, 32.0]   # symmetric; M4 heads clear of the scale numbers
c = W / 2

cut, eng, txt = [], [], []


def rect(x, y, w, h, ox, oy):
    cut.append([(ox + x, oy + y), (ox + x + w, oy + y), (ox + x + w, oy + y + h), (ox + x, oy + y + h)])


def circle(cx, cy, r, ox, oy, n=36):
    cut.append([(ox + cx + r * math.cos(2 * math.pi * k / n), oy + cy + r * math.sin(2 * math.pi * k / n))
                for k in range(n)])


def piece(ox, oy, h0, h1, tabs):
    """Column piece covering heights h0..h1. Drawing y = (h1 - h)."""
    L = h1 - h0
    if tabs:
        cut.append([(ox + x, oy + y) for x, y in [
            (0, 0), (W, 0), (W, L), (c + 13, L), (c + 13, L + T), (c + 5, L + T), (c + 5, L),
            (c - 5, L), (c - 5, L + T), (c - 13, L + T), (c - 13, L), (0, L)]])
    else:
        rect(0, 0, W, L, ox, oy)
    for lo, hi in [(70, 140), (300, 370)]:
        lo2, hi2 = max(lo, h0), min(hi, h1)
        if lo2 < hi2:
            for xs in (c - SW_SPACING / 2, c + SW_SPACING / 2):
                rect(xs - SW_SLOT_W / 2, h1 - hi2, SW_SLOT_W, hi2 - lo2, ox, oy)
    for hh in JOIN_HOLES:
        if h0 < hh < h1:
            for hx in HOLE_X:
                circle(hx, h1 - hh, HOLE_R, ox, oy)
    for h in range(int(h0), int(h1) + 1):
        ln = 6 if h % 10 == 0 else 4 if h % 5 == 0 else 2
        eng.append([(ox, oy + h1 - h), (ox + ln, oy + h1 - h)])
        if h % 10 == 0 and h0 + 3 < h < h1 - 3:
            txt.append((ox + 6.8, oy + h1 - h + 1, 2.4, str(h)))


def label(ox, oy, h1, h, s):
    txt.append((ox + 29, oy + h1 - h, 2.2, s))


# lower piece 0-250 (with tabs), upper piece 250-600
piece(5, 2, 0, SPLIT, True)
label(5, 2, SPLIT, 62, "BOTTOM SW")
label(5, 2, SPLIT, 57, "lever up")
txt.append((5 + 20, 2 + SPLIT - 245 + 0.5, 2.2, "^ UP ^"))
piece(65, 2, SPLIT, TOP, False)
label(65, 2, TOP, 292, "TOP SW")
label(65, 2, TOP, 287, "lever down")
txt.append((65 + 20, 2 + TOP - 255 + 2, 2.2, "v JOIN v"))

# splice plate 50 x 80 (covers 210-290), bolts on the back of the column
SX, SY = 125, 2
rect(0, 0, W, 80, SX, SY)
for hh in JOIN_HOLES:
    for hx in HOLE_X:
        circle(hx, 290 - hh, HOLE_R, SX, SY)
txt.append((SX + 8, SY + 42, 2.6, "SPLICE (back)"))

W_ALL, H_ALL = 180, 360

out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W_ALL}mm" height="{H_ALL}mm" viewBox="0 0 {W_ALL} {H_ALL}">',
       f'<rect width="{W_ALL}" height="{H_ALL}" fill="white"/>']
for p in cut:
    out.append('<polygon points="' + " ".join(f"{x:.3f},{y:.3f}" for x, y in p) +
               '" fill="none" stroke="#FF0000" stroke-width="0.1"/>')
for p in eng:
    out.append(f'<line x1="{p[0][0]:.3f}" y1="{p[0][1]:.3f}" x2="{p[1][0]:.3f}" y2="{p[1][1]:.3f}" '
               'stroke="#0000FF" stroke-width="0.1"/>')
for x, y, s, t in txt:
    out.append(f'<text x="{x:.2f}" y="{y:.2f}" font-family="Arial" font-size="{s}" fill="#0000FF">{t}</text>')
out.append("</svg>")
open("column_split.svg", "w").write("\n".join(out))

doc = ezdxf.new("R2010", setup=True)
doc.units = ezdxf.units.MM
doc.header["$INSUNITS"] = 4
doc.layers.add("CUT", color=1)
doc.layers.add("ENGRAVE", color=5)
msp = doc.modelspace()
for p in cut:
    msp.add_lwpolyline([(x, H_ALL - y) for x, y in p], close=True, dxfattribs={"layer": "CUT"})
for p in eng:
    msp.add_line((p[0][0], H_ALL - p[0][1]), (p[1][0], H_ALL - p[1][1]), dxfattribs={"layer": "ENGRAVE"})
for x, y, s, t in txt:
    msp.add_text(t, height=s, dxfattribs={"layer": "ENGRAVE"}).set_placement((x, H_ALL - y))
doc.saveas("column_split.dxf")

# PLT (40 units/mm), text as outlines
strokes = {1: [], 2: []}
for e in msp:
    pen = 1 if e.dxf.layer == "CUT" else 2
    if e.dxftype() == "TEXT":
        for p in text2path.make_paths_from_entity(e):
            for sub in p.sub_paths():
                pts = list(sub.flattening(0.02))
                if len(pts) > 1:
                    strokes[pen].append([(v.x, v.y) for v in pts])
    else:
        strokes[pen].append([(v.x, v.y) for v in zpath.make_path(e).flattening(0.02)])
lines = ["IN;"]
for pen in (1, 2):
    lines.append(f"SP{pen};")
    for s in strokes[pen]:
        lines.append(f"PU{round(s[0][0]*40)},{round(s[0][1]*40)};")
        lines.append("PD" + ",".join(f"{round(x*40)},{round(y*40)}" for x, y in s[1:]) + ";")
lines.append("PU;SP0;")
open("column_split.plt", "w").write("\n".join(lines))

cairosvg.svg2png(url="column_split.svg", write_to="column_split_preview.png", output_width=900)
print("done")
