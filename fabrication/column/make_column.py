#!/usr/bin/env python3
"""Laser-cut limit-switch column for the motorized stage (5 mm acrylic).

Parts: base plate (stage + column stand on it), column with switch slots and an
engraved mm scale, 2 gussets, 2 carriage flags.
Outputs: switch_column.svg (red = cut, blue = engrave), switch_column.dxf
(layers CUT / ENGRAVE), switch_column_preview.png.
"""
import cairosvg
import ezdxf

T = 5.0            # acrylic thickness (nominal)
SLOT_T = 5.15      # slot width for 5 mm tabs (tight fit, laser kerf ~0.15-0.2)
STAGE_W, STAGE_D = 95.2, 48.2
SW_SPACING = 6.5   # microswitch mounting-hole spacing - MEASURE YOUR SWITCH
SW_SLOT_W = 2.3    # for M2 screws
COL_W, COL_H = 50.0, 600.0
GAPS = (20, 35, 50)  # column front face -> stage edge, choose one when assembling

cut, eng, txt = [], [], []   # polylines (closed), lines, (x, y, size, text)


def rect(x, y, w, h, ox, oy):
    cut.append([(ox + x, oy + y), (ox + x + w, oy + y), (ox + x + w, oy + y + h), (ox + x, oy + y + h)])


def poly(pts, ox, oy):
    cut.append([(ox + x, oy + y) for x, y in pts])


def circle(cx, cy, r, ox, oy, n=36):
    import math
    cut.append([(ox + cx + r * math.cos(2 * math.pi * k / n), oy + cy + r * math.sin(2 * math.pi * k / n))
                for k in range(n)])


def line(x1, y1, x2, y2, ox, oy):
    eng.append([(ox + x1, oy + y1), (ox + x2, oy + y2)])


def text(x, y, s, t, ox, oy):
    txt.append((ox + x, oy + y, s, t))


# ---------------- base plate 150 x 160 ----------------
BX, BY, BW, BH = 5, 2, 150.0, 160.0
rect(0, 0, BW, BH, BX, BY)
for cx, cy in [(7, 7), (BW - 7, 7), (7, BH - 7), (BW - 7, BH - 7)]:
    circle(cx, cy, 2.0, BX, BY)
sx = (BW - STAGE_W) / 2
STAGE_Y = 98.0
for (a, b, c, d) in [(sx, STAGE_Y, sx + STAGE_W, STAGE_Y), (sx + STAGE_W, STAGE_Y, sx + STAGE_W, STAGE_Y + STAGE_D),
                     (sx + STAGE_W, STAGE_Y + STAGE_D, sx, STAGE_Y + STAGE_D), (sx, STAGE_Y + STAGE_D, sx, STAGE_Y)]:
    line(a, b, c, d, BX, BY)
text(sx + 12, STAGE_Y + 27, 4, "STAGE BASE HERE (95.2 x 48.2)", BX, BY)
cxm = BW / 2
for g in GAPS:
    front = STAGE_Y - g
    back = front - SLOT_T
    rect(cxm - 13, back, 8, SLOT_T, BX, BY)          # column tab slots
    rect(cxm + 5, back, 8, SLOT_T, BX, BY)
    for gx in (-17.5, 17.5):                          # 2 gusset tab slots, behind the column edges
        rect(cxm + gx - SLOT_T / 2, back - 14, SLOT_T, 12, BX, BY)
    text(cxm + 28, front - 1, 3, f"gap {g}", BX, BY)

# ---------------- column 50 x 600 (+5 mm tabs) ----------------
CX, CY = 165, 2
c = COL_W / 2
poly([(0, 0), (COL_W, 0), (COL_W, COL_H), (c + 13, COL_H), (c + 13, COL_H + T),
      (c + 5, COL_H + T), (c + 5, COL_H), (c - 5, COL_H), (c - 5, COL_H + T), (c - 13, COL_H + T),
      (c - 13, COL_H), (0, COL_H)], CX, CY)
for lo, hi in [(70, 140), (300, 370)]:          # switch slot height ranges above the plate
    for xs in (COL_W / 2 - SW_SPACING / 2, COL_W / 2 + SW_SPACING / 2):
        rect(xs - SW_SLOT_W / 2, COL_H - hi, SW_SLOT_W, hi - lo, CX, CY)
for h in range(0, int(COL_H) + 1):               # mm scale, 0 = top of base plate
    L = 6 if h % 10 == 0 else 4 if h % 5 == 0 else 2
    line(0, COL_H - h, L, COL_H - h, CX, CY)
    if h % 10 == 0 and 0 < h < COL_H:
        text(6.8, COL_H - h + 1, 2.4, str(h), CX, CY)
text(29, COL_H - 62, 2.2, "BOTTOM SW", CX, CY)
text(29, COL_H - 57, 2.2, "lever up", CX, CY)
text(29, COL_H - 292, 2.2, "TOP SW", CX, CY)
text(29, COL_H - 287, 2.2, "lever down", CX, CY)

# ---------------- 2 gussets ----------------
for oy in (2, 105):
    poly([(0, 0), (0, 90), (2, 90), (2, 90 + T), (14, 90 + T), (14, 90), (40, 90)], 225, oy)

# ---------------- 2 carriage flags 70 x 20 ----------------
for oy in (215, 245):
    rect(0, 0, 70, 20, 225, oy)
    rect(10, 10 - 1.7, 45, 3.4, 225, oy)

W_ALL, H_ALL = 300, 609

# ---------------- SVG ----------------
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
open("switch_column.svg", "w").write("\n".join(out))

# ---------------- DXF (Y up) ----------------
doc = ezdxf.new("R2010", setup=True)
doc.units = ezdxf.units.MM
doc.layers.add("CUT", color=1)
doc.layers.add("ENGRAVE", color=5)
msp = doc.modelspace()
for p in cut:
    msp.add_lwpolyline([(x, H_ALL - y) for x, y in p], close=True, dxfattribs={"layer": "CUT"})
for p in eng:
    msp.add_line((p[0][0], H_ALL - p[0][1]), (p[1][0], H_ALL - p[1][1]), dxfattribs={"layer": "ENGRAVE"})
for x, y, s, t in txt:
    msp.add_text(t, height=s, dxfattribs={"layer": "ENGRAVE"}).set_placement((x, H_ALL - y))
doc.saveas("switch_column.dxf")

cairosvg.svg2png(url="switch_column.svg", write_to="switch_column_preview.png", output_width=1400)
print("cut paths:", len(cut), " engrave lines:", len(eng))
