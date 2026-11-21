#!/usr/bin/env python3
"""Re-cut base plate with tighter slots + a small fit-test coupon.

Laser kerf (~0.2 mm) makes holes bigger and tabs smaller, so the drawn slot
must be narrower than the sheet. Cut the COUPON first, find the slot that grips
your tabs, then cut base_plate_W<width>.dxf with that width.
"""
import math
import cairosvg
import ezdxf
from ezdxf.addons import text2path
from ezdxf import path as zpath

T = 5.0
BW, BH, STAGE_Y = 150.0, 160.0, 98.0
STAGE_W, STAGE_D = 95.2, 48.2
COL_LENS = [7.0, 7.2, 7.4, 7.6, 7.8, 8.0]       # drawn column-slot lengths to try
GUS_LENS = [11.0, 11.2, 11.4, 11.6, 11.8, 12.0] # drawn gusset-slot lengths to try
WIDTHS = [4.0, 4.1, 4.2, 4.3]    # drawn slot widths to try
GAPS = (20, 35, 50)


def save(name, cut, eng, txt, W, H, png_w=900):
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">',
           f'<rect width="{W}" height="{H}" fill="white"/>']
    for p in cut:
        out.append('<polygon points="' + " ".join(f"{x:.3f},{y:.3f}" for x, y in p) +
                   '" fill="none" stroke="#FF0000" stroke-width="0.1"/>')
    for p in eng:
        out.append('<polyline points="' + " ".join(f"{x:.3f},{y:.3f}" for x, y in p) +
                   '" fill="none" stroke="#0000FF" stroke-width="0.1"/>')
    for x, y, s, t in txt:
        out.append(f'<text x="{x:.2f}" y="{y:.2f}" font-family="Arial" font-size="{s}" fill="#0000FF">{t}</text>')
    out.append("</svg>")
    open(name + ".svg", "w").write("\n".join(out))
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=1)
    doc.layers.add("ENGRAVE", color=5)
    msp = doc.modelspace()
    for p in cut:
        msp.add_lwpolyline([(x, H - y) for x, y in p], close=True, dxfattribs={"layer": "CUT"})
    for p in eng:
        msp.add_lwpolyline([(x, H - y) for x, y in p], dxfattribs={"layer": "ENGRAVE"})
    for x, y, s, t in txt:
        msp.add_text(t, height=s, dxfattribs={"layer": "ENGRAVE"}).set_placement((x, H - y))
    doc.saveas(name + ".dxf")
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
    open(name + ".plt", "w").write("\n".join(lines))
    cairosvg.svg2png(url=name + ".svg", write_to=name + "_preview.png", output_width=png_w)


def R(cut, x, y, w, h, ox=0, oy=0):
    cut.append([(ox + x, oy + y), (ox + x + w, oy + y), (ox + x + w, oy + y + h), (ox + x, oy + y + h)])


def C(cut, cx, cy, r, ox=0, oy=0, n=36):
    cut.append([(ox + cx + r * math.cos(2 * math.pi * k / n), oy + cy + r * math.sin(2 * math.pi * k / n))
                for k in range(n)])


# ---------- fit-test coupon ----------
# Row A: column-tab length test, Row B: gusset-tab length test (both at a roomy 4.6 width so only
# the LENGTH is tested). Row C: width test (4.0-4.3) at roomy lengths, so only the WIDTH is tested.
cut, eng, txt = [], [], []
P = 10.0                                   # pitch between slots
n = len(COL_LENS)
CW = 8 + P * max(n, 2 * len(WIDTHS)) + 4
CH = 76
R(cut, 0, 0, CW, CH, 2, 2)
txt.append((4, 7, 2.6, "A column length"))
for i, L in enumerate(COL_LENS):
    x = 2 + 8 + P * i
    R(cut, x, 2 + 10, 4.6, L)
    txt.append((x - 0.6, 2 + 10 + 8.0 + 3.6, 2.0, f"{L:.1f}"))
txt.append((4, 31, 2.6, "B gusset length"))
for i, L in enumerate(GUS_LENS):
    x = 2 + 8 + P * i
    R(cut, x, 2 + 33, 4.6, L)
    txt.append((x - 1.2, 2 + 33 + 12.0 + 3.6, 2.0, f"{L:.1f}"))
txt.append((4, 59.5, 2.6, "C width"))
for i, w in enumerate(WIDTHS):
    x = 2 + 8 + P * (2 * i)
    R(cut, x, 2 + 61, w, 8.4)              # roomy length -> tests width only
    txt.append((x - 0.6, 2 + 61 + 8.4 + 3.4, 2.0, f"{w:.1f}"))
save("fit_coupon", cut, eng, txt, CW + 4, CH + 4, png_w=700)


def plate(w, col_l, gus_l):
    cut, eng, txt = [], [], []
    ox, oy = 2, 2
    R(cut, 0, 0, BW, BH, ox, oy)
    for cx, cy in [(7, 7), (BW - 7, 7), (7, BH - 7), (BW - 7, BH - 7)]:
        C(cut, cx, cy, 2.05, ox, oy)
    sx = (BW - STAGE_W) / 2
    eng.append([(ox + sx, oy + STAGE_Y), (ox + sx + STAGE_W, oy + STAGE_Y), (ox + sx + STAGE_W, oy + STAGE_Y + STAGE_D),
                (ox + sx, oy + STAGE_Y + STAGE_D), (ox + sx, oy + STAGE_Y)])
    txt.append((ox + sx + 12, oy + STAGE_Y + 27, 4, "STAGE BASE HERE (95.2 x 48.2)"))
    cxm = BW / 2
    for g in GAPS:
        front = STAGE_Y - g                   # column front face (stage side) of the slot
        for tx in (cxm - 9, cxm + 9):         # column tab centres
            R(cut, tx - col_l / 2, front - w, col_l, w, ox, oy)
        # Gusset tab centre is 8 mm (drawn) from the gusset's upright edge; the column's back face
        # is the far wall of the column slot. Kerf cancels out, so the gusset ends flush on the column.
        gc = front - w - 8.0
        for gx in (-17.5, 17.5):
            R(cut, cxm + gx - w / 2, gc - gus_l / 2, w, gus_l, ox, oy)
        txt.append((ox + cxm + 28, oy + front - 1, 3, f"gap {g}"))
    txt.append((ox + 5, oy + BH - 3, 2.5, f"W{w:.1f} C{col_l:.1f} G{gus_l:.1f}"))
    name = f"base_plate_W{w:.1f}_C{col_l:.1f}_G{gus_l:.1f}"
    save(name, cut, eng, txt, BW + 4, BH + 4)
    return name


if __name__ == "__main__":
    import sys
    if len(sys.argv) == 4:                    # exact plate: python3 make_base.py W COL GUS
        print(plate(*map(float, sys.argv[1:])))
    else:
        for w in WIDTHS:
            print(plate(w, COL_LENS[0], GUS_LENS[0]))
