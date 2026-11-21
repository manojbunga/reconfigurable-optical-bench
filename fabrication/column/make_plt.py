"""Convert switch_column.dxf to HPGL (.plt): pen 1 = CUT (red), pen 2 = ENGRAVE (blue). Text converted to outlines."""
import ezdxf
from ezdxf.addons import text2path
from ezdxf import path as zpath

doc = ezdxf.readfile("switch_column.dxf")
msp = doc.modelspace()
U = 40  # HPGL units per mm
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
        pts = [(v.x, v.y) for v in zpath.make_path(e).flattening(0.02)]
        strokes[pen].append(pts)
out = ["IN;"]
for pen in (1, 2):
    out.append(f"SP{pen};")
    for s in strokes[pen]:
        x0, y0 = s[0]
        out.append(f"PU{round(x0*U)},{round(y0*U)};")
        out.append("PD" + ",".join(f"{round(x*U)},{round(y*U)}" for x, y in s[1:]) + ";")
out.append("PU;SP0;")
open("switch_column.plt", "w").write("\n".join(out))
print({k: len(v) for k, v in strokes.items()})
