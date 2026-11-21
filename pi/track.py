#!/usr/bin/env python3
"""Automatic tracking of optical-bench components with small tags.

Each component holder carries one printed tag (ArUco DICT_4X4_50, IDs 30-49,
so they never clash with the wall markers 0-19). Every frame the wall is
located (bench.py) and every visible tag is converted to wall coordinates:

    X, Y  = position in the wall frame (mm, origin top-left of sheet, X right, Y down)
    gap   = distance in front of the wall (mm)

then the component's optical centre is reported: tag centre + its offset.

Commands (on the Pi, in ~/camera_calibration/bench):
  python3 track.py make-tags                 # printable tag sheet (PDF) - run on any PC
  python3 track.py track --lens-position 2.0 # live tracking, Ctrl+C to stop
  python3 track.py track --once              # one measurement and exit
  python3 track.py track --image FILE        # process a saved photo

components.json (created on first run) names each tag and stores:
  tag_mm     : measured black-square size of that printed tag (vernier)
  offset_mm  : [dx, dy, dz] from tag centre to the optical centre
               (dx right, dy down, dz towards the camera), measured once
  gap_mm     : null = measure gap from the tag automatically;
               a number = the holder's gap is fixed (e.g. rail parallel to wall),
               which gives the most accurate X, Y.
"""
import argparse
import csv
import datetime as dt
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import bench

COMPONENTS = bench.WORK / "components.json"
TRACK_CSV = bench.WORK / "tracking.csv"
TAG_IDS = range(30, 50)
DEFAULT_TAG_MM = 20.0


# ---------------------------------------------------------------- tag sheet
def make_tags(args):
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    size, gapmm = args.size, 12.0
    W, H = 210.0, 297.0
    c = canvas.Canvas(args.output, pagesize=(W * mm, H * mm))
    cols = int((W - 20) // (size + gapmm))
    x0, y0 = 15.0, 25.0
    c.setFont("Helvetica", 8)
    c.drawString(15 * mm, (H - 12) * mm,
                 f"Component tags | DICT_4X4_50 | black square = {size:.1f} mm | print at 100% (Actual size)")
    c.drawString(15 * mm, (H - 17) * mm,
                 "Cut along the grey lines, keep the white border. Measure each black square with a vernier.")
    for k, tid in enumerate(range(30, 30 + args.count)):
        r, q = divmod(k, cols)
        x = x0 + q * (size + gapmm)
        y = y0 + r * (size + gapmm + 4)
        bits = cv2.aruco.generateImageMarker(d, tid, 6, borderBits=1) < 128
        cell = size / 6
        c.setFillColorRGB(0, 0, 0)
        for i in range(6):
            for j in range(6):
                if bits[i, j]:
                    c.rect((x + j * cell) * mm, (H - y - (i + 1) * cell) * mm, cell * mm, cell * mm,
                           stroke=0, fill=1)
        c.setStrokeColorRGB(0.7, 0.7, 0.7)
        c.setLineWidth(0.3)
        m = 4.0  # white quiet zone kept around the tag
        c.rect((x - m) * mm, (H - y - size - m) * mm, (size + 2 * m) * mm, (size + 2 * m) * mm,
               stroke=1, fill=0)
        c.drawString(x * mm, (H - y - size - m - 3.5) * mm, f"ID {tid}")
    c.showPage()
    c.save()
    print(f"Saved {args.output}: tags ID 30-{29 + args.count}, {size} mm")


# ---------------------------------------------------------------- components
def load_components():
    if COMPONENTS.exists():
        return {int(k): v for k, v in json.load(open(COMPONENTS)).items()}
    return {}


def save_components(comp):
    bench.WORK.mkdir(parents=True, exist_ok=True)
    json.dump({str(k): v for k, v in sorted(comp.items())}, open(COMPONENTS, "w"), indent=2)


def component(comp, tid):
    if tid not in comp:
        comp[tid] = dict(name=f"Component {tid}", tag_mm=DEFAULT_TAG_MM,
                         offset_mm=[0.0, 0.0, 0.0], gap_mm=None)
        save_components(comp)
        print(f"New tag ID {tid} added to {COMPONENTS} - edit its name / tag_mm / offset.")
    return comp[tid]


# ---------------------------------------------------------------- measurement
def corner_shrink_px(wall, board_pts):
    """Average inward pull of detected marker corners (pixels), measured on the
    wall markers by comparing detected sides with the sides predicted by the
    wall pose. Blur / focus changes shrink every marker in the image alike."""
    rvec, _ = cv2.Rodrigues(wall.R)
    d = []
    for i, det in wall.img_corners.items():
        obj = np.hstack([board_pts[i], np.zeros((4, 1))])
        proj, _ = cv2.projectPoints(obj, rvec, wall.t, wall.K, wall.dist)
        proj = proj.reshape(4, 2)
        side = lambda q: np.linalg.norm(np.roll(q, -1, 0) - q, axis=1)
        d.append((side(proj) - side(det)).mean() / 2.0)
    return float(np.median(d)) if d else 0.0


def tag_in_wall(wall, corners_px, cfg, shrink_px=0.0, gap_override=None):
    """Return tag centre (X, Y, gap) in wall mm."""
    s = float(cfg.get("tag_mm", DEFAULT_TAG_MM))
    # Centre pixel = intersection of the diagonals (robust to blur shrink).
    p = np.asarray(corners_px, float)
    d1, d2 = p[2] - p[0], p[3] - p[1]
    A = np.array([d1, -d2]).T
    t = np.linalg.solve(A, p[1] - p[0])[0]
    centre_px = p[0] + t * d1
    # Undo the blur shrink measured on the wall markers before using the size.
    side_px = np.linalg.norm(np.roll(p, -1, 0) - p, axis=1).mean()
    if shrink_px:
        p = centre_px + (p - centre_px) * (1.0 + 2.0 * shrink_px / side_px)

    gap_fixed = gap_override if gap_override is not None else cfg.get("gap_mm")
    if gap_fixed is not None:
        gap = float(gap_fixed)
        xy = wall.to_wall(centre_px.reshape(1, 2), gap)[0]
        return np.array([xy[0], xy[1], gap]), centre_px, "fixed"

    # Order required by SOLVEPNP_IPPE_SQUARE (matches ArUco TL, TR, BR, BL).
    obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0],
                    [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], float)
    ok, rvec, tvec = cv2.solvePnP(obj, p, wall.K, wall.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None, centre_px, "fail"
    # Tags are parallel to the wall (holders do not rotate): keep the depth from
    # the pose, but take X, Y from the sharp centre ray at that depth.
    p_cam = tvec.reshape(3)
    p_wall = wall.R.T @ (p_cam - wall.t)
    gap = float(wall.front_sign * p_wall[2]) + float(cfg.get("gap_offset_mm", 0.0) or 0.0)
    xy = wall.to_wall(centre_px.reshape(1, 2), gap)[0]
    return np.array([xy[0], xy[1], gap]), centre_px, "auto"


def measure_frame(img, args, comp):
    K, dist, cs, bj, bp, outline = args._setup
    wall = bench.Wall(img, bj, bp, outline, K, dist)
    shrink = 0.0 if args.no_blur_fix else corner_shrink_px(wall, bp)
    wall.shrink_px = shrink
    out = []
    for tid, corners in sorted(wall.other_markers.items()):
        if tid not in TAG_IDS:
            continue
        cfg = component(comp, tid)
        res, cpx, mode = tag_in_wall(wall, corners, cfg, shrink, args.gap_mm)
        if res is None:
            continue
        off = np.array(cfg.get("offset_mm", [0, 0, 0]), float)
        oc = res + off
        out.append(dict(id=tid, name=cfg.get("name", f"Component {tid}"), x=oc[0], y=oc[1],
                        gap=oc[2], mode=mode, centre_px=cpx, corners=corners))
    return wall, out


def draw(img, wall, results, path):
    vis = img.copy()
    for c in wall.img_corners.values():
        cv2.polylines(vis, [c.astype(np.int32)], True, (0, 200, 0), 3)
    for r in results:
        cv2.polylines(vis, [r["corners"].astype(np.int32)], True, (0, 0, 255), 5)
        u, v = r["centre_px"].astype(int)
        cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 60, 4)
        cv2.putText(vis, f"{r['name']}: X{r['x']:.1f} Y{r['y']:.1f} gap{r['gap']:.1f}",
                    (int(u) + 40, int(v) - 40), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 4)
    vis = cv2.resize(vis, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(path), vis, [cv2.IMWRITE_JPEG_QUALITY, 85])


# ---------------------------------------------------------------- camera
class Camera:
    """Continuous capture with Picamera2 if available, else rpicam-still per frame."""

    def __init__(self, lens_position):
        self.lens = lens_position
        self.cam = None
        try:
            from picamera2 import Picamera2
            from libcamera import controls
            cam = Picamera2()
            cfg = cam.create_still_configuration(main={"size": (4608, 2592), "format": "RGB888"},
                                                 buffer_count=2)
            cam.configure(cfg)
            ctr = {"AfMode": controls.AfModeEnum.Manual}
            if lens_position is not None:
                ctr["LensPosition"] = float(lens_position)
            cam.set_controls(ctr)
            cam.start()
            time.sleep(1.5)
            self.cam = cam
            print("Camera: Picamera2 continuous capture")
        except Exception as e:
            print(f"Camera: Picamera2 unavailable ({e.__class__.__name__}); using rpicam-still")

    def grab(self):
        if self.cam is not None:
            img = self.cam.capture_array("main")   # RGB888 = BGR byte order for OpenCV
        else:
            path = bench.OUT_DIR / "track_frame.jpg"
            bench.OUT_DIR.mkdir(parents=True, exist_ok=True)
            bench.capture(path, self.lens, timeout_ms=800)
            img = cv2.imread(str(path))
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def close(self):
        if self.cam is not None:
            self.cam.stop()


def track(args):
    args._setup = bench.common_setup(args)
    comp = load_components()
    new = not TRACK_CSV.exists()
    log = open(TRACK_CSV, "a", newline="")
    w = csv.writer(log)
    if new:
        w.writerow(["time", "id", "name", "x_mm", "y_mm", "gap_mm", "gap_mode",
                    "cam_wall_mm", "wall_fit_rms_mm", "frames_averaged"])
    hist = {}
    cam = None
    try:
        if not args.image:
            cam = Camera(args.lens_position)
        n = 0
        while True:
            img = (bench.load_image(args.image, not args.no_rotate) if args.image else cam.grab())
            try:
                wall, res = measure_frame(img, args, comp)
            except RuntimeError as e:
                print(f"[{dt.datetime.now():%H:%M:%S}] {e}")
                if args.once or args.image:
                    break
                continue
            stamp = dt.datetime.now()
            line = []
            for r in res:
                h = hist.setdefault(r["id"], [])
                h.append((r["x"], r["y"], r["gap"]))
                del h[:-args.avg]
                m = np.mean(h, 0)
                line.append(f"{r['name']}: X={m[0]:.2f} Y={m[1]:.2f} gap={m[2]:.1f}"
                            + (" (fixed gap)" if r["mode"] == "fixed" else ""))
                w.writerow([stamp.isoformat(timespec="seconds"), r["id"], r["name"],
                            f"{m[0]:.3f}", f"{m[1]:.3f}", f"{m[2]:.2f}", r["mode"],
                            f"{wall.distance_mm:.1f}", f"{wall.fit_rms_mm:.3f}", len(h)])
            seen = {r["id"] for r in res}
            for k in list(hist):
                if k not in seen:
                    del hist[k]       # component left view: restart its average
            log.flush()
            print(f"[{stamp:%H:%M:%S}] wall {len(wall.ids)} markers, shrink {wall.shrink_px:.2f}px | "
                  + (" | ".join(line) if line else "no component tags visible"))
            n += 1
            if args.preview and (n % args.preview == 0 or args.once or args.image):
                draw(img, wall, res, bench.WORK / "track_preview.jpg")
            if args.once or args.image:
                break
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if cam:
            cam.close()
        log.close()
        print(f"Log: {TRACK_CSV}")


def calibrate_gap(args):
    """Auto-gap calibration: holder at a ruler-measured gap -> store the correction
    so automatic gap mode is right for this tag from then on."""
    args._setup = bench.common_setup(args)
    args.no_blur_fix, args.gap_mm_override = False, None
    comp = load_components()
    cfg = component(comp, args.id)
    raw_cfg = dict(cfg, gap_mm=None, gap_offset_mm=0.0)
    cam = None if args.image else Camera(args.lens_position)
    vals = []
    try:
        for k in range(args.frames):
            img = bench.load_image(args.image, True) if args.image else cam.grab()
            K, dist, cs, bj, bp, outline = args._setup
            wall = bench.Wall(img, bj, bp, outline, K, dist)
            if args.id not in wall.other_markers:
                print(f"  frame {k + 1}: tag {args.id} not visible")
                continue
            res, _, _ = tag_in_wall(wall, wall.other_markers[args.id], raw_cfg,
                                    corner_shrink_px(wall, bp))
            vals.append(res[2])
            print(f"  frame {k + 1}/{args.frames}: raw auto gap {res[2]:.2f} mm")
            if args.image:
                break
    finally:
        if cam:
            cam.close()
    if not vals:
        sys.exit("Tag not seen - nothing saved.")
    raw = float(np.mean(vals))
    cfg["gap_offset_mm"] = round(args.gap_mm - raw, 2)
    cfg["gap_mm"] = None
    save_components(comp)
    sd = float(np.std(vals, ddof=1)) if len(vals) > 1 else float("nan")
    print(f"\nTag {args.id}: ruler gap {args.gap_mm:.2f}, raw auto gap {raw:.2f} (spread {sd:.2f})")
    print(f"Saved gap_offset_mm = {cfg['gap_offset_mm']:+.2f} and gap_mm = null (automatic gap mode)")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calib", default=str(bench.DEFAULT_CALIB))
    p.add_argument("--board", default=str(bench.DEFAULT_BOARD))
    sub = p.add_subparsers(dest="cmd", required=True)
    mt = sub.add_parser("make-tags")
    mt.add_argument("--size", type=float, default=DEFAULT_TAG_MM, help="black square size, mm")
    mt.add_argument("--count", type=int, default=12)
    mt.add_argument("--output", default="component_tags.pdf")
    t = sub.add_parser("track")
    t.add_argument("--lens-position", type=float, default=None)
    t.add_argument("--image", help="process a saved raw photo instead of the camera")
    t.add_argument("--no-rotate", action="store_true")
    t.add_argument("--once", action="store_true", help="one frame, then exit")
    t.add_argument("--gap-mm", type=float, default=None,
                   help="fixed gap for all components (overrides components.json)")
    t.add_argument("--no-blur-fix", action="store_true", help="disable the blur-shrink correction")
    t.add_argument("--avg", type=int, default=5, help="running average over N frames")
    t.add_argument("--preview", type=int, default=10, help="save track_preview.jpg every N frames (0=off)")
    cg = sub.add_parser("calibrate-gap", help="one-time auto-gap correction for one tag")
    cg.add_argument("--id", type=int, required=True)
    cg.add_argument("--gap-mm", type=float, required=True, help="ruler/vernier gap at this position")
    cg.add_argument("--lens-position", type=float, default=None)
    cg.add_argument("--frames", type=int, default=20)
    cg.add_argument("--image")
    a = p.parse_args()
    {"make-tags": make_tags, "track": track, "calibrate-gap": calibrate_gap}[a.cmd](a)


if __name__ == "__main__":
    main()
