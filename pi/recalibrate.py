#!/usr/bin/env python3
"""Refit the lens calibration from your EXISTING fixed50 images with a stable
distortion model (k1, k2, p1, p2; k3 fixed to 0). No new photos needed.

Why: the current fit has k3 ~ -10, which makes the model fold back near the
image edges, so points there are mapped to the wrong place.

  python3 ~/camera_calibration/bench/recalibrate.py
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

BASE = Path.home() / "camera_calibration"

p = argparse.ArgumentParser()
p.add_argument("--image-dir", default=str(BASE / "fixed50_images"))
p.add_argument("--output", default=str(BASE / "camera_calibration_fixed50_k3fixed.npz"))
p.add_argument("--compare", default=str(BASE / "camera_calibration_fixed50_no14.npz"))
p.add_argument("--pattern", type=int, nargs=2, default=[9, 6])
p.add_argument("--square-mm", type=float, default=24.0)
p.add_argument("--exclude", nargs="*", default=["calib_14.jpg"])
p.add_argument("--no-rotate", action="store_true")
p.add_argument("--glob", default="calib_*.jpg", help="only original captures, not rotated copies")
p.add_argument("--aruco-board", help="use the ArUco wall sheet (its wall_board.json) instead of the checkerboard")
p.add_argument("--min-markers", type=int, default=6)
a = p.parse_args()

if a.aruco_board:
    import json
    bj = json.load(open(a.aruco_board))
    sx, sy = float(bj.get("scale_x", 1.0)), float(bj.get("scale_y", 1.0))
    board_pts = {}
    for m in bj["markers"]:
        c = np.array(m["corners_mm"], np.float32)
        c[:, 0] *= sx
        c[:, 1] *= sy
        board_pts[int(m["id"])] = np.hstack([c, np.zeros((4, 1), np.float32)])
    prm = cv2.aruco.DetectorParameters()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    prm.cornerRefinementWinSize = 7
    prm.cornerRefinementMaxIterations = 60
    prm.cornerRefinementMinAccuracy = 0.01
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, bj["dictionary"])), prm)

cols, rows = a.pattern
objp = np.zeros((cols * rows, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * a.square_mm

files = sorted(Path(a.image_dir).glob(a.glob))
EXPECTED = (2592, 4608)   # width x height after the 90-degree CCW rotation
obj_pts, img_pts, names, size = [], [], [], EXPECTED
for f in files:
    if f.name in a.exclude:
        print(f"excluded  {f.name}")
        continue
    img = cv2.imread(str(f))
    if img is None:
        continue
    if not a.no_rotate:
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if g.shape[::-1] != EXPECTED:
        print(f"wrong size {f.name}: {g.shape[1]}x{g.shape[0]} after rotation "
              f"(expected {EXPECTED[0]}x{EXPECTED[1]}) - skipped")
        continue
    if a.aruco_board:
        corners, ids, _ = detector.detectMarkers(g)
        use = [] if ids is None else [(int(i), c) for c, i in zip(corners, ids.ravel())
                                      if int(i) in board_pts]
        if len(use) < a.min_markers:
            print(f"not found {f.name} ({len(use)} markers, need {a.min_markers})")
            continue
        obj_pts.append(np.vstack([board_pts[i] for i, _ in use]))
        img_pts.append(np.vstack([c.reshape(4, 2) for _, c in use]).astype(np.float32))
        names.append(f.name)
        print(f"accepted  {f.name} ({len(use)} markers)")
        continue
    ok, c = cv2.findChessboardCornersSB(g, (cols, rows), flags=cv2.CALIB_CB_ACCURACY)
    if not ok:
        print(f"not found {f.name}")
        continue
    obj_pts.append(objp)
    img_pts.append(c.astype(np.float32))
    names.append(f.name)
    print(f"accepted  {f.name}")

if len(names) < 8:
    raise SystemExit(f"Too few usable images ({len(names)}); need at least 8, ideally 20-30.")

# Views must differ: the same pose repeated gives no information about the lens.
spread = np.std([np.mean(p.reshape(-1, 2), 0) for p in img_pts], 0)
if spread.max() < 150:
    print("WARNING: the photos are nearly identical. Move/tilt the board between photos.")

flags = cv2.CALIB_FIX_K3


def fit(obj_pts, img_pts):
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, None, None, flags=flags)
    per = []
    for o, i, r, t in zip(obj_pts, img_pts, rvecs, tvecs):
        pr, _ = cv2.projectPoints(o, r, t, K, dist)
        per.append(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, 1))))
    return rms, K, dist, np.array(per)


rms, K, dist, per = fit(obj_pts, img_pts)
# Drop blurred / moved photos: per-view error far above the typical one.
for _ in range(3):
    lim = max(1.6 * np.median(per), 0.8)
    bad = [k for k in range(len(per)) if per[k] > lim]
    if not bad or len(per) - len(bad) < 12:
        break
    for k in bad:
        print(f"dropped   {names[k]}  (view error {per[k]:.2f} px > {lim:.2f} px)")
    keep = [k for k in range(len(per)) if k not in bad]
    obj_pts = [obj_pts[k] for k in keep]
    img_pts = [img_pts[k] for k in keep]
    names = [names[k] for k in keep]
    rms, K, dist, per = fit(obj_pts, img_pts)


def valid_fraction(K, d, size):
    w, h = size
    us, vs = np.meshgrid(np.linspace(0, w - 1, 60), np.linspace(0, h - 1, 100))
    uv = np.stack([us.ravel(), vs.ravel()], 1).reshape(-1, 1, 2)
    n = cv2.undistortPoints(uv, K, d).reshape(-1, 2)
    back, _ = cv2.projectPoints(np.hstack([n, np.ones((len(n), 1))]), np.zeros(3), np.zeros(3), K, d)
    return float((np.linalg.norm(back.reshape(-1, 2) - uv.reshape(-1, 2), axis=1) < 0.5).mean())


gc, gr = 3, 5   # coverage grid over the upright (portrait) image
cov = np.zeros((gr, gc), int)
for c in img_pts:
    p = c.reshape(-1, 2)
    cells = set(zip(np.clip((p[:, 1] / size[1] * gr).astype(int), 0, gr - 1),
                    np.clip((p[:, 0] / size[0] * gc).astype(int), 0, gc - 1)))
    for r_, c_ in cells:
        cov[r_, c_] += 1
print("\nCoverage (number of views with corners in each image region, upright view):")
for r_ in range(gr):
    print("   " + "  ".join(f"{v:3d}" for v in cov[r_]))
weak = int((cov < 3).sum())
print("   OK - every region covered" if weak == 0 else
      f"   WEAK: {weak} region(s) with < 3 views -> lens model unreliable there")

print(f"\nViews used: {len(names)}   image size {size[0]} x {size[1]}")
print(f"RMS reprojection: {rms:.4f} px   per-view mean {np.mean(per):.4f}, worst {np.max(per):.4f} px")
print("Camera matrix:\n", K)
print("Distortion (k1 k2 p1 p2 k3):", dist.ravel())
print(f"Image area where the lens model is valid: {valid_fraction(K, dist, size) * 100:.1f} %")
if Path(a.compare).exists():
    old = np.load(a.compare)
    Ko = old["camera_matrix"] if "camera_matrix" in old.files else old["K"]
    do = old["dist_coeffs"] if "dist_coeffs" in old.files else old["dist"]
    print(f"Old calibration ({Path(a.compare).name}) valid area: "
          f"{valid_fraction(np.asarray(Ko, float), np.asarray(do, float).ravel(), size) * 100:.1f} %")

np.savez(a.output, camera_matrix=K, dist_coeffs=dist, image_size=np.array(size),
         rms=rms, views=np.array(names))
print("Saved:", a.output)
