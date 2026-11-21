#!/usr/bin/env python3
"""Optical bench XY measurement on the reference wall.

The wall carries a ring of ArUco markers at known mm positions (wall_board.json).
Every image locates the wall afresh, so the camera-to-wall distance may change
between setups. The object is found by comparing against an empty-wall
reference image, and its position is reported in wall millimetres on a plane
that is --gap-mm in front of the wall surface.

Commands (run on the Raspberry Pi):
  python3 bench.py set-scale --h-mm 99.8 --v-mm 100.1     # after measuring the printed 100 mm bars
  python3 bench.py reference                               # empty wall, once per setup
  python3 bench.py measure [--gap-mm 0] [--frames 5]       # object in front of the wall
  python3 bench.py measure --pixel 1300 2200 --gap-mm 15   # a pixel you picked by hand
  python3 bench.py validate --gap-mm 0 --square-mm 24.0    # checkerboard as an independent ruler

Add --image FILE to any of reference/measure/validate to process a saved photo
instead of capturing. Images are rotated 90 deg counter-clockwise, as in the
camera calibration (--no-rotate to disable).
"""
import argparse
import csv
import datetime as dt
import json
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

BASE = Path.home() / "camera_calibration"
WORK = BASE / "bench"
DEFAULT_CALIB = BASE / "camera_calibration_fixed50_k3fixed.npz"   # from recalibrate.py
DEFAULT_BOARD = WORK / "wall_board.json"
REFERENCE_IMG = WORK / "reference.jpg"
REFERENCE_META = WORK / "reference.json"
LOG_CSV = WORK / "measurements.csv"
OUT_DIR = WORK / "out"


# ---------------------------------------------------------------- loading
def load_calibration(path):
    cal = np.load(str(path))
    K = cal["camera_matrix"] if "camera_matrix" in cal.files else cal["K"]
    dist = cal["dist_coeffs"] if "dist_coeffs" in cal.files else cal["dist"]
    size = None
    for key in ("image_size", "image_size_wh", "size"):
        if key in cal.files:
            size = tuple(int(v) for v in np.array(cal[key]).ravel()[:2])
    return np.asarray(K, float), np.asarray(dist, float).ravel(), size


def load_board(path):
    with open(path) as f:
        b = json.load(f)
    sx, sy = float(b.get("scale_x", 1.0)), float(b.get("scale_y", 1.0))
    pts = {}
    for m in b["markers"]:
        c = np.array(m["corners_mm"], float)
        c[:, 0] *= sx
        c[:, 1] *= sy
        pts[int(m["id"])] = c
    outline = np.array([[0, 0], [b["page_w_mm"] * sx, 0],
                        [b["page_w_mm"] * sx, b["page_h_mm"] * sy],
                        [0, b["page_h_mm"] * sy]], float)
    return b, pts, outline


def capture(path, lens_position=None, timeout_ms=1500):
    cmd = ["rpicam-still", "-n", "-t", str(timeout_ms),
           "--width", "4608", "--height", "2592", "-o", str(path)]
    if lens_position is None:
        cmd += ["--autofocus-mode", "auto", "--autofocus-on-capture"]
    else:
        cmd += ["--autofocus-mode", "manual", "--lens-position", str(lens_position)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or not Path(path).exists():
        sys.exit("Capture failed:\n" + r.stderr[-1500:])


def load_image(path, rotate=True):
    img = cv2.imread(str(path))
    if img is None:
        sys.exit(f"Cannot read image: {path}")
    if rotate:
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img


def check_size(img, K, calib_size):
    h, w = img.shape[:2]
    if calib_size and (w, h) != tuple(calib_size):
        sys.exit(f"Image is {w}x{h} but calibration is {calib_size}. "
                 "Use full resolution and the same rotation as calibration.")
    if (w, h) != (2592, 4608):
        print(f"WARNING: image size {w}x{h}; calibration was made at 2592x4608 (rotated).")


# ---------------------------------------------------------------- wall model
def make_detector(dict_name):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 7
    params.cornerRefinementMaxIterations = 60
    params.cornerRefinementMinAccuracy = 0.01
    if hasattr(aruco, "ArucoDetector"):
        det = aruco.ArucoDetector(dictionary, params)
        return lambda g: det.detectMarkers(g)
    return lambda g: aruco.detectMarkers(g, dictionary, parameters=params)


class Wall:
    """Wall plane located in one image."""

    def __init__(self, img, board_json, board_pts, outline, K, dist):
        self.K, self.dist, self.outline = K, dist, outline
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = make_detector(board_json["dictionary"])(gray)
        found = {}
        self.other_markers = {}   # detections that are not wall markers (component tags)
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if int(i) in board_pts:
                    found[int(i)] = c.reshape(4, 2)
                else:
                    self.other_markers[int(i)] = c.reshape(4, 2)
        self.ids = sorted(found)
        self.img_corners = found
        if len(found) < 4:
            raise RuntimeError(f"Only {len(found)} wall markers detected (need >= 4). "
                               "Check the whole wall is in view, lighting, and focus.")
        img_pts = np.vstack([found[i] for i in self.ids]).astype(np.float64)
        obj_xy = np.vstack([board_pts[i] for i in self.ids]).astype(np.float64)
        # Reject points where the lens model is not trustworthy (undistort ->
        # re-distort must return to the same pixel).
        valid = self.model_valid(img_pts)
        self.rejected_pts = int((~valid).sum())
        if valid.sum() < 16:
            raise RuntimeError(f"Only {valid.sum()} marker corners are in the valid lens-model "
                               "area. Recalibrate with recalibrate.py (fixed k3).")
        img_pts, obj_xy = img_pts[valid], obj_xy[valid]
        self.img_pts, self.obj_xy = img_pts, obj_xy

        # Plane mapping (gap = 0): homography on undistorted normalised points.
        # Independent of focal-length error; only lens distortion matters.
        n = self.normalise(img_pts)
        H, mask = cv2.findHomography(obj_xy, n, cv2.RANSAC, 0.003)
        good = mask.ravel().astype(bool)
        H, _ = cv2.findHomography(obj_xy[good], n[good], 0)
        self.H, self.Hinv = H, np.linalg.inv(H)
        self.inlier_frac = good.mean()

        # Full pose, needed for gap != 0.
        obj3 = np.hstack([obj_xy, np.zeros((len(obj_xy), 1))])
        ok, rvec, tvec = cv2.solvePnP(obj3[good], img_pts[good], K, dist,
                                      flags=cv2.SOLVEPNP_IPPE)
        rvec, tvec = cv2.solvePnPRefineLM(obj3[good], img_pts[good], K, dist, rvec, tvec)
        R, _ = cv2.Rodrigues(rvec)
        self.R, self.t = R, tvec.reshape(3)
        self.cam_w = -R.T @ self.t                     # camera centre in wall coords
        self.front_sign = np.sign(self.cam_w[2])        # side of the wall the camera is on
        proj, _ = cv2.projectPoints(obj3, rvec, tvec, K, dist)
        self.pose_rms_px = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts) ** 2, 1))))
        back = self.to_wall(img_pts, 0.0)
        res = np.linalg.norm(back - obj_xy, axis=1)
        self.fit_rms_mm = float(np.sqrt(np.mean(res[good] ** 2)))
        self.fit_max_mm = float(res[good].max())
        self.distance_mm = float(abs(self.cam_w[2]))
        self.mm_per_px = self.distance_mm / float(K[0, 0])

    def model_valid(self, uv, tol_px=0.5):
        uv = np.asarray(uv, np.float64).reshape(-1, 2)
        n = self.normalise(uv)
        p3 = np.hstack([n, np.ones((len(n), 1))])
        back, _ = cv2.projectPoints(p3, np.zeros(3), np.zeros(3), self.K, self.dist)
        return np.linalg.norm(back.reshape(-1, 2) - uv, axis=1) < tol_px

    def normalise(self, uv):
        uv = np.asarray(uv, np.float64).reshape(-1, 1, 2)
        return cv2.undistortPoints(uv, self.K, self.dist).reshape(-1, 2)

    def to_wall(self, uv, gap_mm=0.0):
        """Image pixels -> wall XY (mm) on the plane gap_mm in front of the wall."""
        n = self.normalise(uv)
        if gap_mm == 0:
            p = np.hstack([n, np.ones((len(n), 1))]) @ self.Hinv.T
            return p[:, :2] / p[:, 2:3]
        rays_c = np.hstack([n, np.ones((len(n), 1))])
        rays_w = rays_c @ self.R              # = (R.T @ ray).T
        z0 = self.front_sign * gap_mm
        s = (z0 - self.cam_w[2]) / rays_w[:, 2]
        p = self.cam_w + s[:, None] * rays_w
        return p[:, :2]

    def outline_px(self):
        """Image region covered by the detected wall markers (convex hull)."""
        allc = np.vstack(list(self.img_corners.values())).astype(np.float32)
        return cv2.convexHull(allc).reshape(-1, 2)

    def summary(self):
        tilt = np.degrees(np.arccos(abs(self.R[2, 2])))
        warn = (f"\nWARNING: {self.rejected_pts} marker corners rejected (outside valid lens model)"
                if self.rejected_pts else "")
        return (f"Wall markers: {len(self.ids)} | camera-wall distance {self.distance_mm:.1f} mm | "
                f"tilt {tilt:.1f} deg | ~{self.mm_per_px:.3f} mm/px\n"
                f"Wall fit: RMS {self.fit_rms_mm:.3f} mm, max {self.fit_max_mm:.3f} mm "
                f"(pose reprojection {self.pose_rms_px:.2f} px)" + warn)


# ---------------------------------------------------------------- object finding
def find_object(ref_img, img, wall, min_area_px, thresh):
    """Largest region that differs from the empty-wall reference, inside the wall."""
    a = cv2.GaussianBlur(cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY), (0, 0), 2.0).astype(np.float32)
    b = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (0, 0), 2.0).astype(np.float32)
    # Compensate overall brightness change (exposure/lighting drift).
    b *= (np.median(a) + 1e-6) / (np.median(b) + 1e-6)
    diff = cv2.absdiff(a, b)
    mask = (diff > thresh).astype(np.uint8) * 255
    wall_mask = np.zeros_like(mask)
    cv2.fillConvexPoly(wall_mask, wall.outline_px().astype(np.int32), 255)
    mask &= wall_mask
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= min_area_px]
    if not cnts:
        return None, mask
    return max(cnts, key=cv2.contourArea), mask


def polygon_centroid(poly):
    m = cv2.moments(poly.astype(np.float32).reshape(-1, 1, 2))
    if abs(m["m00"]) < 1e-9:
        return poly.mean(0)
    return np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])


def measure_once(img, ref_img, wall, args):
    if args.pixel is not None:
        uv = np.array([args.pixel], float)
        xy = wall.to_wall(uv, args.gap_mm)[0]
        return dict(x=xy[0], y=xy[1], width=np.nan, height=np.nan, uv=uv[0], contour=None)
    if ref_img is None:
        sys.exit("No reference image. Run:  python3 bench.py reference   (with the wall empty)")
    cnt, _ = find_object(ref_img, img, wall, args.min_area, args.threshold)
    if cnt is None:
        raise RuntimeError("No object found (nothing differs from the reference image). "
                           "Lower --threshold or check the object is in front of the wall.")
    pts = cnt.reshape(-1, 2).astype(np.float64)
    if wall.model_valid(pts).mean() < 0.99:
        print("WARNING: part of the object lies where the lens model is unreliable; "
              "move it toward the image centre or recalibrate (recalibrate.py).")
    wall_pts = wall.to_wall(pts, args.gap_mm)
    c = polygon_centroid(wall_pts)
    return dict(x=c[0], y=c[1],
                width=float(np.ptp(wall_pts[:, 0])), height=float(np.ptp(wall_pts[:, 1])),
                uv=pts.mean(0), contour=cnt)


def draw(img, wall, res, text, path):
    vis = img.copy()
    cv2.polylines(vis, [wall.outline_px().astype(np.int32)], True, (255, 160, 0), 4)
    for i, c in wall.img_corners.items():
        cv2.polylines(vis, [c.astype(np.int32)], True, (0, 200, 0), 4)
        cv2.putText(vis, str(i), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 200, 0), 4)
    if res is not None:
        if res.get("contour") is not None:
            cv2.drawContours(vis, [res["contour"]], -1, (0, 0, 255), 5)
        u, v = res["uv"].astype(int)
        cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 80, 5)
    y = 90
    for line in text.splitlines():
        cv2.putText(vis, line, (40, y), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 5)
        y += 80
    vis = cv2.resize(vis, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(path), vis, [cv2.IMWRITE_JPEG_QUALITY, 90])


# ---------------------------------------------------------------- commands
def common_setup(args):
    K, dist, calib_size = load_calibration(args.calib)
    board_json, board_pts, outline = load_board(args.board)
    return K, dist, calib_size, board_json, board_pts, outline


def get_image(args, tag):
    if args.image:
        return load_image(args.image, not args.no_rotate), args.image
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = OUT_DIR / f"{tag}_{stamp}.jpg"
    capture(path, args.lens_position)
    return load_image(path, not args.no_rotate), str(path)


def cmd_set_scale(args):
    with open(args.board) as f:
        b = json.load(f)
    nominal = b["scale_bars_mm"]["nominal_length"]
    b["scale_x"] = args.h_mm / nominal
    b["scale_y"] = args.v_mm / nominal
    with open(args.board, "w") as f:
        json.dump(b, f, indent=2)
    print(f"Saved print scale: X {b['scale_x']:.5f}, Y {b['scale_y']:.5f}  ->  {args.board}")


def cmd_reference(args):
    K, dist, cs, bj, bp, outline = common_setup(args)
    img, src = get_image(args, "reference")
    check_size(img, K, cs)
    wall = Wall(img, bj, bp, outline, K, dist)
    WORK.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(REFERENCE_IMG), img, [cv2.IMWRITE_JPEG_QUALITY, 97])
    json.dump(dict(source=src, distance_mm=wall.distance_mm, markers=wall.ids,
                   time=dt.datetime.now().isoformat(timespec="seconds")),
              open(REFERENCE_META, "w"), indent=2)
    print(wall.summary())
    draw(img, wall, None, "REFERENCE (empty wall)", WORK / "reference_preview.jpg")
    print(f"Reference saved: {REFERENCE_IMG}")
    print(f"Preview: {WORK / 'reference_preview.jpg'}")
    print("Do not move the camera or the wall until you finish measuring in this setup.")


def cmd_measure(args):
    K, dist, cs, bj, bp, outline = common_setup(args)
    if args.ppixel is not None:
        args.pixel = [2.0 * args.ppixel[0], 2.0 * args.ppixel[1]]
    ref_img = load_image(REFERENCE_IMG, rotate=False) if REFERENCE_IMG.exists() else None
    results, last = [], None
    for k in range(args.frames):
        img, src = get_image(args, "measure")
        check_size(img, K, cs)
        wall = Wall(img, bj, bp, outline, K, dist)
        if ref_img is not None and ref_img.shape != img.shape:
            sys.exit("Reference image size differs; re-run reference.")
        res = measure_once(img, ref_img, wall, args)
        # Sensitivity of the result to a 1 mm error in the gap you entered.
        uv = res["uv"].reshape(1, 2)
        d1 = wall.to_wall(uv, args.gap_mm + 1.0)[0] - wall.to_wall(uv, args.gap_mm)[0]
        res["gap_sens"] = float(np.linalg.norm(d1))
        results.append(res)
        last = (img, wall, res)
        print(f"  frame {k + 1}/{args.frames}: X={res['x']:.3f} mm  Y={res['y']:.3f} mm")
        if args.image:
            break

    img, wall, res = last
    X = np.array([r["x"] for r in results])
    Y = np.array([r["y"] for r in results])
    sx = float(X.std(ddof=1)) if len(X) > 1 else float("nan")
    sy = float(Y.std(ddof=1)) if len(Y) > 1 else float("nan")
    print("\n" + wall.summary())
    print(f"\nRESULT  (plane {args.gap_mm:.2f} mm in front of the wall, wall origin = top-left of sheet)")
    print(f"  X = {X.mean():.3f} mm   Y = {Y.mean():.3f} mm   ({len(X)} frame(s))")
    if len(X) > 1:
        print(f"  repeatability (1 std): X {sx:.3f} mm, Y {sy:.3f} mm")
    if not np.isnan(res["width"]):
        print(f"  object extent: {res['width']:.2f} x {res['height']:.2f} mm (silhouette)")
    print(f"  gap sensitivity: 1 mm gap error moves this point by {res['gap_sens']:.3f} mm")

    text = f"X={X.mean():.2f} Y={Y.mean():.2f} mm  gap={args.gap_mm} mm"
    prev = OUT_DIR / f"measure_preview_{dt.datetime.now().strftime('%H%M%S')}.jpg"
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    draw(img, wall, res, text, prev)
    print(f"  preview: {prev}")

    new = not LOG_CSV.exists()
    with open(LOG_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "label", "gap_mm", "x_mm", "y_mm", "std_x", "std_y", "frames",
                        "width_mm", "height_mm", "gap_sens_mm_per_mm", "cam_wall_mm",
                        "wall_fit_rms_mm", "mode"])
        w.writerow([dt.datetime.now().isoformat(timespec="seconds"), args.label, args.gap_mm,
                    f"{X.mean():.4f}", f"{Y.mean():.4f}", f"{sx:.4f}", f"{sy:.4f}", len(X),
                    f"{res['width']:.3f}", f"{res['height']:.3f}", f"{res['gap_sens']:.4f}",
                    f"{wall.distance_mm:.1f}", f"{wall.fit_rms_mm:.4f}",
                    "pixel" if args.pixel is not None else "auto"])
    print(f"  logged to {LOG_CSV}")


def cmd_diagnose(args):
    """Per-marker residuals on the saved reference image: where is the wall off?"""
    K, dist, cs, bj, bp, outline = common_setup(args)
    src = args.image or str(REFERENCE_IMG)
    img = load_image(src, rotate=bool(args.image) and not args.no_rotate)
    wall = Wall(img, bj, bp, outline, K, dist)
    nominal = bj["marker_size_mm"] * float(bj.get("scale_x", 1.0))
    rvec, _ = cv2.Rodrigues(wall.R)
    print(wall.summary())
    print(f"\n id   centre(mm)      wall-fit err   pose err   marker side (mm)  [nominal {nominal:.2f}]")
    rows = []
    for i in wall.ids:
        uv = wall.img_corners[i]
        true = bp[i]
        meas = wall.to_wall(uv, 0.0)
        e_fit = np.linalg.norm(meas - true, axis=1).mean()
        obj3 = np.hstack([true, np.zeros((4, 1))])
        pr, _ = cv2.projectPoints(obj3, rvec, wall.t, K, dist)
        e_pose = np.linalg.norm(pr.reshape(-1, 2) - uv, axis=1).mean()
        sides = np.linalg.norm(np.roll(meas, -1, 0) - meas, axis=1)
        c = true.mean(0)
        rows.append((i, c, e_fit, e_pose, sides))
        flag = "  <-- high" if e_fit > 2.5 * wall.fit_rms_mm and e_fit > 0.15 else ""
        print(f" {i:2d}  ({c[0]:6.1f},{c[1]:6.1f})   {e_fit:6.3f} mm   {e_pose:6.2f} px   "
              f"{sides.mean():6.2f} (min {sides.min():.2f}, max {sides.max():.2f}){flag}")
    # Is the error a smooth bend (edges vs centre) or isolated markers?
    cs_ = np.array([r[1] for r in rows])
    ef = np.array([r[2] for r in rows])
    d = np.linalg.norm(cs_ - cs_.mean(0), axis=1)
    corr = np.corrcoef(d, ef)[0, 1] if len(ef) > 2 else float("nan")
    side_err = np.array([r[4].mean() for r in rows]) - nominal
    print(f"\nmean marker side error vs nominal: {side_err.mean():+.3f} mm "
          f"({side_err.mean() / nominal * 100:+.2f} %)")
    print(f"error vs distance-from-centre correlation: {corr:+.2f} "
          "(near +1 = sheet bowed/curled; near 0 = local bumps or individual markers)")

    vis = img.copy()
    for (i, c, e_fit, e_pose, _) in rows:
        uv = wall.img_corners[i]
        meas_c = uv.mean(0)
        col = (0, 0, 255) if e_fit > 2.5 * wall.fit_rms_mm and e_fit > 0.15 else (0, 200, 0)
        cv2.polylines(vis, [uv.astype(np.int32)], True, col, 5)
        cv2.putText(vis, f"{i}:{e_fit:.2f}mm", tuple((meas_c + [-120, 0]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, col, 4)
    out = WORK / "diagnose_preview.jpg"
    vis = cv2.resize(vis, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(out), vis, [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(f"preview: {out}")


def cmd_snap(args):
    """Upright half-size photo with a pixel grid, for picking a point (--ppixel)."""
    img, src = get_image(args, "snap")
    vis = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    h, w = vis.shape[:2]
    for x in range(0, w, 50):
        major = x % 100 == 0
        cv2.line(vis, (x, 0), (x, h), (0, 200, 255) if major else (0, 120, 160), 1)
        if major:
            for y0 in range(20, h, 400):
                cv2.putText(vis, str(x), (x + 3, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    for y in range(0, h, 50):
        major = y % 100 == 0
        cv2.line(vis, (0, y), (w, y), (0, 200, 255) if major else (0, 120, 160), 1)
        if major:
            for x0 in range(3, w, 300):
                cv2.putText(vis, str(y), (x0, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    clean = WORK / "snap.jpg"
    grid = WORK / "snap_grid.jpg"
    WORK.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(clean), cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA),
                [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(grid), vis, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f"Saved {clean} (plain) and {grid} (with pixel grid).")
    print("Open snap.jpg in Paint: the pixel position of the mouse is shown at the bottom-left.")
    print("Then: python3 bench.py measure --ppixel U V --gap-mm G ...")


def cmd_validate(args):
    K, dist, cs, bj, bp, outline = common_setup(args)
    img, src = get_image(args, "validate")
    check_size(img, K, cs)
    wall = Wall(img, bj, bp, outline, K, dist)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cols, rows = args.pattern
    found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=cv2.CALIB_CB_ACCURACY)
    if not found:
        sys.exit("Checkerboard not detected. Make it fully visible and flat.")
    meas = wall.to_wall(corners.reshape(-1, 2), args.gap_mm)
    ideal = np.zeros((rows * cols, 2))
    ideal[:, 0] = np.tile(np.arange(cols), rows) * args.square_mm
    ideal[:, 1] = np.repeat(np.arange(rows), cols) * args.square_mm

    def fit(src_pts, dst_pts, allow_scale):
        mu_s, mu_d = src_pts.mean(0), dst_pts.mean(0)
        A, B = src_pts - mu_s, dst_pts - mu_d
        U, S, Vt = np.linalg.svd(A.T @ B)
        d = np.sign(np.linalg.det(U @ Vt))
        D = np.diag([1, d])
        Rm = (U @ D @ Vt).T
        s = (S * np.diag(D)).sum() / (A ** 2).sum() if allow_scale else 1.0
        return s * (A @ Rm.T) + mu_d, s

    rigid, _ = fit(ideal, meas, False)
    similar, scale = fit(ideal, meas, True)
    err_s = np.linalg.norm(similar - meas, axis=1)
    err = np.linalg.norm(rigid - meas, axis=1)
    span = np.linalg.norm(meas[0] - meas[-1])
    span_ideal = np.linalg.norm(ideal[0] - ideal[-1])
    print(wall.summary())
    print(f"\nVALIDATION  ({rows * cols} checkerboard corners, square {args.square_mm} mm, "
          f"plane {args.gap_mm} mm in front of wall)")
    print(f"  position error after best rigid placement: RMS {np.sqrt(np.mean(err ** 2)):.3f} mm, "
          f"mean {err.mean():.3f} mm, max {err.max():.3f} mm")
    print(f"  shape error with scale removed:            RMS {np.sqrt(np.mean(err_s ** 2)):.3f} mm, "
          f"max {err_s.max():.3f} mm")
    print(f"  scale error: {(scale - 1) * 100:+.3f} %  "
          f"(diagonal measured {span:.2f} mm vs {span_ideal:.2f} mm)")
    print("  Error includes the checkerboard print accuracy: use a vernier-measured --square-mm.")
    prev = OUT_DIR / "validate_preview.jpg"
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    vis = img.copy()
    cv2.drawChessboardCorners(vis, (cols, rows), corners, True)
    draw(vis, wall, None, f"validate RMS {np.sqrt(np.mean(err ** 2)):.3f} mm", prev)
    print(f"  preview: {prev}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calib", default=str(DEFAULT_CALIB))
    p.add_argument("--board", default=str(DEFAULT_BOARD))
    sub = p.add_subparsers(dest="cmd", required=True)

    def img_opts(sp):
        sp.add_argument("--image", help="process this saved image instead of capturing")
        sp.add_argument("--no-rotate", action="store_true", help="image is already rotated")
        sp.add_argument("--lens-position", type=float, default=None,
                        help="manual focus (dioptres, ~100/distance_cm); default autofocus")

    s = sub.add_parser("set-scale", help="store measured length of the printed 100 mm bars")
    s.add_argument("--h-mm", type=float, required=True)
    s.add_argument("--v-mm", type=float, required=True)

    r = sub.add_parser("reference", help="capture the empty wall for this setup")
    img_opts(r)

    m = sub.add_parser("measure", help="measure the object position")
    img_opts(m)
    m.add_argument("--gap-mm", type=float, default=0.0, help="object distance in front of wall surface")
    m.add_argument("--frames", type=int, default=5, help="captures to average")
    m.add_argument("--pixel", type=float, nargs=2, metavar=("U", "V"),
                   help="measure this pixel (rotated image coords) instead of auto-detecting")
    m.add_argument("--ppixel", type=float, nargs=2, metavar=("U", "V"),
                   help="point picked on snap.jpg (half-size upright image); measured instead of auto-detect")
    m.add_argument("--threshold", type=float, default=30.0, help="grey-level change counted as object")
    m.add_argument("--min-area", type=float, default=2000.0, help="smallest object area in pixels")
    m.add_argument("--label", default="", help="note stored in the CSV log")

    v = sub.add_parser("validate", help="checkerboard accuracy test")
    img_opts(v)
    v.add_argument("--gap-mm", type=float, default=0.0)
    v.add_argument("--square-mm", type=float, default=24.0)
    v.add_argument("--pattern", type=int, nargs=2, default=[9, 6], metavar=("COLS", "ROWS"))

    sn = sub.add_parser("snap", help="photo for picking a point by hand (use with measure --ppixel)")
    img_opts(sn)

    dg = sub.add_parser("diagnose", help="per-marker error map of the saved reference image")
    dg.add_argument("--image", help="raw image to check instead of reference.jpg")
    dg.add_argument("--no-rotate", action="store_true")

    args = p.parse_args()
    try:
        {"set-scale": cmd_set_scale, "reference": cmd_reference, "diagnose": cmd_diagnose, "snap": cmd_snap,
         "measure": cmd_measure, "validate": cmd_validate}[args.cmd](args)
    except RuntimeError as e:
        sys.exit(f"ERROR: {e}")


if __name__ == "__main__":
    main()
