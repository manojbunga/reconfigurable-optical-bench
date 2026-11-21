"""Render synthetic camera images with known geometry and check bench.py."""
import json, subprocess, sys
import cv2, numpy as np

W, H = 2592, 4608                      # rotated (portrait) frame, as in calibration
K = np.array([[3434.05, 0, 1400.05], [0, 3454.36, 2251.15], [0, 0, 1]])
DIST = np.array([0.03, -0.06, 0.0008, 0.0011, 0.0])
np.savez("synth_calib.npz", camera_matrix=K, dist_coeffs=DIST)

board = json.load(open("wall_board_wall300.json"))
PX_PER_MM = 12
tex = np.full((int(300 * PX_PER_MM), int(300 * PX_PER_MM)), 235, np.uint8)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for m in board["markers"]:
    s = int(round(board["marker_size_mm"] * PX_PER_MM))
    im = cv2.aruco.generateImageMarker(d, m["id"], s, borderBits=1)
    x, y = [int(round(v * PX_PER_MM)) for v in m["top_left"]]
    tex[y:y + s, x:x + s] = np.where(im < 128, 25, 235)

# precompute pixel rays (normalised, undistorted)
uu, vv = np.meshgrid(np.arange(W), np.arange(H))
pix = np.stack([uu.ravel(), vv.ravel()], 1).astype(np.float64).reshape(-1, 1, 2)
RAYS = cv2.undistortPoints(pix, K, DIST).reshape(-1, 2)
RAYS = np.hstack([RAYS, np.ones((len(RAYS), 1))])


def checker_tex(square_mm=24.0, cols=9, rows=6):
    n_c, n_r = cols + 1, rows + 1
    t = np.zeros((int(n_r * square_mm * PX_PER_MM), int(n_c * square_mm * PX_PER_MM)), np.uint8)
    q = square_mm * PX_PER_MM
    for r in range(n_r):
        for c in range(n_c):
            t[int(r * q):int((r + 1) * q), int(c * q):int((c + 1) * q)] = 25 if (r + c) % 2 == 0 else 235
    return t


def render(D, tilt_deg, obj=None, checker=None, seed=0):
    """Camera at distance D in front of wall centre, rotated tilt about Y.
    obj = (x, y, r, gap) disk; checker = (x0, y0, gap) top-left of checker texture."""
    a = np.radians(tilt_deg)
    Rwc = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])  # cam->world
    C = np.array([150.0, 150.0, 0.0]) - Rwc[:, 2] * D
    dirs = RAYS @ Rwc.T
    out = np.full(len(dirs), 120, np.float32)

    def hit(z):
        s = (z - C[2]) / dirs[:, 2]
        return C[:2] + s[:, None] * dirs[:, :2]

    p = hit(0.0)
    ix, iy = (p[:, 0] * PX_PER_MM).astype(int), (p[:, 1] * PX_PER_MM).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < tex.shape[1]) & (iy < tex.shape[0])
    out[ok] = tex[iy[ok], ix[ok]]
    if checker is not None:
        ct = checker_tex()
        x0, y0, g = checker
        p = hit(-g) - [x0 - 24.0, y0 - 24.0]     # texture includes one outer square
        ix, iy = (p[:, 0] * PX_PER_MM).astype(int), (p[:, 1] * PX_PER_MM).astype(int)
        ok = (ix >= 0) & (iy >= 0) & (ix < ct.shape[1]) & (iy < ct.shape[0])
        out[ok] = ct[iy[ok], ix[ok]]
    if obj is not None:
        x, y, r, g = obj
        p = hit(-g)
        inside = np.hypot(p[:, 0] - x, p[:, 1] - y) < r
        out[inside] = 70
    img = out.reshape(H, W)
    img = cv2.GaussianBlur(img, (0, 0), 1.0)
    rng = np.random.default_rng(seed)
    img = np.clip(img + rng.normal(0, 2.0, img.shape), 0, 255).astype(np.uint8)
    raw = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)   # what rpicam-still would save
    return cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)


def run(*a):
    r = subprocess.run([sys.executable, "bench.py", "--calib", "synth_calib.npz",
                        "--board", "wall_board_wall300.json", *a], capture_output=True, text=True)
    return r.stdout + r.stderr


def grab(out, key):
    import re
    m = re.search(r"X = ([-\d.]+) mm\s+Y = ([-\d.]+)", out)
    return (float(m.group(1)), float(m.group(2))) if m else None


rows = []
for D, tilt in [(400, 0), (500, 6), (800, 3), (1000, 0)]:
    cv2.imwrite("s_ref.jpg", render(D, tilt), [cv2.IMWRITE_JPEG_QUALITY, 95])
    o = run("reference", "--image", "s_ref.jpg")
    if "ERROR" in o:
        print(D, tilt, o); continue
    wall_line = [l for l in o.splitlines() if l.startswith("Wall")]
    for (x, y, r, g) in [(150, 150, 15, 0), (90, 200, 12, 0), (200, 100, 15, 30), (110, 120, 10, 60)]:
        cv2.imwrite("s_obj.jpg", render(D, tilt, obj=(x, y, r, g), seed=1), [cv2.IMWRITE_JPEG_QUALITY, 95])
        o = run("measure", "--image", "s_obj.jpg", "--gap-mm", str(g), "--frames", "1")
        xy = grab(o, "X")
        if xy is None:
            print("FAIL", D, tilt, x, y, g, o[-600:]); continue
        err = np.hypot(xy[0] - x, xy[1] - y)
        # wrong-gap check: report as if object were on the wall
        o0 = run("measure", "--image", "s_obj.jpg", "--gap-mm", "0", "--frames", "1")
        xy0 = grab(o0, "X")
        rows.append((D, tilt, x, y, g, xy[0], xy[1], err, np.hypot(xy0[0] - x, xy0[1] - y)))
    print(D, tilt, wall_line)

print("\n D  tilt  true(x,y)  gap  ->  measured        err_mm   err_if_gap_ignored")
for r in rows:
    print(f"{r[0]:5} {r[1]:3} ({r[2]:4},{r[3]:4}) {r[4]:3}  -> ({r[5]:8.3f},{r[6]:8.3f})  {r[7]:.3f}    {r[8]:.2f}")

# checkerboard validation at gap 0 and 20
for D, g in [(500, 0), (500, 20), (800, 0)]:
    cv2.imwrite("s_val.jpg", render(D, 4, checker=(54, 90, g), seed=2), [cv2.IMWRITE_JPEG_QUALITY, 95])
    o = run("validate", "--image", "s_val.jpg", "--gap-mm", str(g))
    print(f"\nvalidate D={D} gap={g}:", [l for l in o.splitlines() if "error" in l])
