#!/usr/bin/env python3
"""Closed-loop control of the motorized stage: command -> move -> camera check -> correct.

The Arduino (stage_control.ino) moves the motors; this program, on the Raspberry Pi,
sends it commands over USB and uses the camera (track.py) to measure what really
happened, then sends a correction if the error is bigger than the tolerance.

Run in ~/camera_calibration/bench  (Arduino USB cable plugged into the Pi):
  python3 stage.py status
  python3 stage.py home
  python3 stage.py calibrate-axis --id 30 --dist 50 --lens-position 1.45   # once per setup
  python3 stage.py move 20 --id 30 --lens-position 1.45                    # closed loop
  python3 stage.py align --id 30 --ref 31 --lens-position 1.45             # match heights
  python3 stage.py set 50 --id 30 --base 45 --lens-position 1.45      # tag 30 to 50 mm above BASE
  python3 stage.py heights --ids 30 31 --base 45 --lens-position 1.45  # where is everything
  python3 stage.py rot 90
  python3 stage.py shell --id 30 --lens-position 1.45                      # interactive

"Axis" = the direction the carriage really moves, as the camera sees it on the wall.
calibrate-axis measures it once (move +dist, see where the tag went), so later
moves are checked along that direction even if the stage is not exactly vertical
in the wall frame. It also tells you the real mm-per-commanded-mm.
"""
import argparse
import csv
import datetime as dt
import glob
import json
import sys
import time

import numpy as np

import bench
import track

AXIS_FILE = bench.WORK / "stage_axis.json"
LOG_CSV = bench.WORK / "stage_log.csv"
REPLY_TAGS = ("OK", "ERR", "LIMIT", "STOPPED", "STATUS")


# ---------------------------------------------------------------- Arduino link
class Stage:
    def __init__(self, port=None, baud=115200):
        import serial                                  # pip: pyserial
        if port is None:
            ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if not ports:
                sys.exit("Arduino not found (no /dev/ttyACM* or /dev/ttyUSB*). Is the USB cable in the Pi?")
            port = ports[0]
        # Opening the port normally RESETS the Mega (it forgets HOME). Prevent that:
        # turn off "hang-up on close" and keep DTR low when opening.
        import subprocess
        subprocess.run(["stty", "-F", port, "-hupcl"], capture_output=True)
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.5
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        t0 = time.time()                               # if it did reset, wait for its banner
        while time.time() - t0 < 2.5:
            line = self.ser.readline().decode(errors="replace").strip()
            if "ready" in line:
                print("  (Arduino restarted - run HOME again)")
                time.sleep(0.3)
                break
            if not line and time.time() - t0 > 0.8:
                break
        self.ser.reset_input_buffer()
        print(f"Arduino on {port}")

    def cmd(self, text, timeout=180, quiet=False):
        """Send one command, wait for its final reply line. Returns (tag, fields, line)."""
        self.ser.write((text.strip() + "\n").encode())
        t0 = time.time()
        while time.time() - t0 < timeout:
            line = self.ser.readline().decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                if not quiet:
                    print("  " + line)
                continue
            tag = line.split()[0]
            if tag in REPLY_TAGS:
                fields = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)
                if not quiet:
                    print("  arduino: " + line)
                return tag, fields, line
        raise TimeoutError(f"no reply to '{text}' within {timeout}s")

    def close(self):
        self.ser.close()


# ---------------------------------------------------------------- camera side
class Vision:
    def __init__(self, args):
        args.no_blur_fix = getattr(args, "no_blur_fix", False)
        args.gap_mm = None
        args._setup = bench.common_setup(args)
        self.args = args
        self.comp = track.load_components()
        self.cam = track.Camera(args.lens_position)

    def measure(self, ids, frames=5, discard=2):
        """Median (x, y, gap) in wall mm for each tag id over `frames` fresh frames."""
        for _ in range(discard):                       # drop frames taken during the move
            self.cam.grab()
        got = {i: [] for i in ids}
        for _ in range(frames):
            img = self.cam.grab()
            try:
                _, res = track.measure_frame(img, self.args, self.comp)
            except RuntimeError as e:
                print(f"  (frame skipped: {e})")
                continue
            for r in res:
                if r["id"] in got:
                    got[r["id"]].append((r["x"], r["y"], r["gap"]))
        out = {}
        for i, v in got.items():
            if not v:
                raise RuntimeError(f"tag {i} not seen by the camera")
            a = np.array(v)
            out[i] = (np.median(a, 0), len(v), float(np.max(np.std(a[:, :2], 0))) if len(v) > 1 else 0.0)
        return out

    def close(self):
        self.cam.close()


# ---------------------------------------------------------------- helpers
def load_axis():
    if not AXIS_FILE.exists():
        sys.exit("No axis calibration yet. Run:  python3 stage.py calibrate-axis --id <tag> --dist 50")
    return json.load(open(AXIS_FILE))


def log_row(**kw):
    new = not LOG_CSV.exists()
    with open(LOG_CSV, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["time", "action", "tag", "commanded_mm", "iteration",
                                          "sent_mm", "measured_mm", "error_mm", "stage_Z", "note"])
        if new:
            w.writeheader()
        w.writerow({"time": dt.datetime.now().isoformat(timespec="seconds"), **kw})


AUTO_CORRECT = False   # set by --auto: apply corrections without asking


def confirm_correction(err):
    """Show the error and ask the operator before sending a correction move."""
    global AUTO_CORRECT
    if AUTO_CORRECT:
        return True
    while True:
        ans = input(f"  Error is {err:+.3f} mm. Send correction move of {err:+.3f} mm? "
                    f"[y = yes, n = no, a = yes to all] ").strip().lower()
        if ans in ("y", "yes", ""):
            return True
        if ans in ("n", "no"):
            print("  Correction skipped by operator.")
            return False
        if ans in ("a", "all"):
            AUTO_CORRECT = True
            return True


def closed_loop_move(stage, vis, tid, dist, axis, tol=0.1, max_iter=3, frames=5, settle=0.6):
    """Move the carriage `dist` mm along the axis and verify with the camera."""
    u = np.array(axis["u"], float)
    p0 = vis.measure([tid], frames)[tid][0]
    print(f"Start: tag {tid} at X={p0[0]:.2f} Y={p0[1]:.2f} (wall mm)")
    send = dist
    err = dist
    for it in range(1, max_iter + 1):
        tag, f, _ = stage.cmd(f"MOVE {send:.3f}")
        if tag != "OK":
            print(f"Stopped: Arduino answered {tag}")
            log_row(action="move", tag=tid, commanded_mm=dist, iteration=it, sent_mm=round(send, 3),
                    stage_Z=f.get("Z"), note=tag)
            return None
        time.sleep(settle)
        m, n, sd = vis.measure([tid], frames)[tid]
        achieved = float(np.dot(m[:2] - p0[:2], u))
        err = dist - achieved
        print(f"  check {it}: Arduino (step count) Z = {f.get('Z')} mm | camera measured move "
              f"{achieved:+.3f} mm of {dist:+.3f}  ->  error {err:+.3f} mm  (frame spread {sd:.3f})")
        log_row(action="move", tag=tid, commanded_mm=dist, iteration=it, sent_mm=round(send, 3),
                measured_mm=round(achieved, 3), error_mm=round(err, 3), stage_Z=f.get("Z"))
        if abs(err) <= tol:
            print(f"DONE: within {tol} mm after {it} move(s). Final error {err:+.3f} mm")
            return err
        if it == max_iter:
            break
        if not confirm_correction(err):
            log_row(action="move", tag=tid, commanded_mm=dist, iteration=it, error_mm=round(err, 3),
                    note="correction declined")
            return err
        send = err
    print(f"Not within {tol} mm after {max_iter} moves (last error {err:+.3f} mm). "
          f"Check backlash, loose nut/flag, or tag visibility.")
    return err


def height_above_base(vis, ids, base, u, frames=5):
    """Height of each tag above the BASE tag, measured along the stage axis (mm)."""
    m = vis.measure(list(ids) + [base], frames)
    pb = m[base][0]
    return {i: float(np.dot(m[i][0][:2] - pb[:2], u)) for i in ids}, m


def closed_loop_set(stage, vis, tid, base, target, axis, tol=0.1, max_iter=4, frames=5, settle=0.6):
    """Put tag `tid` at `target` mm above the BASE tag (absolute, camera-referenced)."""
    u = np.array(axis["u"], float)
    err = None
    for it in range(0, max_iter + 1):
        h = height_above_base(vis, [tid], base, u, frames)[0][tid]
        err = target - h
        print(f"  {'start' if it == 0 else f'check {it}'}: tag {tid} is {h:.3f} mm above base "
              f"-> error {err:+.3f} mm")
        log_row(action="set", tag=tid, commanded_mm=target, iteration=it, measured_mm=round(h, 3),
                error_mm=round(err, 3))
        if abs(err) <= tol:
            print(f"DONE: tag {tid} at {h:.3f} mm above base (within {tol} mm)")
            return err
        if it == max_iter:
            break
        if it > 0 and not confirm_correction(err):
            log_row(action="set", tag=tid, commanded_mm=target, iteration=it, error_mm=round(err, 3),
                    note="correction declined")
            return err
        tag, f, _ = stage.cmd(f"MOVE {err:.3f}")
        if tag != "OK":
            print(f"Stopped: Arduino answered {tag} (soft limit or switch). Target may be out of range.")
            return None
        time.sleep(settle)
    print(f"Not within {tol} mm after {max_iter} moves (last error {err:+.3f} mm)")
    return err


# ---------------------------------------------------------------- commands
def c_status(a):
    s = Stage(a.port)
    s.cmd("STATUS")
    s.close()


def c_home(a):
    s = Stage(a.port)
    s.cmd("HOME")
    s.close()


def c_rot(a):
    s = Stage(a.port)
    s.cmd(f"ROT {a.deg}")
    s.close()


def c_calibrate_axis(a):
    s, v = Stage(a.port), Vision(a)
    try:
        p0 = v.measure([a.id], a.frames)[a.id][0]
        tag, f, _ = s.cmd(f"MOVE {a.dist}")
        if tag != "OK":
            sys.exit(f"Move failed ({tag}). Home first and make sure there is {a.dist} mm of room above.")
        time.sleep(0.6)
        p1 = v.measure([a.id], a.frames)[a.id][0]
        d = p1[:2] - p0[:2]
        L = float(np.linalg.norm(d))
        if L < 0.3 * abs(a.dist):
            sys.exit(f"Tag moved only {L:.2f} mm on the wall - is tag {a.id} on the moving carriage?")
        u = d / L * np.sign(a.dist)
        tilt = float(np.degrees(np.arctan2(u[0], -u[1])))
        json.dump({"u": [float(u[0]), float(u[1])], "scale": L / abs(a.dist), "tag": a.id,
                   "dist": a.dist, "tilt_deg": tilt, "date": dt.datetime.now().isoformat(timespec="seconds")},
                  open(AXIS_FILE, "w"), indent=2)
        print(f"\nCommanded {a.dist:+.2f} mm, camera saw {L:.3f} mm  (scale {L / abs(a.dist):.4f})")
        print(f"Axis direction on the wall: u = ({u[0]:+.4f}, {u[1]:+.4f}); "
              f"tilt from wall vertical = {tilt:+.2f} deg")
        print(f"Saved {AXIS_FILE}")
        log_row(action="calibrate-axis", tag=a.id, commanded_mm=a.dist, measured_mm=round(L, 3),
                error_mm=round(a.dist - L * np.sign(a.dist), 3), stage_Z=f.get("Z"))
        if not a.stay:
            s.cmd(f"MOVE {-a.dist}")
    finally:
        v.close()
        s.close()


def c_move(a):
    axis = load_axis()
    s, v = Stage(a.port), Vision(a)
    try:
        closed_loop_move(s, v, a.id, a.mm, axis, a.tol, a.max_iter, a.frames)
    finally:
        v.close()
        s.close()


def align(s, v, tid, ref, axis, tol, max_iter, frames):
    u = np.array(axis["u"], float)
    m = v.measure([tid, ref], frames)
    pt, pr = m[tid][0], m[ref][0]
    delta = float(np.dot(pr[:2] - pt[:2], u))
    print(f"Tag {tid} is {-delta:+.3f} mm from reference tag {ref} along the stage axis "
          f"(sideways offset {float(u[0] * (pr[1] - pt[1]) - u[1] * (pr[0] - pt[0])):+.2f} mm, not correctable by this axis)")
    if abs(delta) <= tol:
        print("Already aligned.")
        return 0.0
    print(f"Moving {delta:+.3f} mm to align...")
    return closed_loop_move(s, v, tid, delta, axis, tol, max_iter, frames)


def c_align(a):
    axis = load_axis()
    s, v = Stage(a.port), Vision(a)
    try:
        align(s, v, a.id, a.ref, axis, a.tol, a.max_iter, a.frames)
    finally:
        v.close()
        s.close()


def c_set(a):
    axis = load_axis()
    s, v = Stage(a.port), Vision(a)
    try:
        closed_loop_set(s, v, a.id, a.base, a.mm, axis, a.tol, a.max_iter, a.frames)
    finally:
        v.close()
        s.close()


def c_heights(a):
    axis = load_axis()
    v = Vision(a)
    try:
        hs, m = height_above_base(v, a.ids, a.base, np.array(axis["u"]), a.frames)
        for i, h in hs.items():
            print(f"  tag {i}: {h:8.3f} mm above base  (X={m[i][0][0]:.2f} Y={m[i][0][1]:.2f}, spread {m[i][2]:.3f})")
    finally:
        v.close()


def c_shell(a):
    """Interactive: camera stays on, type commands like a ground station."""
    s, v = Stage(a.port), Vision(a)
    axis = json.load(open(AXIS_FILE)) if AXIS_FILE.exists() else None
    print("\nCommands: set <mm above base> | heights | move <mm> | goto <mm> | align <ref_id> | rot <deg> |"
          " home | status | where | raw <arduino command> | quit")
    try:
        while True:
            try:
                line = input("stage> ").strip()
            except EOFError:
                break
            if not line:
                continue
            w = line.split()
            c = w[0].lower()
            try:
                if c in ("quit", "exit", "q"):
                    break
                elif c == "move" and len(w) == 2:
                    if axis is None:
                        print("Run calibrate-axis first (outside the shell)."); continue
                    closed_loop_move(s, v, a.id, float(w[1]), axis, a.tol, a.max_iter, a.frames)
                elif c == "goto" and len(w) == 2:
                    _, f, _ = s.cmd("STATUS", quiet=True)
                    if f.get("homed") != "1":
                        print("HOME first."); continue
                    closed_loop_move(s, v, a.id, float(w[1]) - float(f["Z"]), axis, a.tol, a.max_iter, a.frames)
                elif c == "set" and len(w) == 2:
                    closed_loop_set(s, v, a.id, a.base, float(w[1]), axis, a.tol, a.max_iter, a.frames)
                elif c == "heights":
                    hs, _ = height_above_base(v, [a.id], a.base, np.array(axis["u"]), a.frames)
                    for i, h in hs.items():
                        print(f"  tag {i}: {h:.3f} mm above base")
                elif c == "align" and len(w) == 2:
                    align(s, v, a.id, int(w[1]), axis, a.tol, a.max_iter, a.frames)
                elif c == "rot" and len(w) == 2:
                    s.cmd(f"ROT {float(w[1])}")
                elif c == "home":
                    s.cmd("HOME")
                elif c == "status":
                    s.cmd("STATUS")
                elif c == "where":
                    for i, (m, n, sd) in v.measure([a.id], a.frames).items():
                        print(f"  tag {i}: X={m[0]:.2f} Y={m[1]:.2f} gap={m[2]:.1f} mm ({n} frames, spread {sd:.3f})")
                elif c == "raw" and len(w) > 1:
                    s.cmd(" ".join(w[1:]))
                else:
                    print("?  move <mm> | goto <mm> | align <ref_id> | rot <deg> | home | status | where | raw ... | quit")
            except (RuntimeError, TimeoutError) as e:
                print(f"  problem: {e}")
    finally:
        v.close()
        s.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="serial port (default: auto /dev/ttyACM*)")
    p.add_argument("--calib", default=str(bench.DEFAULT_CALIB))
    p.add_argument("--board", default=str(bench.DEFAULT_BOARD))
    sub = p.add_subparsers(dest="cmd", required=True)

    def vis_opts(q):
        q.add_argument("--id", type=int, default=30, help="tag on the rod / component being positioned")
        q.add_argument("--lens-position", type=float, default=None)
        q.add_argument("--frames", type=int, default=5, help="frames averaged per measurement")
        q.add_argument("--tol", type=float, default=0.1, help="accepted error, mm")
        q.add_argument("--max-iter", type=int, default=3, help="max correction moves")
        q.add_argument("--base", type=int, default=45, help="BASE tag fixed at the bench zero mark")
        q.add_argument("--auto", action="store_true", help="apply corrections without asking")

    sub.add_parser("status")
    sub.add_parser("home")
    r = sub.add_parser("rot"); r.add_argument("deg", type=float)
    ca = sub.add_parser("calibrate-axis"); vis_opts(ca)
    ca.add_argument("--dist", type=float, default=50.0)
    ca.add_argument("--stay", action="store_true", help="do not move back afterwards")
    mv = sub.add_parser("move"); mv.add_argument("mm", type=float); vis_opts(mv)
    al = sub.add_parser("align"); vis_opts(al); al.add_argument("--ref", type=int, required=True)
    sh = sub.add_parser("shell"); vis_opts(sh)
    st = sub.add_parser("set"); st.add_argument("mm", type=float); vis_opts(st)
    hg = sub.add_parser("heights"); vis_opts(hg)
    hg.add_argument("--ids", type=int, nargs="+", default=[30])
    a = p.parse_args()
    global AUTO_CORRECT
    AUTO_CORRECT = getattr(a, "auto", False)
    {"status": c_status, "home": c_home, "rot": c_rot, "calibrate-axis": c_calibrate_axis,
     "move": c_move, "align": c_align, "shell": c_shell, "set": c_set, "heights": c_heights}[a.cmd](a)


if __name__ == "__main__":
    main()
