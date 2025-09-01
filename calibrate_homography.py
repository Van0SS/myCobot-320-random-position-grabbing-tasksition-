#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Calibration script to compute camera-→robot homography (H).

Workflow (manual click method)
-----------------------------
1. Connect the myCobot-280 via USB so we can read its current TCP X-Y.
2. Run this script with the same `--port` and `--baud` you use for normal
   operation.  A camera window will open.
3. For each sample point:
   • Move the robot gripper tip so it exactly touches any well-defined point
     (e.g. a chessboard corner) on the table.
   • Press **SPACE** — the script grabs the current camera frame and waits for
     you to click that exact tip position with the mouse.
   • The script records `(u,v)` from the click and `(x,y)` from
     `mc.get_coords()`.
4. Collect at least 4 samples (more is better, 8-12 typical).
5. Press **Enter** to compute `H` with `cv2.findHomography` (RANSAC) and save
   it to `env/homography.json`.

The resulting JSON has the form
{"H": [[...],[...],[...]]}

You can re-run at any time; the file will be overwritten.
"""

import argparse
import json
import os
import sys
from pathlib import Path
import time

import cv2
import numpy as np
from pymycobot import MyCobot280

# Shared robot pose targets
from robot_constants import TARGET_Z, TARGET_ORIENTATION


class Collector:
    def __init__(self, mc, out_path: Path):
        self.mc = mc
        self.out_path = out_path
        self.pixel_pts: list[list[float]] = []
        self.robot_pts: list[list[float]] = []
        self.window = "Calibration"
        self.is_free = False  # if robot is in free mode
        # cache of last printed coords to avoid spamming the console
        self._last_coords_print: list[float] | None = None

    # ------------------------------------------------------------------
    # Helper utilities
    # ------------------------------------------------------------------

    def _current_coords(self) -> list[float] | None:
        """Return the current TCP coordinates \[x, y, z, rx, ry, rz\] or ``None`` on error."""
        try:
            coords = self.mc.get_coords()
        except Exception:
            return None
        if not coords or isinstance(coords, int) or len(coords) < 6:
            return None
        return [float(c) for c in coords]

    def _overlay_coords(self, img):
        """Draw current XYZ + orientation on *img* and print to console when changed."""
        coords = self._current_coords()
        if coords is None:
            return

        labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for i, (lbl, val) in enumerate(zip(labels, coords)):
            y = 30 + i * 25

            # Determine reference and check deviation for Z & orientation
            warn = False
            if i == 2:  # Z index
                warn = abs(val - TARGET_Z) > 2.0
            elif i >= 3:  # orientation indices 3,4,5
                ref_val = TARGET_ORIENTATION[i - 3]
                warn = abs(val - ref_val) > 2.0

            suffix = " !" if warn else ""
            text = f"{lbl}: {val:.1f}{' mm' if i < 3 else '°'}{suffix}"

            cv2.putText(
                img,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),  # white text
                2,
                cv2.LINE_AA,
            )

        # Print to console only if any component changed more than a small tolerance
        tol_pos = 0.5  # mm
        tol_ang = 0.5  # deg
        if self._last_coords_print is None or any(
            abs(c - p) > (tol_pos if idx < 3 else tol_ang)
            for idx, (c, p) in enumerate(zip(coords, self._last_coords_print))
        ):
            coord_str = " ".join(f"{lbl}={val:.1f}" for lbl, val in zip(labels, coords))
            print(f"\r{coord_str}        ", end="", flush=True)
            self._last_coords_print = coords

    def _mouse_cb(self, event, x, y, _flags, _param):  # noqa: N803
        if event == cv2.EVENT_LBUTTONDOWN:
            coords = self.mc.get_coords()
            if not coords or isinstance(coords, int):
                print("[WARN] Robot did not return valid coords (got: {}); try again.".format(coords))
                return
            rx, ry = float(coords[0]), float(coords[1])
            self.pixel_pts.append([float(x), float(y)])
            self.robot_pts.append([float(rx), float(ry)])
            print(f"  • Sample #{len(self.pixel_pts)}: pixel=({x},{y}) ➜ robot=({rx:.1f},{ry:.1f})")

    def run(self, cam_index: int):
        cap = cv2.VideoCapture(cam_index)
        if not cap.isOpened():
            print(f"[ERR] Cannot open camera index {cam_index}")
            sys.exit(1)

        cv2.namedWindow(self.window)
        cv2.setMouseCallback(self.window, self._mouse_cb)

        print("\nInstructions:\n"
              "  • Move the gripper tip to the table point you wish to sample.\n"
              "  • Press F to toggle **free mode** (servos off / on).\n"
              "  • While servos are on, press SPACE, then click the tip in the paused image.\n"
              "  • Collect ≥4 samples, then press ENTER to solve.\n"
              "  • Press ESC to quit without saving.\n")

        paused_img = None
        while True:
            if paused_img is None:
                ret, frame = cap.read()
                if not ret:
                    print("[ERR] Failed to grab frame")
                    break
                # Always overlay and show current full coordinates
                self._overlay_coords(frame)
                cv2.imshow(self.window, frame)
            else:
                img = paused_img.copy()
                self._overlay_coords(img)
                cv2.imshow(self.window, img)

            key = cv2.waitKey(30) & 0xFF
            if key == 27:  # ESC
                print("[INFO] Aborted.")
                break
            elif key == 13:  # ENTER
                if len(self.pixel_pts) < 4:
                    print("[WARN] Need at least 4 samples; currently", len(self.pixel_pts))
                    continue
                self._solve()
                break
            elif key == 32:  # SPACE
                # freeze current frame until next SPACE
                ret, frame = cap.read()
                if ret:
                    paused_img = frame.copy()
                    print("[INFO] Image frozen — click the gripper tip.")
                else:
                    print("[WARN] Cannot freeze — no frame.")
            elif key == ord("r"):
                paused_img = None  # resume live
            elif key == ord("f") or key == ord("F"):
                # Toggle free/lock mode
                if not self.is_free:
                    print("[INFO] Releasing all servos — you can move the arm by hand.")
                    try:
                        res = self.mc.release_all_servos(0)
                        print(f"[INFO] release_all_servos result: {res}")
                        # print(self.mc.read_next_error())
                        # for i in range(1, 7):
                        #     res = self.mc.release_servo(i)
                        #     print(f"[INFO] release_servo {i} result: {res}")
                        time.sleep(1)
                        status = self.mc.is_all_servo_enable()
                        print(f"All servos enabled: {status}")

                    except Exception as exc:
                        print("[WARN] release_all_servos failed:", exc)
                    self.is_free = True
                else:
                    print("[INFO] Focusing all servos (locking joints).")
                    try:
                        self.mc.focus_all_servos()
                    except Exception as exc:
                        print("[WARN] focus_all_servos failed:", exc)
                    self.is_free = False

        cap.release()
        cv2.destroyAllWindows()

    def _solve(self):
        pix = np.array(self.pixel_pts, dtype=float)
        rob = np.array(self.robot_pts, dtype=float)
        H, mask = cv2.findHomography(pix, rob, cv2.RANSAC)
        if H is None:
            print("[ERR] Homography solve failed.")
            return
        print("\n[SUCCESS] Homography computed. Inliers:", int(mask.sum()), "/", len(mask))
        print(H)
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        with self.out_path.open("w") as f:
            json.dump({"H": H.tolist()}, f, indent=2)
        print(f"[INFO] Saved to {self.out_path}\n")


def main():
    parser = argparse.ArgumentParser(description="Manual homography calibration for myCobot-280")
    parser.add_argument("--port", default=os.getenv("MYCOBOT_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--baud", type=int, default=int(os.getenv("MYCOBOT_BAUD", 115200)))
    parser.add_argument("--output", default="env/homography.json", help="Destination JSON file")
    parser.add_argument("--cam-index", type=int, default=int(os.getenv("CAM_INDEX", 0)), help="OpenCV camera index (default 0)")
    args = parser.parse_args()

    print(f"[INFO] Connecting to myCobot-280 on {args.port} …")
    mc = MyCobot280(args.port, args.baud)

    try:
        mc.set_gripper_state(1, 100)  # Close gripper with 100% speed
        time.sleep(1)  # Wait for gripper to fully open
    except Exception as exc:
        print("[WARN] Failed to close gripper:", exc)

    collector = Collector(mc, Path(args.output))
    collector.run(args.cam_index)


if __name__ == "__main__":
    main() 