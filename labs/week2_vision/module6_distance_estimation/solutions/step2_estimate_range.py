"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 6 — Step 2: Estimate Range and Approach  (SOLUTION)
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

# -- Course setup: makes the shared `neo_lab` helper importable.
#    You don't need to read or change this block. --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

# -- Constants --------------------------------------------------------------
FOCAL_PX = 320.0
REAL_GATE_WIDTH = 1.5
MIN_AREA = 400
COL_CENTER = 320
STOP_DIST = 2.5
APPROACH_PITCH = 0.15
MAX_YAW = 0.2
SEARCH_YAW = 0.2

# -- Module-level state -----------------------------------------------------
_done = False

def reset():
    global _done
    _done = False


def update(drone):
    global _done
    if _done:
        return True
    image = drone.camera.get_color_image()
    best = neo_lab.largest_cyan_gate(image, MIN_AREA)
    if best is None:
        drone.flight.send_pcmd(0, 0, SEARCH_YAW, 0)
        return False
    x, y, w, h = cv2.boundingRect(best)
    distance = FOCAL_PX * REAL_GATE_WIDTH / max(w, 1)
    gate_col = x + w / 2.0
    err = (gate_col - COL_CENTER) / COL_CENTER
    yaw = uav_utils.clamp(err * MAX_YAW, -MAX_YAW, MAX_YAW)
    if distance <= STOP_DIST:
        drone.flight.stop()
        print(f"[Step 2] Reached gate, distance ~ {distance:.2f} m")
        _done = True
        return True
    drone.flight.send_pcmd(APPROACH_PITCH, 0, yaw, 0)
    return False


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Estimate Range and Approach")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
