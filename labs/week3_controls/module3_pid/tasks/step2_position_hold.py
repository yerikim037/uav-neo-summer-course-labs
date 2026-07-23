"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Fly a Distance (PID on Position)
Integrate forward velocity into position and PID to a target distance,
while a proportional term keeps altitude.
"""

import drone_core
import drone_utils as uav_utils

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
TARGET_DIST = 4.0    # meters forward
TARGET_HEIGHT = 1.0  # hold launch height
KP = 0.15
KI = 0.0
KD = 0.5    # strong velocity damping to avoid overshoot
PITCH_LIMIT = 0.25
ALT_KP = 0.12
THROTTLE_LIMIT = 0.5
MIN_TRAVEL = 5.0   # fly at least this long before checking 'arrived'
SETTLE_SPEED = 0.25  # must slow below this to count as arrived
HOLD_TIME = 1.5

# -- Module-level state -----------------------------------------------------
_pos = 0.0
_err_int = 0.0
_prev_err = 0.0
_t = 0.0
_hold = 0.0
_done = False

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Return the PID controller output from the three gain terms (see README, Key terms)."""
    ##################################
    #### START PUT CODE HERE #########
    output = kp * err + ki * err_int + kd * err_dot
    ###### END PUT CODE HERE #########
    ##################################
    return output

def reset():
    global _pos, _err_int, _prev_err, _t, _hold, _done
    _pos = 0.0
    _err_int = 0.0
    _prev_err = 0.0
    _t = 0.0
    _hold = 0.0
    _done = False


def update(drone):
    global _pos, _err_int, _prev_err, _t, _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########
    dt = drone.get_delta_time()
    vel = drone.physics.get_linear_velocity()
    forward_vel = vel[0]  # x-axis is forward
    _pos += forward_vel * dt
    _t += dt
    error = TARGET_DIST - _pos
    _err_int += error * dt
    error_dot = (error - _prev_err) / dt if dt > 0 else 0.0
    _prev_err = error
    pitch = uav_utils.clamp(pid_control(error, _err_int, error_dot, KP, KI, KD), -PITCH_LIMIT, PITCH_LIMIT)
    current_height = neo_lab.height(drone)
    height_error = TARGET_HEIGHT - current_height
    throttle = uav_utils.clamp(ALT_KP * height_error, -THROTTLE_LIMIT, THROTTLE_LIMIT)
    drone.flight.send_pcmd(pitch, 0, 0, throttle)   
    # There is no direct (x, z) readout, so estimate forward distance by dead reckoning:
    # integrate the forward component of drone.physics.get_linear_velocity() over time.
    # PID that distance to TARGET_DIST for the pitch command (clamped to PITCH_LIMIT), and
    # use a proportional term (ALT_KP) on height to hold TARGET_HEIGHT. Count as arrived
    # only after MIN_TRAVEL, once speed drops below SETTLE_SPEED for HOLD_TIME. See the
    # README (Key terms) for dead reckoning and the PID law.
    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Fly a Distance (PID on Position)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
