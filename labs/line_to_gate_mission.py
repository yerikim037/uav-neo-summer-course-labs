"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

File Name: line_to_gate_mission.py

Title: Line Follow -> AR-Tag Gate Alignment -> Mark Waypoint -> Pass-Through

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Real-flight mission (not a lab). The drone climbs, follows a line on the
floor with the downward camera, then when an ArUco tag on an upcoming gate is
spotted with the forward camera it switches to aligning with that tag -- the
gate's height is NOT assumed to be known ahead of time, so alignment climbs or
descends using the tag's position in the image instead of a fixed altitude. Once
centered and close enough, it MARKS the gate's world position and publishes it as a
waypoint on a ROS2 topic for a teammate's path-planning code, then commits to flying
straight through, and finally resumes line-following (or lands, if last gate).

This script owns VISION + ALIGNMENT + the marked waypoint only. A separate teammate's
"path code" subscribes to the published waypoint and plans/flies the trajectory. The
handoff contract (topic name, message type, frame) is defined by the constants in the
"Gate waypoint handoff" section below and MUST match the teammate's subscriber exactly.

Run this the same way any other drone_core script is run:
    python3 line_to_gate_mission.py          # real drone
    python3 line_to_gate_mission.py -s        # simulator (for testing before real flight)

Expected Outcome: Drone takes off, climbs to LINE_FOLLOW_HEIGHT, tracks the floor
line with roll corrections, detects the gate, yaws/climbs to center it, approaches,
flies through blind for GATE_PASS_DURATION seconds, then either looks for the next
line segment or lands.

Testing the gate-alignment logic in the simulator:
    The bundled UAVNeo sim has no printed ArUco tags and no floor line -- its gates
    are glowing cyan/white square frames instead. Running with "-s" therefore swaps
    detection for sim-visible equivalents (see _is_sim() / _find_targets() / _line_mask()
    below): the downward glow stands in for the line (same trick module3 uses), and
    the cyan gate's bounding box stands in for the AR tag. The ALIGN_GATE and
    PASS_THROUGH state logic -- the yaw/throttle P-control, the depth-based range
    check, the centering tolerance, the blind pass-through timing -- is IDENTICAL in
    both modes, so a clean sim run through a gate is a real test of that control
    logic before you trust it on hardware. Only the low-level "where is the target
    in this frame" detection differs between sim and real. The waypoint marking runs
    in both modes too: in sim it PRINTS the computed point (no ROS2 available); on the
    real drone it PUBLISHES it on the ROS2 topic for the teammate's path code.

BEFORE FLYING FOR REAL, YOU MUST TUNE / AGREE WITH YOUR TEAMMATE:
    - LINE_HSV_LOWER / LINE_HSV_UPPER  to your actual line's color under your lighting.
    - MARKER_SIZE_M                    to the printed tag's real side length (meters).
    - FOCAL_PX                         calibrated for your camera (see module6 README).
    - GATE_PASS_DURATION / GATE_PASS_PITCH  to your gate's physical depth and drone speed.
    - GATE_WAYPOINT_TOPIC / _FRAME / the message type  MUST match the teammate's subscriber.
    - The heading/frame conventions in _gate_world_point() are documented assumptions --
      fly once, compare the printed/published point to reality, and correct the signs.
Fly the first attempts over a mat/soft surface with a safety pilot on the transmitter --
the safety pilot always has override authority via the RC transmitter regardless of what
this script commands.
"""

########################################################################################
# Imports
########################################################################################

import math

import cv2
import numpy as np

import drone_core
import drone_utils as uav_utils

# -- Course setup: makes the shared `neo_lab` helper importable (don't edit). --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

########################################################################################
# Constants
########################################################################################

IMG_W, IMG_H = 640, 480
COL_CENTER = IMG_W / 2.0
ROW_CENTER = IMG_H / 2.0

# -- Line following (downward camera) --------------------------------------------------
# Tune to your physical line's color. Defaults assume a dark (e.g. black tape) line.
LINE_HSV_LOWER = np.array([0, 0, 0], dtype=np.uint8)
LINE_HSV_UPPER = np.array([179, 255, 60], dtype=np.uint8)
LINE_MIN_PIXELS = 150         # fewer than this -> treat the line as lost
LINE_FORWARD_PITCH = 0.18     # constant forward speed while tracking
LINE_MAX_ROLL = 0.25          # strafe authority for centering over the line
LINE_LOST_TIMEOUT = 1.5       # seconds with no line before holding position

LINE_FOLLOW_HEIGHT = 1.2      # m -- cruise altitude while line-following

# -- Gate / AR-tag detection (forward camera) ------------------------------------------
# 5x5 markers are used in person; the sim uses 6x6 (cv2.aruco.DICT_6X6_250).
ARUCO_DICT = cv2.aruco.DICT_5X5_250
MARKER_SIZE_M = 0.15                  # real side length of the printed tag -- MEASURE THIS
FOCAL_PX = 320.0                      # pinhole fallback; calibrate per module6 README
GATE_TAG_VERTICAL_OFFSET_M = 0.0      # tag center vs. true gate-opening center, +up

# Sim-only stand-ins (see "Testing ... in the simulator" above). Not used on real hardware.
SIM_CYAN_MIN_AREA = 400        # neo_lab.largest_cyan_gate min contour area
SIM_REAL_GATE_WIDTH_M = 1.5    # the sim gate's true outer width (matches module6)

GATE_ACQUIRE_FRAMES = 3        # consecutive frames required before locking on (debounce)
GATE_MIN_LOCK_PX = 12          # ignore marker detections smaller than this (noise/far away)
GATE_LOST_TIMEOUT = 1.5        # seconds the locked tag can vanish before we give up and re-search
SEARCH_YAW = 0.15              # gentle scan in place while the locked tag is briefly out of frame

GATE_MAX_YAW = 0.3             # yaw authority to center the tag horizontally
GATE_MAX_THROTTLE = 0.35       # vertical authority to center the tag (gate height is unknown!)
GATE_CENTER_TOL_PX = 40        # pixel error under this counts as "centered"
GATE_APPROACH_PITCH = 0.2      # forward speed once centered
GATE_THROUGH_TRIGGER_M = 1.2   # commit to a blind pass-through once this close and centered

ALIGN_MIN_HEIGHT_M = 0.5       # never let alignment throttle drive the drone below/above these
ALIGN_MAX_HEIGHT_M = 3.5

GATE_PASS_PITCH = 0.25         # forward speed while flying blind through the gate opening
GATE_PASS_DURATION = 1.5       # seconds -- tune to the gate's physical depth and drone speed

# Set to True if there is another line segment after this gate; False lands after passing it.
MORE_GATES_AFTER_THIS_ONE = False

MAX_MISSION_TIME = 90.0        # seconds -- failsafe: force landing no matter the state

# -- Gate waypoint handoff -> teammate's path code (ROS2) ------------------------------
# When aligned & centered on a gate, we compute the gate's WORLD position and publish it
# ONCE (latched) so the teammate's path-planning node can pick it up. The point is in a
# local NED world frame whose origin is the GPS fix captured at launch:
#     x = north (m), y = east (m), z = down (m, so gate height above ground is NEGATIVE)
# THESE MUST BE AGREED WITH YOUR TEAMMATE -- the subscriber has to match exactly.
GATE_WAYPOINT_TOPIC = "/mission/gate_waypoint"  # ROS2 topic the path code subscribes to
GATE_WAYPOINT_FRAME = "map"                     # frame_id stamped on the message
#   Message type: geometry_msgs/PoseStamped (position = gate point, orientation = heading
#   through the gate). If your teammate wants a bare point, switch to PointStamped.
METERS_PER_DEG_LAT = 111320.0   # local tangent-plane conversion (matches module2 gates)

########################################################################################
# Global variables
########################################################################################

drone = drone_core.create_drone()
_launcher = neo_lab.Launcher(LINE_FOLLOW_HEIGHT)

_state = "LAUNCH"
_mission_timer = 0.0

_line_lost_timer = 0.0

_locked_id = None
_acquire_count = 0
_gate_lost_timer = 0.0

_pass_timer = 0.0

_gps_origin = None      # (lat0, lon0) captured at launch; origin of the local NED frame
_wp_node = None         # lazily-created ROS2 node for the waypoint publisher (real only)
_wp_pub = None          # the waypoint publisher


########################################################################################
# Helpers
########################################################################################

_sim_cache = None


def _is_sim():
    """True once when running under 'python3 ... -s' (DroneSim), cached after first call."""
    global _sim_cache
    if _sim_cache is None:
        _sim_cache = "Sim" in type(drone.flight).__name__
    return _sim_cache


def _marker_center_and_size(marker):
    """(row_center, col_center, pixel_side) for an ARMarker -- corners are (row, col)."""
    corners = marker.get_corners().astype(np.float32)
    center = corners.mean(axis=0)
    sides = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]
    return float(center[0]), float(center[1]), float(np.mean(sides))


def _find_targets(image):
    """List of (id, row, col, pixel_size) candidate gate targets in a forward-camera frame.

    Real: one entry per detected ArUco tag, id = the tag's integer ID.
    Sim: at most one entry, the largest cyan gate contour, id = the fixed string
    "sim_gate" (the sim has no per-gate tag IDs to lock onto, only one gate at a time).
    """
    if _is_sim():
        contour = neo_lab.largest_cyan_gate(image, SIM_CYAN_MIN_AREA)
        if contour is None:
            return []
        x, y, w, h = cv2.boundingRect(contour)
        return [("sim_gate", y + h / 2.0, x + w / 2.0, (w + h) / 2.0)]
    return [(m.get_id(), *_marker_center_and_size(m))
            for m in uav_utils.get_ar_markers(image, marker_type=ARUCO_DICT)]


def _line_mask(down_image):
    """Binary line mask on the downward camera.

    Real: HSV-threshold the physical line (LINE_HSV_LOWER/UPPER).
    Sim: there is no floor line, so fall back to the gate's downward glow
    (same trick as module3's step1_detect_line.py) -- flying over it like a lane
    line at least gets the sim run moving forward toward the gate.
    """
    if _is_sim():
        return neo_lab.bright_mask(down_image) > 0
    hsv = cv2.cvtColor(down_image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, LINE_HSV_LOWER, LINE_HSV_UPPER) > 0


def _depth_at_m(depth_image, row, col, patch=10):
    """Median depth (meters) in a small window, ignoring invalid (0) readings."""
    if depth_image is None:
        return None
    r0, r1 = max(0, int(row) - patch), min(depth_image.shape[0], int(row) + patch)
    c0, c1 = max(0, int(col) - patch), min(depth_image.shape[1], int(col) + patch)
    window = depth_image[r0:r1, c0:c1]
    valid = window[window > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 100.0  # cm -> m


def _estimate_range_m(depth_image, row, col, pixel_side, max_range_m):
    """Prefer the depth camera; fall back to pinhole projection from the target's known size."""
    d = _depth_at_m(depth_image, row, col)
    if d is not None and 0.05 < d < max_range_m:
        return d
    if pixel_side <= 1:
        return None
    real_size_m = SIM_REAL_GATE_WIDTH_M if _is_sim() else MARKER_SIZE_M
    return FOCAL_PX * real_size_m / pixel_side


########################################################################################
# Waypoint marking + ROS2 handoff to the teammate's path code
########################################################################################

def _capture_gps_origin():
    """Record the launch GPS fix as the origin of the local NED world frame.

    Sim has a different GPS mapping and uses world_position() directly, so the origin
    is unused there.
    """
    global _gps_origin
    if _is_sim():
        _gps_origin = (0.0, 0.0)
        return
    lat, lon, _ = (float(v) for v in drone.physics.get_gps())
    _gps_origin = (lat, lon)
    print(f"[Mark] Local NED origin captured at lat={lat:.7f} lon={lon:.7f}")


def _drone_world_ned():
    """Drone position (north_m, east_m, down_m) in the local NED frame set at launch."""
    if _is_sim():
        x_east, y_up, z_north = neo_lab.world_position(drone)   # sim true position
        return z_north, x_east, -y_up
    lat, lon, _ = (float(v) for v in drone.physics.get_gps())
    lat0, lon0 = _gps_origin
    north = (lat - lat0) * METERS_PER_DEG_LAT
    east = (lon - lon0) * METERS_PER_DEG_LAT * math.cos(math.radians(lat0))
    down = -neo_lab.height(drone)
    return north, east, down


def _gate_world_point(dist_m):
    """(north, east, down) of the gate center + heading (deg) through it, at alignment.

    The gate is centered in the image and `dist_m` straight ahead, so it sits along the
    drone's heading. ASSUMPTION (verify on hardware): yaw is clockwise from north, per
    physics.py's get_attitude docstring -> north uses cos(yaw), east uses sin(yaw).
    """
    n0, e0, _ = _drone_world_ned()
    _, _, yaw = (float(a) for a in drone.physics.get_attitude())
    yaw_rad = math.radians(yaw)
    north = n0 + dist_m * math.cos(yaw_rad)
    east = e0 + dist_m * math.sin(yaw_rad)
    # Row-centered on the gate -> gate center is at the drone's current height (+ any
    # tag-to-opening offset). down is negative for a point above the ground.
    down = -(neo_lab.height(drone) + GATE_TAG_VERTICAL_OFFSET_M)
    return north, east, down, yaw


def _publish_gate_waypoint(north, east, down, yaw_deg):
    """Publish the gate point on the ROS2 topic (real drone only; latched for late subs)."""
    global _wp_node, _wp_pub
    # Imported lazily so sim runs (no ROS2 installed) never touch these modules.
    import rclpy as ros2
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
    from geometry_msgs.msg import PoseStamped

    if _wp_pub is None:
        _wp_node = ros2.create_node("gate_waypoint_pub")
        qos = QoSProfile(depth=1)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # latch: late subscribers still get it
        _wp_pub = _wp_node.create_publisher(PoseStamped, GATE_WAYPOINT_TOPIC, qos)

    half = math.radians(yaw_deg) / 2.0  # yaw-only quaternion about the down axis
    msg = PoseStamped()
    msg.header.stamp = _wp_node.get_clock().now().to_msg()
    msg.header.frame_id = GATE_WAYPOINT_FRAME
    msg.pose.position.x = float(north)
    msg.pose.position.y = float(east)
    msg.pose.position.z = float(down)
    msg.pose.orientation.w = math.cos(half)
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = math.sin(half)
    _wp_pub.publish(msg)


def _mark_gate(dist_m):
    """Compute the gate waypoint and hand it to the path code (publish real / print sim)."""
    north, east, down, yaw = _gate_world_point(dist_m)
    if _is_sim():
        print(f"[Mark] Gate waypoint (sim, NED) north={north:.2f} east={east:.2f} "
              f"down={down:.2f} heading={yaw:.1f}deg  -> would publish to {GATE_WAYPOINT_TOPIC}")
        return
    _publish_gate_waypoint(north, east, down, yaw)
    print(f"[Mark] Published gate waypoint to {GATE_WAYPOINT_TOPIC}: north={north:.2f} "
          f"east={east:.2f} down={down:.2f} heading={yaw:.1f}deg")


########################################################################################
# State handlers -- each returns the name of the next state
########################################################################################

def _do_line_follow(dt):
    global _line_lost_timer, _acquire_count, _locked_id

    # The line may end before the gate, so always check the forward camera first.
    fwd_image = drone.camera.get_color_image()
    targets = _find_targets(fwd_image)
    biggest = max((t[3] for t in targets), default=0.0)
    if biggest >= GATE_MIN_LOCK_PX:
        _acquire_count += 1
    else:
        _acquire_count = 0
    if _acquire_count >= GATE_ACQUIRE_FRAMES:
        _acquire_count = 0
        _locked_id = None  # let ALIGN_GATE lock onto the best target it sees
        print("[Line] Gate acquired -- switching to alignment")
        return "ALIGN_GATE"

    down_image = drone.camera.get_downward_image()
    mask = _line_mask(down_image)
    ys, xs = np.nonzero(mask)

    if xs.size < LINE_MIN_PIXELS:
        _line_lost_timer += dt
        drone.flight.stop()  # hold position rather than guess where the line went
        return "LINE_FOLLOW"

    _line_lost_timer = 0.0
    line_col = float(np.mean(xs))
    offset = (line_col - COL_CENTER) / COL_CENTER  # -1 (left) .. +1 (right)
    roll = uav_utils.clamp(offset * LINE_MAX_ROLL, -LINE_MAX_ROLL, LINE_MAX_ROLL)
    drone.flight.send_pcmd(LINE_FORWARD_PITCH, roll, 0, 0)
    return "LINE_FOLLOW"


def _do_align_gate(dt):
    global _locked_id, _gate_lost_timer

    image = drone.camera.get_color_image()
    depth = drone.camera.get_depth_image()
    targets = _find_targets(image)

    target = None
    if _locked_id is not None:
        for t in targets:
            if t[0] == _locked_id:
                target = t
                break
    if target is None and targets:
        target = max(targets, key=lambda t: t[3])
        _locked_id = target[0]

    if target is None:
        _gate_lost_timer += dt
        if _gate_lost_timer > GATE_LOST_TIMEOUT:
            print("[Gate] Lost the target -- resuming line search")
            _locked_id = None
            _gate_lost_timer = 0.0
            return "LINE_FOLLOW"
        drone.flight.send_pcmd(0, 0, SEARCH_YAW, 0)  # gentle scan while briefly out of frame
        return "ALIGN_GATE"
    _gate_lost_timer = 0.0

    _, row, col, pixel_side = target
    max_range_m = drone.camera.get_max_range() / 100.0
    dist_m = _estimate_range_m(depth, row, col, pixel_side, max_range_m)

    col_err = col - COL_CENTER
    row_err = row - ROW_CENTER
    if dist_m and GATE_TAG_VERTICAL_OFFSET_M:
        row_err -= FOCAL_PX * GATE_TAG_VERTICAL_OFFSET_M / dist_m

    yaw = uav_utils.clamp(col_err / COL_CENTER * GATE_MAX_YAW, -GATE_MAX_YAW, GATE_MAX_YAW)
    throttle = uav_utils.clamp(-row_err / ROW_CENTER * GATE_MAX_THROTTLE,
                               -GATE_MAX_THROTTLE, GATE_MAX_THROTTLE)

    # Never let vertical alignment drive the drone outside a safe altitude band.
    current_height = neo_lab.height(drone)
    if current_height >= ALIGN_MAX_HEIGHT_M:
        throttle = min(throttle, 0.0)
    elif current_height <= ALIGN_MIN_HEIGHT_M:
        throttle = max(throttle, 0.0)

    centered = abs(col_err) < GATE_CENTER_TOL_PX and abs(row_err) < GATE_CENTER_TOL_PX
    pitch = GATE_APPROACH_PITCH if centered else 0.0
    drone.flight.send_pcmd(pitch, 0, yaw, throttle)

    if centered and dist_m is not None and dist_m <= GATE_THROUGH_TRIGGER_M:
        print(f"[Gate] Aligned (id={_locked_id}) at {dist_m:.2f} m -- marking waypoint")
        _mark_gate(dist_m)            # hand the gate point to the teammate's path code
        return "PASS_THROUGH"
    return "ALIGN_GATE"


def _do_pass_through(dt):
    global _pass_timer
    _pass_timer += dt
    drone.flight.send_pcmd(GATE_PASS_PITCH, 0, 0, 0)  # fly straight; the tag is out of frame now
    if _pass_timer >= GATE_PASS_DURATION:
        drone.flight.stop()
        _pass_timer = 0.0
        print("[Gate] Pass-through complete")
        return "LINE_FOLLOW" if MORE_GATES_AFTER_THIS_ONE else "DONE"
    return "PASS_THROUGH"


########################################################################################
# Functions
########################################################################################

def start():
    global _state, _mission_timer, _line_lost_timer
    global _locked_id, _acquire_count, _gate_lost_timer, _pass_timer, _gps_origin

    _launcher.reset()
    _state = "LAUNCH"
    _mission_timer = 0.0
    _line_lost_timer = 0.0
    _locked_id = None
    _acquire_count = 0
    _gate_lost_timer = 0.0
    _pass_timer = 0.0
    _gps_origin = None

    print("\n" + "=" * 60)
    print("  Mission: line-follow -> gate align -> mark waypoint -> pass-through")
    print("=" * 60 + "\n")


def update():
    global _state, _mission_timer

    dt = drone.get_delta_time()
    _mission_timer += dt
    if _mission_timer > MAX_MISSION_TIME and _state != "DONE":
        print("[Failsafe] Max mission time exceeded -- landing")
        _state = "DONE"

    if _state == "LAUNCH":
        if _launcher.update(drone):
            _capture_gps_origin()   # fix the local NED origin now that we are airborne
            _state = "LINE_FOLLOW"
        return

    if _state == "LINE_FOLLOW":
        _state = _do_line_follow(dt)
    elif _state == "ALIGN_GATE":
        _state = _do_align_gate(dt)
    elif _state == "PASS_THROUGH":
        _state = _do_pass_through(dt)
    elif _state == "DONE":
        drone.flight.land()


def update_slow():
    if _state == "LAUNCH":
        print(f"[Launch] height={neo_lab.height(drone):.2f}m")
    else:
        print(f"[{_state}] height={neo_lab.height(drone):.2f}m t={_mission_timer:.1f}s")


########################################################################################
# DO NOT MODIFY: Register callbacks and begin execution
########################################################################################

if __name__ == "__main__":
    drone.set_start_update(start, update, update_slow)
    drone.go()
