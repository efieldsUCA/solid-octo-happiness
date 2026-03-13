#!/usr/bin/env python3
"""
HSV Color Calibration Tool — ROS2 / RealSense Version
======================================================
Subscribes to your live RealSense color topic so what you tune is
EXACTLY what sorting_master.py will see.

PRE-REQUISITES:
  Terminal 1: ros2 launch solid_octo octo_launch.py
  Terminal 2: ros2 run tf2_ros static_transform_publisher ...
  Terminal 3: ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

  Then in a new terminal:
    cd ~/seniordesign_ws/src/solid_octo/solid_octo
    source /opt/ros/jazzy/setup.bash
    source ~/seniordesign_ws/install/setup.bash
    python3 hsv_calibrator.py

WORKFLOW:
  1. Press 1=red  2=yellow  3=green  4=blue  to pick a color
  2. Hold that ball/bucket in front of the camera
  3. Drag sliders until ONLY that object is WHITE in the MASK window
  4. Green box in live feed = round enough.  Orange = not round enough yet.
  5. Press S to save that color
  6. Repeat for all 4 colors
  7. Press Q to export colors.json next to this script

TIPS:
  - Keep H tight, but make S_low and V_low WIDE (50-80 range)
  - Test by moving the ball closer/farther and into shadows
  - If a color bleeds into other objects, raise S_low
  - Retune at the competition venue under their lighting — takes 10 min
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import threading

# ─────────────────────────────────────────────────────────────
# TOPIC — matches your sorting_master.py
# ─────────────────────────────────────────────────────────────
COLOR_TOPIC = "/camera/camera/color/image_raw"

# ─────────────────────────────────────────────────────────────
# DEFAULT STARTING RANGES
# Intentionally wide — tighten H if you get false positives,
# but keep S/V loose to survive lighting changes
# ─────────────────────────────────────────────────────────────
DEFAULTS = {
    "red": {
        "low1":  [0,   80, 60],
        "high1": [10,  255, 255],
        "low2":  [170, 80, 60],
        "high2": [180, 255, 255],
        "dual":  True
    },
    "yellow": {
        "low1":  [18,  80, 60],
        "high1": [35,  255, 255],
        "dual":  False
    },
    "green": {
        "low1":  [35,  60, 60],
        "high1": [85,  255, 255],
        "dual":  False
    },
    "blue": {
        "low1":  [95,  80, 60],
        "high1": [135, 255, 255],
        "dual":  False
    },
}

COLOR_ORDER = ["red", "yellow", "green", "blue"]
WINDOW_MAIN = "Live Feed  |  1=red 2=yellow 3=green 4=blue  S=save  Q=quit"
WINDOW_MASK = "MASK  (tune until ONLY your target is WHITE)"
WINDOW_CTRL = "Sliders"


# ─────────────────────────────────────────────────────────────
# TRACKBAR HELPERS
# ─────────────────────────────────────────────────────────────
def nothing(_):
    pass


def create_trackbars(color_name):
    d = DEFAULTS[color_name]
    try:
        cv2.destroyWindow(WINDOW_CTRL)
    except Exception:
        pass
    cv2.namedWindow(WINDOW_CTRL, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_CTRL, 500, 400 if d["dual"] else 230)

    cv2.createTrackbar("H_low1",  WINDOW_CTRL, d["low1"][0],  179, nothing)
    cv2.createTrackbar("S_low1",  WINDOW_CTRL, d["low1"][1],  255, nothing)
    cv2.createTrackbar("V_low1",  WINDOW_CTRL, d["low1"][2],  255, nothing)
    cv2.createTrackbar("H_high1", WINDOW_CTRL, d["high1"][0], 179, nothing)
    cv2.createTrackbar("S_high1", WINDOW_CTRL, d["high1"][1], 255, nothing)
    cv2.createTrackbar("V_high1", WINDOW_CTRL, d["high1"][2], 255, nothing)

    if d["dual"]:
        cv2.createTrackbar("H_low2",  WINDOW_CTRL, d["low2"][0],  179, nothing)
        cv2.createTrackbar("S_low2",  WINDOW_CTRL, d["low2"][1],  255, nothing)
        cv2.createTrackbar("V_low2",  WINDOW_CTRL, d["low2"][2],  255, nothing)
        cv2.createTrackbar("H_high2", WINDOW_CTRL, d["high2"][0], 179, nothing)
        cv2.createTrackbar("S_high2", WINDOW_CTRL, d["high2"][1], 255, nothing)
        cv2.createTrackbar("V_high2", WINDOW_CTRL, d["high2"][2], 255, nothing)


def read_trackbars(dual=False):
    lo1 = [cv2.getTrackbarPos("H_low1",  WINDOW_CTRL),
           cv2.getTrackbarPos("S_low1",  WINDOW_CTRL),
           cv2.getTrackbarPos("V_low1",  WINDOW_CTRL)]
    hi1 = [cv2.getTrackbarPos("H_high1", WINDOW_CTRL),
           cv2.getTrackbarPos("S_high1", WINDOW_CTRL),
           cv2.getTrackbarPos("V_high1", WINDOW_CTRL)]
    if not dual:
        return lo1, hi1, None, None
    lo2 = [cv2.getTrackbarPos("H_low2",  WINDOW_CTRL),
           cv2.getTrackbarPos("S_low2",  WINDOW_CTRL),
           cv2.getTrackbarPos("V_low2",  WINDOW_CTRL)]
    hi2 = [cv2.getTrackbarPos("H_high2", WINDOW_CTRL),
           cv2.getTrackbarPos("S_high2", WINDOW_CTRL),
           cv2.getTrackbarPos("V_high2", WINDOW_CTRL)]
    return lo1, hi1, lo2, hi2


# ─────────────────────────────────────────────────────────────
# VISION HELPERS
# ─────────────────────────────────────────────────────────────
def apply_mask(hsv, lo1, hi1, lo2=None, hi2=None):
    mask = cv2.inRange(hsv, np.array(lo1), np.array(hi1))
    if lo2 is not None:
        mask = cv2.bitwise_or(mask,
               cv2.inRange(hsv, np.array(lo2), np.array(hi2)))
    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def draw_overlay(frame, mask, color_name, saved_colors):
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    display = frame.copy()

    for c in contours:
        area = cv2.contourArea(c)
        if area < 800:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        x, y, w, h  = cv2.boundingRect(c)
        col = (0, 255, 0) if circularity > 0.7 else (0, 165, 255)
        cv2.rectangle(display, (x, y), (x+w, y+h), col, 2)
        cv2.putText(display,
                    f"circ:{circularity:.2f}  area:{int(area)}",
                    (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)

    saved_str = "  ".join([f"[{c[0].upper()}]" for c in saved_colors])
    cv2.putText(display, f"Tuning: {color_name.upper()}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    cv2.putText(display, f"Saved: {saved_str or 'none yet'}",
                (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(display, "GREEN box = round enough   ORANGE = not round",
                (10, display.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    return display


# ─────────────────────────────────────────────────────────────
# EXPORT
# ─────────────────────────────────────────────────────────────
DUAL_RED_PATCH = """
# ── Paste this into the SortingMaster class in sorting_master.py ─────

def get_mask(self, hsv, color_name):
    \"\"\"Build HSV mask, handling dual-range red automatically.\"\"\"
    lower = np.array(self.color_ranges[color_name][0])
    upper = np.array(self.color_ranges[color_name][1])
    mask  = cv2.inRange(hsv, lower, upper)

    key2 = color_name + "_2"
    if key2 in self.color_ranges:
        lower2 = np.array(self.color_ranges[key2][0])
        upper2 = np.array(self.color_ranges[key2][1])
        mask   = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))

    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

# Then replace every:
#   mask = cv2.inRange(hsv, lower, upper)
# with:
#   mask = self.get_mask(hsv, color_name)
# ─────────────────────────────────────────────────────────────────────
"""


def export_json(saved):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "colors.json")
    out = {}
    for name, data in saved.items():
        out[name] = [data["lower"], data["upper"]]
        if data.get("dual"):
            out[name + "_2"] = [data["lower2"], data["upper2"]]

    with open(path, "w") as f:
        json.dump(out, f, indent=2)

    print(f"\n✓  Exported → {path}")
    print("   Colors saved:", list(saved.keys()))
    if "red" in saved:
        print(DUAL_RED_PATCH)


# ─────────────────────────────────────────────────────────────
# ROS2 NODE  (camera subscriber runs in background thread)
# ─────────────────────────────────────────────────────────────
class CalibratorNode(Node):
    def __init__(self):
        super().__init__("hsv_calibrator")
        self.bridge       = CvBridge()
        self.latest_frame = None
        self.lock         = threading.Lock()

        self.create_subscription(
            Image,
            COLOR_TOPIC,
            self._image_cb,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {COLOR_TOPIC}")

    def _image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.latest_frame = frame

    def get_frame(self):
        with self.lock:
            return None if self.latest_frame is None \
                       else self.latest_frame.copy()


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = CalibratorNode()

    # ROS spin in background so image callbacks keep firing
    spin_thread = threading.Thread(target=rclpy.spin,
                                   args=(node,), daemon=True)
    spin_thread.start()

    saved_colors = {}
    current_idx  = 0
    color_name   = COLOR_ORDER[current_idx]
    dual         = DEFAULTS[color_name]["dual"]

    cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_MAIN, 640, 480)
    cv2.resizeWindow(WINDOW_MASK, 640, 480)
    create_trackbars(color_name)

    print("\n─── HSV Calibrator (ROS2 mode) ───────────────────────")
    print(f"  Topic : {COLOR_TOPIC}")
    print("  Keys  : 1=red  2=yellow  3=green  4=blue")
    print("          S=save current color   Q=quit & export")
    print("──────────────────────────────────────────────────────")
    print("  Waiting for first camera frame", end="", flush=True)

    # Block until first frame arrives
    while node.get_frame() is None:
        cv2.waitKey(200)
        print(".", end="", flush=True)
    print(" ready!\n")

    while rclpy.ok():
        frame = node.get_frame()
        if frame is None:
            cv2.waitKey(30)
            continue

        frame = cv2.resize(frame, (640, 480))
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lo1, hi1, lo2, hi2 = read_trackbars(dual)
        mask    = apply_mask(hsv, lo1, hi1, lo2, hi2)
        display = draw_overlay(frame, mask, color_name, saved_colors)

        cv2.imshow(WINDOW_MAIN, display)
        cv2.imshow(WINDOW_MASK, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))

        key = cv2.waitKey(30) & 0xFF

        # Switch color
        if key in [ord('1'), ord('2'), ord('3'), ord('4')]:
            current_idx = key - ord('1')
            color_name  = COLOR_ORDER[current_idx]
            dual        = DEFAULTS[color_name]["dual"]
            create_trackbars(color_name)
            print(f"→ Switched to: {color_name}")

        # Save
        elif key in [ord('s'), ord('S')]:
            lo1, hi1, lo2, hi2 = read_trackbars(dual)
            entry = {"lower": lo1, "upper": hi1, "dual": dual}
            if dual:
                entry["lower2"] = lo2
                entry["upper2"] = hi2
            saved_colors[color_name] = entry
            print(f"✓ Saved {color_name}: lower={lo1} upper={hi1}" +
                  (f"\n         lower2={lo2} upper2={hi2}" if dual else ""))

        # Quit
        elif key in [ord('q'), ord('Q')]:
            break

    cv2.destroyAllWindows()

    if saved_colors:
        export_json(saved_colors)
    else:
        print("Nothing saved — no colors.json written.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
