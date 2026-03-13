#!/usr/bin/env python3
"""
detector_node.py
Subscribes to the RealSense D455 color + depth images.
Publishes detected balls/buckets as a JSON string on /detected_objects.
Each detection includes depth_m so sorting_master doesn't need /scan.
"""

import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
from collections import deque

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ── Config ────────────────────────────────────────────────────
COLORS_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "colors.json"
)
MIN_AREA  = 600         # minimum contour area in pixels (320x240 frame)
MAX_DEPTH = 2.0         # ignore detections farther than this (meters)
KERNEL   = np.ones((5, 5), np.uint8)


def load_colors(path: str) -> dict:
    with open(path) as f:
        raw = json.load(f)
    out = {}
    for name, v in raw.items():
        out[name] = {
            "lower":  np.array(v["lower"],  dtype=np.uint8),
            "upper":  np.array(v["upper"],  dtype=np.uint8),
            "bgr":    tuple(v["bgr"]),
            "lower2": np.array(v["lower2"], dtype=np.uint8) if "lower2" in v else None,
            "upper2": np.array(v["upper2"], dtype=np.uint8) if "upper2" in v else None,
        }
    return out


def build_mask(hsv: np.ndarray, color: dict) -> np.ndarray:
    mask = cv2.inRange(hsv, color["lower"], color["upper"])
    if color["lower2"] is not None:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, color["lower2"], color["upper2"]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def has_straight_lines(cnt: np.ndarray, gray: np.ndarray) -> bool:
    x, y, w, h = cv2.boundingRect(cnt)
    pad = 4
    roi = gray[max(y-pad,0):min(y+h+pad, gray.shape[0]),
               max(x-pad,0):min(x+w+pad, gray.shape[1])]
    if roi.size == 0:
        return False
    edges = cv2.Canny(roi, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30,
                             minLineLength=max(10, int(min(w,h)*0.25)),
                             maxLineGap=8)
    return lines is not None and len(lines) >= 2


def classify_contour(cnt, gray):
    area      = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    if area < MIN_AREA or perimeter == 0:
        return None, None, None
    circularity = (4 * np.pi * area) / (perimeter ** 2)
    bbox     = cv2.boundingRect(cnt)
    # Balls are highly circular (>0.65); buckets are boxy (<0.65)
    obj_type = "BALL" if circularity > 0.65 else "BUCKET"
    return obj_type, bbox, circularity


class DetectorNode(Node):

    def __init__(self):
        super().__init__("detector_node")

        self.bridge      = CvBridge()
        self.depth_frame = None   # latest aligned depth (full res)
        self.colors      = load_colors(COLORS_JSON)

        # Publishers
        self.pub_objects = self.create_publisher(String, "/detected_objects", 10)
        self.pub_debug   = self.create_publisher(Image,  "/detector/debug_image", 10)

        # RealSense subscriptions
        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self.color_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"detector_node ready — colors: {list(self.colors.keys())}"
        )

    # ── Depth callback — just store the latest frame ──────────
    def depth_cb(self, msg: Image):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            pass

    # ── Color callback — main detection ───────────────────────
    def color_cb(self, msg: Image):
        self.get_logger().info("color_cb fired", throttle_duration_sec=2.0)
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Downscale to 320x240 for speed
            frame       = cv2.resize(raw, (320, 240))
            depth_small = cv2.resize(self.depth_frame, (320, 240), interpolation=cv2.INTER_NEAREST) if self.depth_frame is not None else None

            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = []

            for color_name, color_data in self.colors.items():
                mask      = build_mask(hsv, color_data)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    obj_type, bbox, circ = classify_contour(cnt, gray)
                    if obj_type is None:
                        continue

                    x, y, w, h = bbox
                    M  = cv2.moments(cnt)
                    cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
                    cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h // 2

                    # Depth: sample 9x9 region around centre, median of valid pixels
                    if depth_small is not None:
                        r = 4
                        patch = depth_small[max(cy-r,0):cy+r+1, max(cx-r,0):cx+r+1]
                        valid = patch[patch > 0].astype(float)
                        depth_m = float(np.median(valid)) / 1000.0 if len(valid) > 0 else 0.0
                    else:
                        depth_m = 0.0

                    # Skip objects beyond max depth
                    if depth_m > MAX_DEPTH:
                        continue

                    detections.append({
                        "color":       color_name,
                        "type":        obj_type,
                        "cx":          cx,
                        "cy":          cy,
                        "area":        int(cv2.contourArea(cnt)),
                        "circularity": round(circ, 3),
                        "depth_m":     round(depth_m, 3),
                    })

                    cv2.rectangle(frame, (x, y), (x+w, y+h), color_data["bgr"], 2)
                    cv2.putText(frame, f"{color_name} {obj_type} {depth_m:.2f}m",
                                (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                                color_data["bgr"], 1)

            msg_out      = String()
            msg_out.data = json.dumps(detections)
            self.pub_objects.publish(msg_out)

            if self.pub_debug.get_subscription_count() > 0:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            if detections:
                summary = [(d["color"], d["type"], str(d["depth_m"]) + "m", "c=" + str(d["circularity"])) for d in detections]
                self.get_logger().info(str(summary), throttle_duration_sec=1.0)

        except Exception as e:
            import traceback
            self.get_logger().error(f"Detection error: {e}\n{traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
