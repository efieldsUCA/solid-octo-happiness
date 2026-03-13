#!/usr/bin/env python3
"""
sorting_master.py
Handles vision-based fine approach and grab/release.
Subscribes to /detected_objects from detector_node (includes depth_m).
Publishes /grab_complete (std_msgs/Bool) so waypoint_navigator proceeds to bucket.

Flow:
  1. Idle until /vision_activate (Bool True) received from waypoint_navigator
  2. Steer toward highest-priority ball using proportional control
  3. When depth_m <= GRAB_DISTANCE_M: stop, send GRAB, publish /grab_complete
  4. waypoint_navigator drives to bucket, then publishes /release_activate
  5. sorting_master sends RELEASE
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg    import String, Bool
from geometry_msgs.msg import Twist

# ── Tunable constants ──────────────────────────────────────────
IMAGE_WIDTH    = 320
IMAGE_CENTRE_X = IMAGE_WIDTH // 2

TARGET_PRIORITY = ["RED", "YELLOW", "GREEN", "BLUE"]
TARGET_TYPE     = "BALL"

GRAB_DISTANCE_M  = 0.25   # stop and grab when depth reads this close
LINEAR_SPEED     = 0.12   # m/s forward creep
KP_ANGULAR       = 0.006  # pixels error → rad/s (tuned for 320px wide frame)
CENTRE_DEADBAND  = 15     # pixels


class SortingMaster(Node):

    def __init__(self):
        super().__init__("sorting_master")

        # Publishers
        self.cmd_vel_pub    = self.create_publisher(Twist,  "/cmd_vel",        10)
        self.arm_pub        = self.create_publisher(String, "/arm_command",     10)
        self.grab_done_pub  = self.create_publisher(Bool,   "/grab_complete",   10)

        # Subscribers
        self.create_subscription(String, "/detected_objects", self.objects_cb,   10)
        self.create_subscription(Bool,   "/vision_activate",  self.activate_cb,  10)
        self.create_subscription(Bool,   "/release_activate", self.release_cb,   10)

        # State
        self.latest_objects   = []
        self.vision_active    = False
        self.grabbed          = False
        self.last_valid_depth = 999.0   # tracks last non-zero depth reading

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("sorting_master ready — waiting for /vision_activate")

    # ── Callbacks ─────────────────────────────────────────────
    def objects_cb(self, msg: String):
        try:
            self.latest_objects = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_objects = []

    def activate_cb(self, msg: Bool):
        if msg.data:
            self.vision_active = True
            self.grabbed       = False
            self.get_logger().info("Vision handoff ACTIVATED — approaching ball")

    def release_cb(self, msg: Bool):
        if msg.data:
            self._send_arm("RELEASE")
            self.get_logger().info("RELEASE sent")

    # ── Main control loop ──────────────────────────────────────
    def control_loop(self):
        if not self.vision_active or self.grabbed:
            return

        target = self._pick_target()

        if target is None:
            self._send_velocity(0.0, 0.0)
            return

        depth_m = target.get("depth_m", 0.0)
        cx      = target["cx"]
        error   = cx - IMAGE_CENTRE_X

        # Track last valid depth reading
        if depth_m > 0.0:
            self.last_valid_depth = depth_m

        # Grab when close enough, OR when depth goes to 0 (ball too close for
        # D455 minimum range ~0.3m) and we were recently within 0.5m
        close_enough = (depth_m > 0.0 and depth_m <= GRAB_DISTANCE_M) or \
                       (depth_m == 0.0 and self.last_valid_depth <= 0.5)
        if close_enough:
            self._send_velocity(0.0, 0.0)
            self._send_arm("GRAB")
            self.grabbed       = True
            self.vision_active = False
            done_msg      = Bool()
            done_msg.data = True
            self.grab_done_pub.publish(done_msg)
            self.get_logger().info(
                f"GRAB triggered — {target['color']} at {depth_m:.2f}m"
            )
            return

        # Proportional steering toward ball
        angular = 0.0 if abs(error) < CENTRE_DEADBAND else -KP_ANGULAR * error
        self._send_velocity(LINEAR_SPEED, angular)

        self.get_logger().debug(
            f"Tracking {target['color']} | cx={cx} err={error:+d} "
            f"ω={angular:+.3f} depth={depth_m:.2f}m"
        )

    # ── Helpers ────────────────────────────────────────────────
    def _pick_target(self):
        balls = [o for o in self.latest_objects if o["type"] == TARGET_TYPE]
        if not balls:
            return None
        def key(o):
            try:    pri = TARGET_PRIORITY.index(o["color"])
            except: pri = 99
            return (pri, -o["area"])
        return sorted(balls, key=key)[0]

    def _send_velocity(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_vel_pub.publish(t)

    def _send_arm(self, command: str):
        msg      = String()
        msg.data = command
        self.arm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SortingMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
