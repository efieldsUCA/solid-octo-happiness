#!/usr/bin/env python3
"""
waypoint_navigator.py
Drives the full ball-sorting sequence without RViz.

Sequence per ball:
  1. Nav2 → BALL waypoint (approach to ~0.5m)
  2. Publish /vision_activate → sorting_master does fine approach + GRAB
  3. Wait for /grab_complete
  4. Nav2 → BUCKET waypoint
  5. Publish /release_activate → sorting_master sends RELEASE
  6. Nav2 → next ball (or return to start when done)

WAYPOINT FORMAT:  (x_meters, y_meters, yaw_radians)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ─────────────────────────────────────────────────────────────
# EDIT THESE after measuring your arena
# ─────────────────────────────────────────────────────────────
BALL_WAYPOINTS = [
    (1.730, -1.152, 0.325),   # ball 1 approach (0.5m before ball at 2.146, -1.430)
    # add more balls here
]

BUCKET_WAYPOINTS = [
    (3.387, -3.133, -2.138),  # bucket for ball 1
    # add matching bucket per ball above
]

HOME = (0.000, 0.000, 0.000)
# ─────────────────────────────────────────────────────────────


def make_pose(nav: BasicNavigator, x, y, yaw) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp    = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def drive_to(nav: BasicNavigator, x, y, yaw, label="") -> bool:
    nav.get_logger().info(f"Navigating → {label} ({x:.2f}, {y:.2f})")
    nav.goToPose(make_pose(nav, x, y, yaw))
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            nav.get_logger().info(
                f"  {label} — {fb.distance_remaining:.2f}m remaining",
                throttle_duration_sec=2.0
            )
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info(f"  Reached {label}")
        return True
    nav.get_logger().warn(f"  {label} failed: {result}")
    return False


class WaypointSequencer(Node):
    def __init__(self):
        super().__init__("waypoint_sequencer")
        self.vision_pub  = self.create_publisher(Bool, "/vision_activate",  10)
        self.release_pub = self.create_publisher(Bool, "/release_activate", 10)
        self.grab_done   = False
        self.create_subscription(Bool, "/grab_complete", self._grab_cb, 10)

    def _grab_cb(self, msg: Bool):
        if msg.data:
            self.grab_done = True

    def signal_vision(self):
        self.grab_done = False
        msg = Bool(); msg.data = True
        self.vision_pub.publish(msg)

    def wait_for_grab(self, timeout_sec=30.0):
        start = self.get_clock().now()
        while not self.grab_done:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn("Grab timeout — continuing anyway")
                return False
        return True

    def signal_release(self):
        msg = Bool(); msg.data = True
        self.release_pub.publish(msg)


def main():
    rclpy.init()
    nav       = BasicNavigator()
    sequencer = WaypointSequencer()

    nav.get_logger().info("Waiting for Nav2...")
    nav.waitUntilNav2Active()
    nav.get_logger().info("Nav2 active — starting sequence")

    pairs = list(zip(BALL_WAYPOINTS, BUCKET_WAYPOINTS))

    for i, ((bx, by, byaw), (dx, dy, dyaw)) in enumerate(pairs):
        nav.get_logger().info(f"=== Ball {i+1}/{len(pairs)} ===")

        # 1. Drive to ball approach waypoint
        if not drive_to(nav, bx, by, byaw, f"ball{i+1}"):
            nav.get_logger().warn(f"Skipping ball {i+1} — nav failed")
            continue

        # 2. Hand off to vision for fine approach + grab
        nav.get_logger().info("Handing off to vision...")
        sequencer.signal_vision()
        sequencer.wait_for_grab(timeout_sec=45.0)

        # 3. Drive to bucket
        drive_to(nav, dx, dy, dyaw, f"bucket{i+1}")

        # 4. Release
        sequencer.signal_release()
        import time; time.sleep(1.0)   # give arm time to release

    # Return home
    nav.get_logger().info("Returning home...")
    drive_to(nav, *HOME, "home")

    nav.get_logger().info("Sequence complete.")
    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
