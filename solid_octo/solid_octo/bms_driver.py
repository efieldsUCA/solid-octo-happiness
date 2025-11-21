import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy


class BallManipulationSystem(Node):
    """
    A ROS interface for the ball manipulation system
    """

    def __init__(self):
        super().__init__("ball_manipulation_system_node")
        # Create joy subscriber
        self.joy_subr = self.create_subscription(
            topic="joy",
            msg_type=Joy,
            callback=self.get_yz,
            qos_profile=1,
        )
        # Variables
        # Logging
        self.get_logger().info("BTS is activated.")

    def get_yz(self, msg):
        print(f"close: {msg.axes[-2]}")
        print(f"lift: {msg.axes[-1]}")
        self.get_logger().debug(f"/joy message: {msg}")


def main(args=None):
    rclpy.init(args=args)
    bms_node = BallManipulationSystem()
    rclpy.spin(bms_node)
    bms_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
