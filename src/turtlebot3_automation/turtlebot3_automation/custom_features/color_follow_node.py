"""Custom feature: follow a colored object using computer vision."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from ..utils.logging import configure_logging

try:  # pragma: no cover - runtime dependency
    from cv_bridge import CvBridge
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("cv_bridge is required for color following") from exc


# HSV color ranges for common colors (widened for better detection)
COLOR_RANGES = {
    "red": [(0, 50, 50), (15, 255, 255), (165, 50, 50), (180, 255, 255)],  # Red wraps around - widened
    "blue": [(85, 50, 50), (135, 255, 255)],   # Blue range - widened
    "green": [(35, 50, 50), (85, 255, 255)],   # Green range - widened
    "yellow": [(15, 50, 50), (45, 255, 255)],  # Yellow range - widened
    "orange": [(5, 50, 50), (25, 255, 255)],   # Orange range - widened
}


class ColorFollowNode(Node):
    """Drive the robot toward a target colored object."""

    def __init__(self) -> None:
        super().__init__("color_follow_node")
        self.declare_parameter("target_color", "red")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_gain", 0.0025)
        self.declare_parameter("area_gain", 0.000002)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("max_linear", 0.18)
        self.declare_parameter("min_area", 100)

        self._logger = configure_logging(self.get_name())
        self._bridge = CvBridge()

        cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        cam_topic = self.get_parameter("camera_topic").get_parameter_value().string_value

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._image_sub = self.create_subscription(Image, cam_topic, self._on_image, 10)

        self._last_detection = False
        target = self.get_parameter("target_color").get_parameter_value().string_value
        self._logger.info("Color follow node initialized. Target color=%s", target)

    def _detect_color(self, frame: np.ndarray, color_name: str):
        """Detect colored blobs in the frame and return the largest contour center."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if color_name not in COLOR_RANGES:
            self._logger.warn("Unknown color %s, defaulting to red", color_name)
            color_name = "red"

        color_range = COLOR_RANGES[color_name]

        # Handle red color (wraps around HSV hue)
        if color_name == "red":
            mask1 = cv2.inRange(hsv, np.array(color_range[0]), np.array(color_range[1]))
            mask2 = cv2.inRange(hsv, np.array(color_range[2]), np.array(color_range[3]))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, np.array(color_range[0]), np.array(color_range[1]))

        # Clean up mask (reduced morphology for testing)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, 0

        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        min_area = self.get_parameter("min_area").get_parameter_value().integer_value
        if area < min_area:
            return None, 0

        # Get center of largest blob
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            self._logger.info("🎯 Detected color %s at (%d,%d) with area %d", color_name, cx, cy, int(area))
            return (cx, cy), area

        return None, 0

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        target_color = self.get_parameter("target_color").get_parameter_value().string_value
        center, area = self._detect_color(frame, target_color)
        twist = Twist()

        if center is not None:
            self._last_detection = True
            cx, cy = center
            width = frame.shape[1]
            height = frame.shape[0]
            error_x = (cx - width / 2.0)

            # Get parameters
            angular_gain = self.get_parameter("angular_gain").get_parameter_value().double_value
            linear_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
            area_gain = self.get_parameter("area_gain").get_parameter_value().double_value
            max_angular = self.get_parameter("max_angular").get_parameter_value().double_value
            max_linear = self.get_parameter("max_linear").get_parameter_value().double_value

            # Calculate control commands
            angular = -error_x * angular_gain
            # Slow down as the colored object gets larger (closer)
            linear = min(linear_speed, linear_speed - area * area_gain)

            twist.angular.z = max(-max_angular, min(max_angular, angular))
            twist.linear.x = max(0.0, min(max_linear, linear))

            self._logger.debug(
                "Color %s detected (area=%d) -> cmd linear=%.3f angular=%.3f",
                target_color,
                int(area),
                twist.linear.x,
                twist.angular.z,
            )
        else:
            self._last_detection = False

        self._cmd_pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = ColorFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        node.get_logger().info("Color follow node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
