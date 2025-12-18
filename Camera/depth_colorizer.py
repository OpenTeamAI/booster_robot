#!/usr/bin/env python3
import sys
from typing import Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


COLOR_MAPS = {
    "turbo": cv2.COLORMAP_TURBO,
    "jet": cv2.COLORMAP_JET,
    "viridis": cv2.COLORMAP_VIRIDIS,
    "magma": cv2.COLORMAP_MAGMA,
    "plasma": cv2.COLORMAP_PLASMA,
}


class DepthColorizer(Node):
    def __init__(self) -> None:
        super().__init__("depth_colorizer")
        self.declare_parameter("input_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("output_topic", "/camera/camera/depth/image_rect_colorized")
        self.declare_parameter("max_depth_mm", 4000.0)
        self.declare_parameter("invert", True)
        self.declare_parameter("color_map", "turbo")

        self._bridge = CvBridge()
        self._input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self._output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._max_depth_mm = self.get_parameter("max_depth_mm").get_parameter_value().double_value
        self._invert = self.get_parameter("invert").get_parameter_value().bool_value
        color_map_name = self.get_parameter("color_map").get_parameter_value().string_value
        self._color_map = COLOR_MAPS.get(color_map_name, cv2.COLORMAP_TURBO)

        self._sub = self.create_subscription(
            Image,
            self._input_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(Image, self._output_topic, 10)
        self.get_logger().info(
            f"Colorizing depth: {self._input_topic} -> {self._output_topic}"
        )

    def _depth_to_mm(self, msg: Image) -> Tuple[np.ndarray, float]:
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding in ("16UC1", "mono16"):
            depth_mm = depth.astype(np.float32)
            scale = 1.0
        elif msg.encoding == "32FC1":
            depth_mm = depth.astype(np.float32) * 1000.0
            scale = 1000.0
        else:
            depth_mm = depth.astype(np.float32)
            scale = 1.0
        return depth_mm, scale

    def _on_depth(self, msg: Image) -> None:
        depth_mm, _ = self._depth_to_mm(msg)
        max_depth = max(self._max_depth_mm, 1.0)
        depth_mm = np.nan_to_num(depth_mm, nan=0.0, posinf=max_depth, neginf=0.0)
        depth_mm = np.clip(depth_mm, 0.0, max_depth)
        scaled = (depth_mm / max_depth * 255.0).astype(np.uint8)
        if self._invert:
            scaled = 255 - scaled
        color = cv2.applyColorMap(scaled, self._color_map)
        out = self._bridge.cv2_to_imgmsg(color, encoding="bgr8")
        out.header = msg.header
        self._pub.publish(out)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = DepthColorizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
