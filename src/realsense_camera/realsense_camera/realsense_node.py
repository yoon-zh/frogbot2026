"""
Native RealSense camera driver node using pyrealsense2.

Streams aligned color and depth frames and camera intrinsics directly
to ROS 2 topics without requiring the external realsense2_camera ROS wrapper.
"""

import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


class RealSenseDriverNode(Node):
    """
    Streams RealSense RGB, aligned depth, and camera intrinsics to ROS 2 topics.
    """

    def __init__(self):
        super().__init__("realsense_driver_node")

        # Parameters
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("color_fps", 30)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("depth_fps", 30)
        self.declare_parameter("camera_frame_id", "camera_color_optical_frame")
        self.declare_parameter("enable_depth", True)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        self.color_w = int(self.get_parameter("color_width").value)
        self.color_h = int(self.get_parameter("color_height").value)
        self.color_fps = int(self.get_parameter("color_fps").value)
        self.depth_w = int(self.get_parameter("depth_width").value)
        self.depth_h = int(self.get_parameter("depth_height").value)
        self.depth_fps = int(self.get_parameter("depth_fps").value)
        self.frame_id = str(self.get_parameter("camera_frame_id").value)
        self.enable_depth = bool(self.get_parameter("enable_depth").value)

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_color = self.create_publisher(Image, color_topic, sensor_qos)
        self.pub_depth = self.create_publisher(Image, depth_topic, sensor_qos) if self.enable_depth else None
        self.pub_info = self.create_publisher(CameraInfo, info_topic, sensor_qos)

        self.bridge = CvBridge() if CvBridge is not None else None

        self.pipeline = None
        self.align = None
        self.timer = None

        if rs is None:
            self.get_logger().error(
                "pyrealsense2 is not installed! Run 'pip install pyrealsense2' or use the "
                "official realsense2_camera ROS wrapper."
            )
            return

        self._start_camera()

    def _start_camera(self):
        """Initializes the RealSense hardware pipeline."""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()

            config.enable_stream(
                rs.stream.color,
                self.color_w,
                self.color_h,
                rs.format.bgr8,
                self.color_fps,
            )

            if self.enable_depth:
                config.enable_stream(
                    rs.stream.depth,
                    self.depth_w,
                    self.depth_h,
                    rs.format.z16,
                    self.depth_fps,
                )
                self.align = rs.align(rs.stream.color)

            profile = self.pipeline.start(config)
            self.get_logger().info(
                f"RealSense camera started successfully ({self.color_w}x{self.color_h} @ {self.color_fps}fps)"
            )

            # Timer for frame capture loop
            period = 1.0 / float(self.color_fps)
            self.timer = self.create_timer(period, self._poll_frames)

        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense camera: {e}")
            self.pipeline = None

    def _poll_frames(self):
        """Polls frames from the hardware pipeline and publishes them."""
        if self.pipeline is None:
            return

        try:
            frames = self.pipeline.poll_for_frames()
            if not frames:
                return

            if self.align is not None:
                frames = self.align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if self.enable_depth else None

            if not color_frame:
                return

            stamp = self.get_clock().now().to_msg()
            header = Header()
            header.stamp = stamp
            header.frame_id = self.frame_id

            # Color image
            color_data = np.asanyarray(color_frame.get_data())
            if self.bridge is not None:
                color_msg = self.bridge.cv2_to_imgmsg(color_data, encoding="bgr8")
                color_msg.header = header
                self.pub_color.publish(color_msg)

            # Depth image (16UC1 millimeters)
            if depth_frame and self.pub_depth is not None and self.bridge is not None:
                depth_data = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_data, encoding="16UC1")
                depth_msg.header = header
                self.pub_depth.publish(depth_msg)

            # Camera Info
            color_profile = color_frame.get_profile().as_video_stream_profile()
            intr = color_profile.get_intrinsics()

            info_msg = CameraInfo()
            info_msg.header = header
            info_msg.width = intr.width
            info_msg.height = intr.height
            info_msg.distortion_model = "plumb_bob"

            # Intrinsics matrix K
            info_msg.k = [
                intr.fx, 0.0, intr.ppx,
                0.0, intr.fy, intr.ppy,
                0.0, 0.0, 1.0,
            ]
            # Projection matrix P
            info_msg.p = [
                intr.fx, 0.0, intr.ppx, 0.0,
                0.0, intr.fy, intr.ppy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
            self.pub_info.publish(info_msg)

        except Exception as e:
            self.get_logger().warn(f"Error reading RealSense frame: {e}")

    def destroy_node(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

