"""
ROS 2 Node for RealSense Glass Obstacle Detection.

Subscribes to RealSense RGB, aligned depth, and camera intrinsics,
runs YOLO-World zero-shot detection, calculates 3D physical coordinates,
and publishes synthetic obstacle point clouds to Nav2 costmaps.
"""

import json
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import message_filters

from realsense_camera.glass_detector import (
    GlassDetector,
    GlassObstacle3D,
    create_pointcloud2_msg,
)


class GlassDetectionNode(Node):
    """
    ROS 2 Node that detects glass doors/walls and publishes a synthetic PointCloud2
    wall obstacle layer to Nav2.
    """

    def __init__(self):
        super().__init__("glass_detection_node")

        # Declare ROS 2 parameters
        self.declare_parameter("model_path", "yolov8s-world.pt")
        self.declare_parameter(
            "classes",
            ["glass door", "glass wall", "glass window", "transparent door", "door frame"],
        )
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("camera_height", 0.30)
        self.declare_parameter("camera_pitch_deg", 0.0)
        self.declare_parameter("depth_method", "hybrid")  # hybrid | ground_plane | frame_depth
        self.declare_parameter("wall_height", 1.5)
        self.declare_parameter("point_spacing", 0.05)
        self.declare_parameter("vertical_spacing", 0.05)
        self.declare_parameter("frame_strip_px", 10)
        self.declare_parameter("min_depth", 0.2)
        self.declare_parameter("max_depth", 8.0)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("device", "")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("obstacle_cloud_topic", "/camera/glass_obstacles")
        self.declare_parameter("annotated_image_topic", "/camera/glass_detection/image_annotated")
        self.declare_parameter("markers_topic", "/camera/glass_detection/markers")

        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.08)

        # Retrieve parameters
        model_path = self.get_parameter("model_path").value
        classes = self.get_parameter("classes").value
        conf_threshold = float(self.get_parameter("conf_threshold").value)
        camera_height = float(self.get_parameter("camera_height").value)
        camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
        depth_method = str(self.get_parameter("depth_method").value)
        wall_height = float(self.get_parameter("wall_height").value)
        point_spacing = float(self.get_parameter("point_spacing").value)
        vertical_spacing = float(self.get_parameter("vertical_spacing").value)
        frame_strip_px = int(self.get_parameter("frame_strip_px").value)
        min_depth = float(self.get_parameter("min_depth").value)
        max_depth = float(self.get_parameter("max_depth").value)
        device_str = str(self.get_parameter("device").value).strip()
        device = device_str if device_str else None

        self.target_frame = str(self.get_parameter("target_frame").value)
        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        obstacle_cloud_topic = str(self.get_parameter("obstacle_cloud_topic").value)
        annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        markers_topic = str(self.get_parameter("markers_topic").value)

        self.publish_annotated = bool(self.get_parameter("publish_annotated_image").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)

        # Setup CvBridge
        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().warn("cv_bridge not installed. Image conversions will use fallback numpy.")

        # Initialize detector
        self.get_logger().info(f"Initializing GlassDetector with model '{model_path}' and classes: {classes}")
        self.detector = GlassDetector(
            model_path=model_path,
            classes=classes,
            conf_threshold=conf_threshold,
            camera_height=camera_height,
            camera_pitch_deg=camera_pitch_deg,
            depth_method=depth_method,
            wall_height=wall_height,
            point_spacing=point_spacing,
            vertical_spacing=vertical_spacing,
            frame_strip_px=frame_strip_px,
            min_depth=min_depth,
            max_depth=max_depth,
            device=device,
        )

        if not self.detector.is_model_loaded():
            self.get_logger().error(
                "YOLO model failed to load. Please ensure 'ultralytics' is installed "
                "(pip install ultralytics) and network or weights file is accessible."
            )

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera Intrinsics Cache (fx, fy, cx, cy)
        self.cached_intrinsics: Optional[Tuple[float, float, float, float]] = None
        self.cached_optical_frame: Optional[str] = None

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publishers
        self.pub_cloud = self.create_publisher(PointCloud2, obstacle_cloud_topic, 10)
        self.pub_annotated = self.create_publisher(Image, annotated_image_topic, 5) if self.publish_annotated else None
        self.pub_markers = self.create_publisher(MarkerArray, markers_topic, 5) if self.publish_markers else None
        self.pub_status = self.create_publisher(String, "/camera/glass_detection/status", 5)

        # Camera Info Subscription
        self.sub_info = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_cb,
            sensor_qos,
        )

        # Synchronized RGB and Depth Subscriptions
        sync_queue = int(self.get_parameter("sync_queue_size").value)
        sync_slop = float(self.get_parameter("sync_slop_sec").value)

        self.sub_color = message_filters.Subscriber(self, Image, color_topic, qos_profile=sensor_qos)
        self.sub_depth = message_filters.Subscriber(self, Image, depth_topic, qos_profile=sensor_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth],
            queue_size=sync_queue,
            slop=sync_slop,
        )
        self.sync.registerCallback(self._sync_frames_cb)

        # Heartbeat timer for diagnostics
        self.create_timer(2.0, self._publish_status_heartbeat)

        self.last_detection_count = 0
        self.total_frames_processed = 0

        self.get_logger().info(
            f"GlassDetectionNode ready.\n"
            f"  Subscribed: {color_topic}, {depth_topic}, {camera_info_topic}\n"
            f"  Publishing: {obstacle_cloud_topic} (target frame: {self.target_frame})"
        )

    def _camera_info_cb(self, msg: CameraInfo):
        """Caches intrinsic parameters from camera info."""
        if self.cached_intrinsics is None:
            k = msg.k
            fx = float(k[0])
            fy = float(k[4])
            cx = float(k[2])
            cy = float(k[5])
            if fx > 0 and fy > 0:
                self.cached_intrinsics = (fx, fy, cx, cy)
                self.cached_optical_frame = msg.header.frame_id
                self.get_logger().info(
                    f"Received camera intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f} "
                    f"in frame '{msg.header.frame_id}'"
                )

    def _sync_frames_cb(self, color_msg: Image, depth_msg: Image):
        """Processes time-synchronized RGB and Depth images."""
        self.total_frames_processed += 1

        # Determine camera intrinsics
        if self.cached_intrinsics is None:
            # Fallback default 640x480 intrinsics if CameraInfo hasn't arrived
            fx = float(color_msg.width) * 0.6
            fy = fx
            cx = float(color_msg.width) / 2.0
            cy = float(color_msg.height) / 2.0
            intrinsics = (fx, fy, cx, cy)
        else:
            intrinsics = self.cached_intrinsics

        optical_frame = color_msg.header.frame_id or self.cached_optical_frame or "camera_color_optical_frame"

        # Convert ROS Image to OpenCV NumPy array
        rgb_image = self._convert_ros_to_cv(color_msg, is_depth=False)
        depth_image, depth_scale = self._convert_depth_to_cv(depth_msg)

        if rgb_image is None:
            return

        # Lookup transform from camera optical frame to target frame
        tf_mat = self._lookup_tf_matrix(self.target_frame, optical_frame, color_msg.header.stamp)

        # Run 2D detection
        detections = self.detector.detect_2d(rgb_image)
        self.last_detection_count = len(detections)

        # Process detections into 3D glass obstacles and point clouds
        obstacles = self.detector.process_detections(
            detections=detections,
            intrinsics=intrinsics,
            depth_img=depth_image,
            depth_scale_to_meters=depth_scale,
            tf_transform_matrix=tf_mat,
        )

        # Assemble and publish combined point cloud
        self._publish_cloud(obstacles, color_msg.header.stamp)

        # Publish visual markers for Foxglove / RViz
        if self.publish_markers and self.pub_markers is not None:
            self._publish_markers(obstacles, color_msg.header.stamp)

        # Publish annotated 2D image for Foxglove / web UI
        if self.publish_annotated and self.pub_annotated is not None:
            annotated_bgr = self.detector.annotate_image(rgb_image, obstacles)
            self._publish_annotated_image(annotated_bgr, color_msg.header)

    def _convert_ros_to_cv(self, msg: Image, is_depth: bool = False) -> Optional[np.ndarray]:
        """Converts sensor_msgs/Image to cv2 numpy array."""
        if self.bridge is not None:
            try:
                encoding = "bgr8" if not is_depth else "passthrough"
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            except Exception as e:
                self.get_logger().error(f"CvBridge conversion error: {e}")
                return None

        # Fallback pure numpy parsing
        try:
            if msg.encoding in ("rgb8", "bgr8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                return arr if msg.encoding == "bgr8" else cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "mono8":
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            else:
                return None
        except Exception:
            return None

    def _convert_depth_to_cv(self, msg: Image) -> Tuple[Optional[np.ndarray], float]:
        """
        Converts sensor_msgs/Image depth image to numpy array in meters.
        Returns (depth_array, scale_to_meters).
        """
        raw = self._convert_ros_to_cv(msg, is_depth=True)
        if raw is None:
            return None, 1.0

        if msg.encoding in ("16UC1", "mono16"):
            # RealSense raw depth in millimeters
            return raw, 0.001
        elif msg.encoding in ("32FC1", "type_32FC1"):
            # Depth directly in meters
            return raw, 1.0
        else:
            return raw, 0.001

    def _lookup_tf_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp,
    ) -> Optional[np.ndarray]:
        """
        Looks up the 4x4 transformation matrix from source_frame to target_frame.
        Returns None if transform is not yet available.
        """
        try:
            ts: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            try:
                # Fallback to latest available transform
                ts: TransformStamped = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
            except Exception:
                return None

        # Convert translation and quaternion into 4x4 matrix
        trans = ts.transform.translation
        rot = ts.transform.rotation

        t_vec = np.array([trans.x, trans.y, trans.z])
        r_mat = self._quaternion_to_rot_matrix(rot.x, rot.y, rot.z, rot.w)

        tf_mat = np.eye(4, dtype=np.float64)
        tf_mat[:3, :3] = r_mat
        tf_mat[:3, 3] = t_vec
        return tf_mat

    @staticmethod
    def _quaternion_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
        """Converts quaternion (x, y, z, w) to 3x3 rotation matrix."""
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float64)

    def _publish_cloud(self, obstacles: list, stamp):
        """Concatenates all obstacle wall points and publishes PointCloud2."""
        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame

        if obstacles:
            all_pts = np.vstack([obs.points_base for obs in obstacles])
        else:
            all_pts = np.empty((0, 3), dtype=np.float32)

        cloud_msg = create_pointcloud2_msg(header, all_pts)
        self.pub_cloud.publish(cloud_msg)

    def _publish_markers(self, obstacles: list, stamp):
        """Publishes 3D bounding boxes and text labels for visualization."""
        marker_array = MarkerArray()

        # Delete previous markers
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        for i, obs in enumerate(obstacles):
            # Center point between left and right bottom endpoints
            cx = 0.5 * (obs.p_left_base[0] + obs.p_right_base[0])
            cy = 0.5 * (obs.p_left_base[1] + obs.p_right_base[1])
            cz = 0.5 * self.detector.wall_height

            dx = obs.p_right_base[0] - obs.p_left_base[0]
            dy = obs.p_right_base[1] - obs.p_left_base[1]
            yaw = math.atan2(dy, dx)

            # 3D Wall Box Marker (semi-transparent cyan)
            box_marker = Marker()
            box_marker.header.frame_id = self.target_frame
            box_marker.header.stamp = stamp
            box_marker.ns = "glass_walls"
            box_marker.id = i * 2
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose.position.x = cx
            box_marker.pose.position.y = cy
            box_marker.pose.position.z = cz

            # Quaternion for yaw rotation
            box_marker.pose.orientation.z = math.sin(yaw / 2.0)
            box_marker.pose.orientation.w = math.cos(yaw / 2.0)

            box_marker.scale.x = max(0.05, obs.width_3d)
            box_marker.scale.y = 0.04  # 4cm wall thickness
            box_marker.scale.z = self.detector.wall_height

            box_marker.color.r = 0.1
            box_marker.color.g = 0.7
            box_marker.color.b = 1.0
            box_marker.color.a = 0.45  # semi-transparent
            box_marker.lifetime.sec = 1
            marker_array.markers.append(box_marker)

            # Text Label Marker
            txt_marker = Marker()
            txt_marker.header.frame_id = self.target_frame
            txt_marker.header.stamp = stamp
            txt_marker.ns = "glass_labels"
            txt_marker.id = i * 2 + 1
            txt_marker.type = Marker.TEXT_VIEW_FACING
            txt_marker.action = Marker.ADD
            txt_marker.pose.position.x = cx
            txt_marker.pose.position.y = cy
            txt_marker.pose.position.z = self.detector.wall_height + 0.2
            txt_marker.scale.z = 0.20  # text size
            txt_marker.color.r = 1.0
            txt_marker.color.g = 1.0
            txt_marker.color.b = 1.0
            txt_marker.color.a = 0.95
            txt_marker.text = f"{obs.detection.class_name} ({obs.distance:.1f}m)"
            txt_marker.lifetime.sec = 1
            marker_array.markers.append(txt_marker)

        self.pub_markers.publish(marker_array)

    def _publish_annotated_image(self, bgr_img: np.ndarray, header: Header):
        """Publishes the annotated image as sensor_msgs/Image."""
        if self.bridge is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding="bgr8")
                img_msg.header = header
                self.pub_annotated.publish(img_msg)
            except Exception:
                pass

    def _publish_status_heartbeat(self):
        """Publishes operational diagnostics as a JSON string."""
        status = {
            "node": "glass_detection_node",
            "model_loaded": self.detector.is_model_loaded(),
            "model_path": self.detector.model_path,
            "classes": self.detector.classes,
            "target_frame": self.target_frame,
            "depth_method": self.detector.depth_method,
            "has_intrinsics": self.cached_intrinsics is not None,
            "frames_processed": self.total_frames_processed,
            "last_detections": self.last_detection_count,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlassDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

