"""
Glass obstacle detector and 3D point cloud generator.

Uses a zero-shot open-vocabulary model (YOLO-World) to detect transparent
and translucent glass obstacles (doors, walls, windows) in RGB imagery,
estimates their 3D physical coordinates using ground-plane ray intersection
and/or frame-edge depth sampling, and produces synthetic 3D point clouds
for ingestion into Nav2 costmaps.
"""

from dataclasses import dataclass
import math
import struct
from typing import Dict, List, Optional, Tuple, Union

import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


@dataclass
class Detection2D:
    """2D object detection bounding box."""
    u_min: float
    v_min: float
    u_max: float
    v_max: float
    confidence: float
    class_name: str
    class_id: int = 0


@dataclass
class GlassObstacle3D:
    """Detected glass obstacle with estimated 3D geometry."""
    # Bottom endpoints in camera optical frame [X (right), Y (down), Z (forward)]
    p_left_optical: Tuple[float, float, float]
    p_right_optical: Tuple[float, float, float]
    # Bottom endpoints in robot base frame [X (forward), Y (left), Z (up)]
    p_left_base: Tuple[float, float, float]
    p_right_base: Tuple[float, float, float]
    distance: float
    width_3d: float
    detection: Detection2D
    method_used: str  # "frame_depth" or "ground_plane"
    points_base: np.ndarray  # Nx3 synthetic obstacle points in base frame


class GlassDetector:
    """
    Detects glass doors/walls and projects them into 3D obstacle point clouds.
    """

    DEFAULT_CLASSES = [
        "glass door",
        "glass wall",
        "glass window",
        "transparent door",
        "door frame",
        "sliding glass door",
    ]

    def __init__(
        self,
        model_path: str = "yolov8s-world.pt",
        classes: Optional[List[str]] = None,
        conf_threshold: float = 0.25,
        camera_height: float = 0.30,       # meters above ground
        camera_pitch_deg: float = 0.0,      # degrees, tilted down is positive
        depth_method: str = "hybrid",       # "hybrid", "ground_plane", "frame_depth"
        wall_height: float = 1.5,           # meters, vertical obstacle wall height
        point_spacing: float = 0.05,        # meters, horizontal point resolution
        vertical_spacing: float = 0.05,     # meters, vertical point resolution
        frame_strip_px: int = 10,           # pixels along bottom edge to sample depth
        min_depth: float = 0.2,             # meters
        max_depth: float = 8.0,             # meters
        device: Optional[str] = None,
    ):
        self.model_path = model_path
        self.classes = classes if classes is not None else list(self.DEFAULT_CLASSES)
        self.conf_threshold = conf_threshold
        self.camera_height = float(camera_height)
        self.camera_pitch_rad = math.radians(float(camera_pitch_deg))
        self.depth_method = depth_method.lower()
        self.wall_height = float(wall_height)
        self.point_spacing = max(0.01, float(point_spacing))
        self.vertical_spacing = max(0.01, float(vertical_spacing))
        self.frame_strip_px = max(1, int(frame_strip_px))
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.device = device

        self.model = None
        self._load_model()

    def _load_model(self):
        """Loads the pre-trained YOLO-World model if available."""
        if YOLO is None:
            return

        try:
            self.model = YOLO(self.model_path)
            if hasattr(self.model, "set_classes") and self.classes:
                self.model.set_classes(self.classes)
        except Exception as e:
            self.model = None

    def is_model_loaded(self) -> bool:
        """Returns True if the YOLO model is successfully initialized."""
        return self.model is not None

    def detect_2d(self, image_bgr_or_rgb: np.ndarray) -> List[Detection2D]:
        """
        Runs YOLO-World zero-shot detection on the input frame.
        """
        if self.model is None:
            return []

        try:
            results = self.model.predict(
                image_bgr_or_rgb,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception:
            return []

        detections = []
        if not results:
            return detections

        res = results[0]
        if not hasattr(res, "boxes") or res.boxes is None:
            return detections

        boxes = res.boxes
        coords = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else boxes.xyxy
        confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else boxes.conf
        cls_ids = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else boxes.cls

        names = res.names if hasattr(res, "names") else {}

        for i in range(len(coords)):
            u_min, v_min, u_max, v_max = coords[i]
            conf = float(confs[i])
            cls_id = int(cls_ids[i])
            class_name = names.get(cls_id, f"class_{cls_id}")

            detections.append(
                Detection2D(
                    u_min=float(u_min),
                    v_min=float(v_min),
                    u_max=float(u_max),
                    v_max=float(v_max),
                    confidence=conf,
                    class_name=class_name,
                    class_id=cls_id,
                )
            )

        return detections

    def project_pixel_ground_plane(
        self,
        u: float,
        v: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        camera_height: Optional[float] = None,
        pitch_rad: Optional[float] = None,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Calculates 3D coordinates (X, Y, Z) in camera optical frame for a ground pixel
        using ray-ground plane intersection:

            Z = h / (sin(theta) + ((v - c_y) / f_y) * cos(theta))
            X = (u - c_x) * Z / f_x
            Y = (v - c_y) * Z / f_y
        """
        h = camera_height if camera_height is not None else self.camera_height
        theta = pitch_rad if pitch_rad is not None else self.camera_pitch_rad

        if fy <= 0.0 or fx <= 0.0:
            return None

        # Normalized ray slope downwards in optical frame
        vy = (v - cy) / fy
        denom = math.sin(theta) + vy * math.cos(theta)

        # If denominator <= 0, the ray points above the horizon line (never hits floor)
        if denom <= 1e-4:
            return None

        z = h / denom
        if z < self.min_depth or z > self.max_depth:
            return None

        x = ((u - cx) / fx) * z
        y = vy * z

        return (x, y, z)

    def sample_frame_depth(
        self,
        depth_img: np.ndarray,
        u_min: float,
        v_max: float,
        u_max: float,
        depth_scale_to_meters: float = 1.0,
    ) -> Optional[Tuple[float, float]]:
        """
        Samples depth along the bottom 5-10 pixels of the bounding box where solid
        frame material (baseboard, frame extrusion) is present.
        Returns (depth_left, depth_right) in meters, or None if no valid frame depth found.
        """
        h, w = depth_img.shape[:2]
        v_bottom = int(min(h - 1, max(0, v_max)))
        v_top = int(max(0, v_bottom - self.frame_strip_px))

        u_start = int(max(0, min(w - 1, u_min)))
        u_end = int(max(0, min(w, u_max)))

        if v_bottom <= v_top or u_end <= u_start:
            return None

        strip = depth_img[v_top:v_bottom, u_start:u_end].astype(np.float32) * depth_scale_to_meters

        # Filter out invalid, zero, or out-of-range depths
        valid_mask = (strip >= self.min_depth) & (strip <= self.max_depth) & np.isfinite(strip)
        if np.count_nonzero(valid_mask) < 4:
            return None

        width_strip = u_end - u_start
        mid_col = width_strip // 2

        left_half = strip[:, :mid_col]
        left_valid = left_half[(left_half >= self.min_depth) & (left_half <= self.max_depth) & np.isfinite(left_half)]

        right_half = strip[:, mid_col:]
        right_valid = right_half[(right_half >= self.min_depth) & (right_half <= self.max_depth) & np.isfinite(right_half)]

        all_valid = strip[valid_mask]
        overall_median = float(np.median(all_valid))

        z_left = float(np.median(left_valid)) if len(left_valid) >= 2 else overall_median
        z_right = float(np.median(right_valid)) if len(right_valid) >= 2 else overall_median

        return (z_left, z_right)

    def optical_to_base_frame(
        self,
        p_optical: Tuple[float, float, float],
        pitch_rad: Optional[float] = None,
        camera_height: Optional[float] = None,
        x_offset: float = 0.15,
        y_offset: float = 0.0,
    ) -> Tuple[float, float, float]:
        """
        Converts a 3D point from camera optical frame to robot base frame.
        Optical frame: X right, Y down, Z forward.
        Base frame:    X forward, Y left, Z up.
        """
        xo, yo, zo = p_optical
        theta = pitch_rad if pitch_rad is not None else self.camera_pitch_rad
        h = camera_height if camera_height is not None else self.camera_height

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # Un-pitch the ray: forward component and downward component
        x_base = zo * cos_t - yo * sin_t + x_offset
        y_base = -xo + y_offset
        z_base = -zo * sin_t - yo * cos_t + h

        return (x_base, y_base, z_base)

    def process_detections(
        self,
        detections: List[Detection2D],
        intrinsics: Tuple[float, float, float, float],  # (fx, fy, cx, cy)
        depth_img: Optional[np.ndarray] = None,
        depth_scale_to_meters: float = 1.0,
        tf_transform_matrix: Optional[np.ndarray] = None,  # 4x4 matrix from optical to base
    ) -> List[GlassObstacle3D]:
        """
        Transforms 2D detections into 3D glass obstacles and synthetic wall points.
        """
        fx, fy, cx, cy = intrinsics
        obstacles = []

        for det in detections:
            method_used = "ground_plane"
            p_left_opt = None
            p_right_opt = None

            # Try Method B (Frame-Edge Depth) if requested and depth image provided
            if self.depth_method in ("hybrid", "frame_depth") and depth_img is not None:
                depth_pair = self.sample_frame_depth(
                    depth_img,
                    det.u_min,
                    det.v_max,
                    det.u_max,
                    depth_scale_to_meters=depth_scale_to_meters,
                )
                if depth_pair is not None:
                    z_l, z_r = depth_pair
                    xl = ((det.u_min - cx) / fx) * z_l
                    yl = ((det.v_max - cy) / fy) * z_l
                    xr = ((det.u_max - cx) / fx) * z_r
                    yr = ((det.v_max - cy) / fy) * z_r
                    p_left_opt = (xl, yl, z_l)
                    p_right_opt = (xr, yr, z_r)
                    method_used = "frame_depth"

            # Fall back to Method A (Ground-Plane Intersection)
            if p_left_opt is None or p_right_opt is None:
                if self.depth_method == "frame_depth":
                    continue  # Strict frame depth only, discard if no solid frame

                p_left_opt = self.project_pixel_ground_plane(det.u_min, det.v_max, fx, fy, cx, cy)
                p_right_opt = self.project_pixel_ground_plane(det.u_max, det.v_max, fx, fy, cx, cy)
                method_used = "ground_plane"

            if p_left_opt is None or p_right_opt is None:
                continue

            # Transform bottom endpoints to base frame
            if tf_transform_matrix is not None:
                p_left_base = self._apply_tf(p_left_opt, tf_transform_matrix)
                p_right_base = self._apply_tf(p_right_opt, tf_transform_matrix)
            else:
                p_left_base = self.optical_to_base_frame(p_left_opt)
                p_right_base = self.optical_to_base_frame(p_right_opt)

            # Clamp floor Z coordinate close to ground level (Z ≈ 0 in base_link)
            p_left_base = (p_left_base[0], p_left_base[1], max(0.0, p_left_base[2]))
            p_right_base = (p_right_base[0], p_right_base[1], max(0.0, p_right_base[2]))

            dx = p_right_base[0] - p_left_base[0]
            dy = p_right_base[1] - p_left_base[1]
            width_3d = math.hypot(dx, dy)
            center_x = 0.5 * (p_left_base[0] + p_right_base[0])
            center_y = 0.5 * (p_left_base[1] + p_right_base[1])
            dist = math.hypot(center_x, center_y)

            # Generate dense synthetic point cloud for the virtual glass wall
            points = self.generate_wall_cloud(p_left_base, p_right_base)

            obstacles.append(
                GlassObstacle3D(
                    p_left_optical=p_left_opt,
                    p_right_optical=p_right_opt,
                    p_left_base=p_left_base,
                    p_right_base=p_right_base,
                    distance=dist,
                    width_3d=width_3d,
                    detection=det,
                    method_used=method_used,
                    points_base=points,
                )
            )

        return obstacles

    def generate_wall_cloud(
        self,
        p_left_base: Tuple[float, float, float],
        p_right_base: Tuple[float, float, float],
    ) -> np.ndarray:
        """
        Interpolates points horizontally along the base of the glass obstacle
        and extrudes vertically up to wall_height to form a 3D barrier cloud.
        """
        xl, yl, zl = p_left_base
        xr, yr, zr = p_right_base

        dx = xr - xl
        dy = yr - yl
        width = math.hypot(dx, dy)

        # Number of horizontal and vertical steps
        n_h = max(2, int(math.ceil(width / self.point_spacing)) + 1)
        n_v = max(2, int(math.ceil(self.wall_height / self.vertical_spacing)) + 1)

        alphas = np.linspace(0.0, 1.0, n_h, dtype=np.float32)
        base_x = (1.0 - alphas) * xl + alphas * xr
        base_y = (1.0 - alphas) * yl + alphas * yr
        base_z = (1.0 - alphas) * zl + alphas * zr

        # Vertical heights from ground level up to wall_height
        v_offsets = np.linspace(0.0, self.wall_height, n_v, dtype=np.float32)

        # Meshgrid coordinates
        bx_grid, vz_grid = np.meshgrid(base_x, v_offsets)
        by_grid, _ = np.meshgrid(base_y, v_offsets)
        bz_grid, _ = np.meshgrid(base_z, v_offsets)

        final_z = bz_grid + vz_grid

        # Flatten into Nx3 point array
        points = np.stack([bx_grid.flatten(), by_grid.flatten(), final_z.flatten()], axis=1)
        return points

    def _apply_tf(
        self,
        point: Tuple[float, float, float],
        tf_mat: np.ndarray,
    ) -> Tuple[float, float, float]:
        """Applies a 4x4 transformation matrix to a 3D point."""
        vec = np.array([point[0], point[1], point[2], 1.0], dtype=np.float64)
        transformed = tf_mat @ vec
        return (float(transformed[0]), float(transformed[1]), float(transformed[2]))

    def annotate_image(
        self,
        image: np.ndarray,
        obstacles: List[GlassObstacle3D],
    ) -> np.ndarray:
        """
        Draws 2D bounding boxes, labels, distances, and method tags on the image.
        """
        import cv2

        annotated = image.copy()

        for obs in obstacles:
            det = obs.detection
            u1, v1 = int(det.u_min), int(det.v_min)
            u2, v2 = int(det.u_max), int(det.v_max)

            # Color code: Cyan for ground_plane, Orange/Magenta for frame_depth
            box_color = (255, 200, 0) if obs.method_used == "frame_depth" else (0, 220, 255)

            # Bounding box
            cv2.rectangle(annotated, (u1, v1), (u2, v2), box_color, 2)

            # Bottom contact line (emphasized)
            cv2.line(annotated, (u1, v2), (u2, v2), (0, 0, 255), 3)

            label = f"{det.class_name} {det.confidence:.2f} | {obs.distance:.2f}m ({obs.method_used})"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)

            cv2.rectangle(annotated, (u1, max(0, v1 - th - 6)), (u1 + tw + 4, max(th + 6, v1)), box_color, -1)
            cv2.putText(
                annotated,
                label,
                (u1 + 2, max(th, v1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

        return annotated


def create_pointcloud2_msg(
    header,
    points_xyz: np.ndarray,
):
    """
    Creates a sensor_msgs/PointCloud2 message from an Nx3 array of float32 points.
    Encodes via fast binary packing without requiring external C extensions.
    """
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points_xyz)
    msg.is_dense = True
    msg.is_bigendian = False

    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width

    if len(points_xyz) > 0:
        pts = points_xyz.astype(np.float32)
        msg.data = pts.tobytes()
    else:
        msg.data = b""

    return msg

