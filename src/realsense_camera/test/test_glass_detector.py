"""
Unit tests for GlassDetector geometry, depth sampling, and point cloud generation.
"""

import math
import struct
import unittest
import numpy as np

from realsense_camera.glass_detector import (
    GlassDetector,
    Detection2D,
    create_pointcloud2_msg,
)


class TestGlassDetector(unittest.TestCase):
    def setUp(self):
        self.detector = GlassDetector(
            model_path="dummy.pt",
            camera_height=0.30,
            camera_pitch_deg=0.0,
            depth_method="hybrid",
            wall_height=1.5,
            point_spacing=0.10,
            vertical_spacing=0.10,
            min_depth=0.2,
            max_depth=10.0,
        )
        # Mock intrinsics: 640x480 camera
        self.fx = 384.0
        self.fy = 384.0
        self.cx = 320.0
        self.cy = 240.0

    def test_ground_plane_projection_math(self):
        """Verifies deterministic ground plane intersection formula."""
        # For pixel at bottom edge v=480:
        # vy = (480 - 240) / 384 = 240 / 384 = 0.625
        # With h = 0.30 and theta = 0: Z = 0.30 / 0.625 = 0.48m
        res = self.detector.project_pixel_ground_plane(
            u=320.0,
            v=480.0,
            fx=self.fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy,
            camera_height=0.30,
            pitch_rad=0.0,
        )
        self.assertIsNotNone(res)
        x, y, z = res
        self.assertAlmostEqual(z, 0.48, places=2)
        self.assertAlmostEqual(x, 0.0, places=2)
        self.assertAlmostEqual(y, 0.30, places=2)

    def test_ground_plane_horizon_rejection(self):
        """Pixels above the horizon (v <= cy when theta=0) must not hit the floor."""
        # v = 200 is above optical center (cy=240), pointing upwards
        res = self.detector.project_pixel_ground_plane(
            u=320.0,
            v=200.0,
            fx=self.fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy,
            camera_height=0.30,
            pitch_rad=0.0,
        )
        self.assertIsNone(res)

    def test_sample_frame_depth(self):
        """Verifies frame-edge depth sampling along the bottom pixels."""
        # Create a synthetic 480x640 depth image with 0.0 everywhere except
        # a solid bottom rail at 2.5 meters
        depth_img = np.zeros((480, 640), dtype=np.float32)
        depth_img[470:480, 100:300] = 2.50  # 2.5m frame

        depths = self.detector.sample_frame_depth(
            depth_img=depth_img,
            u_min=100,
            v_max=480,
            u_max=300,
            depth_scale_to_meters=1.0,
        )
        self.assertIsNotNone(depths)
        zl, zr = depths
        self.assertAlmostEqual(zl, 2.50, places=2)
        self.assertAlmostEqual(zr, 2.50, places=2)

    def test_sample_frame_depth_transparent(self):
        """When glass is transparent with zero depth readings, sampling returns None."""
        depth_img = np.zeros((480, 640), dtype=np.float32)  # All zero (no IR return)
        depths = self.detector.sample_frame_depth(
            depth_img=depth_img,
            u_min=100,
            v_max=480,
            u_max=300,
            depth_scale_to_meters=1.0,
        )
        self.assertIsNone(depths)

    def test_generate_wall_cloud(self):
        """Verifies horizontal interpolation and vertical extrusion."""
        p_left = (-0.5, 2.0, 0.0)
        p_right = (0.5, 2.0, 0.0)

        cloud = self.detector.generate_wall_cloud(p_left, p_right)
        self.assertEqual(len(cloud.shape), 2)
        self.assertEqual(cloud.shape[1], 3)
        self.assertGreater(len(cloud), 10)

        # Check bounds
        x_min, y_min, z_min = cloud.min(axis=0)
        x_max, y_max, z_max = cloud.max(axis=0)

        self.assertAlmostEqual(x_min, -0.5, places=1)
        self.assertAlmostEqual(x_max, 0.5, places=1)
        self.assertAlmostEqual(z_min, 0.0, places=1)
        self.assertAlmostEqual(z_max, self.detector.wall_height, delta=0.1)

    def test_create_pointcloud2_msg(self):
        """Verifies binary packaging of PointCloud2 messages."""
        from std_msgs.msg import Header

        header = Header()
        header.frame_id = "base_link"

        pts = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
        ], dtype=np.float32)

        msg = create_pointcloud2_msg(header, pts)
        self.assertEqual(msg.header.frame_id, "base_link")
        self.assertEqual(msg.width, 2)
        self.assertEqual(msg.height, 1)
        self.assertEqual(len(msg.data), 2 * 12)  # 2 points * 12 bytes

        # Unpack first point
        x, y, z = struct.unpack_from("fff", msg.data, 0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(z, 3.0)


if __name__ == "__main__":
    unittest.main()

