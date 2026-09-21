#!/usr/bin/env python3
"""Calibrate a fixed RealSense camera to OpenYAM and update a local JSON file.

Use a dedicated calibration environment, for example:
uv run --no-project --with numpy --with opencv-contrib-python --with pyrealsense2 \
  python tools/calibrate_openyam_camera.py --output local-setup/openyam_bench.json

This utility opens only the camera. It never imports DimOS robot modules or
starts a coordinator.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

# RealSense color images use the optical convention (+X right, +Y down, +Z
# forward).  DimOS publishes the camera rigid-body frame as ``camera_link``
# (+X forward, +Y left, +Z up).  PnP estimates the former; the blueprint needs
# the latter.
CAMERA_LINK_FROM_COLOR_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rotation_matrix_from_quaternion(quaternion_xyzw: list[float]) -> np.ndarray:
    """Return a 3x3 rotation matrix for an xyzw quaternion."""
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = np.linalg.norm([x, y, z, w])
    if norm == 0:
        raise ValueError("tag quaternion must be non-zero")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> list[float]:
    """Convert a proper 3x3 rotation matrix to an xyzw quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = 2 * np.sqrt(trace + 1.0)
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
        w = 0.25 * scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = 2 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            x, y, z, w = 0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = 2 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            x, y, z, w = (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = 2 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            x, y, z, w = (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale
    return [float(x), float(y), float(z), float(w)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--tag-size", type=float, required=True, metavar="METRES")
    parser.add_argument("--tag-in-world", nargs=3, type=float, required=True)
    parser.add_argument(
        "--tag-quaternion-xyzw", nargs=4, type=float, default=[0.0, 0.0, 0.0, 1.0],
        help="AprilTag orientation in the OpenYAM world frame (default: identity)",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.tag_size <= 0:
        parser.error("--tag-size must be positive")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
    profile = pipeline.start(config)
    try:
        for _ in range(30):
            pipeline.wait_for_frames(5000)
        frame = pipeline.wait_for_frames(5000).get_color_frame()
        image = np.asanyarray(frame.get_data())
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        if ids is None or len(ids) != 1:
            raise RuntimeError("exactly one AprilTag must be visible")
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        half = args.tag_size / 2
        object_points = np.array(
            [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
            dtype=np.float64,
        )
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            corners[0].reshape(4, 2),
            np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]]),
            np.asarray(intr.coeffs),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            raise RuntimeError("AprilTag pose estimation failed")
        rotation, _ = cv2.Rodrigues(rvec)
        world_from_tag = np.eye(4)
        world_from_tag[:3, :3] = rotation_matrix_from_quaternion(args.tag_quaternion_xyzw)
        world_from_tag[:3, 3] = args.tag_in_world
        color_from_tag = np.eye(4)
        color_from_tag[:3, :3] = rotation
        color_from_tag[:3, 3] = tvec.reshape(3)
        world_from_color_optical = world_from_tag @ np.linalg.inv(color_from_tag)
        camera_link_from_color_optical = np.eye(4)
        camera_link_from_color_optical[:3, :3] = CAMERA_LINK_FROM_COLOR_OPTICAL
        world_from_camera_link = (
            world_from_color_optical @ np.linalg.inv(camera_link_from_color_optical)
        )
        data = json.loads(args.output.read_text()) if args.output.exists() else {}
        data.update(
            camera_serial=args.serial,
            camera_translation_m=world_from_camera_link[:3, 3].tolist(),
            camera_quaternion_xyzw=quaternion_from_rotation_matrix(
                world_from_camera_link[:3, :3]
            ),
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2) + "\n")
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
