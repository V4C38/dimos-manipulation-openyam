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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--tag-size", type=float, required=True, metavar="METRES")
    parser.add_argument("--tag-in-world", nargs=3, type=float, required=True)
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
        world_from_tag[:3, 3] = args.tag_in_world
        color_from_tag = np.eye(4)
        color_from_tag[:3, :3] = rotation
        color_from_tag[:3, 3] = tvec.reshape(3)
        world_from_color = world_from_tag @ np.linalg.inv(color_from_tag)
        q = cv2.Rodrigues(world_from_color[:3, :3])[0].reshape(3)
        data = json.loads(args.output.read_text()) if args.output.exists() else {}
        data.update(
            camera_serial=args.serial,
            camera_translation_m=world_from_color[:3, 3].tolist(),
            camera_rotation_rodrigues= q.tolist(),
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2) + "\n")
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
