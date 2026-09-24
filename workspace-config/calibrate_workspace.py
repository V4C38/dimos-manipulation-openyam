#!/usr/bin/env python3
"""Align the fixed RGB-D camera to the arm using the AprilTag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

LOCAL = Path(__file__).resolve().parent
BODY_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def camera_link_from_color(camera_info, serial: str) -> tuple[np.ndarray, np.ndarray]:
    """Return color intrinsics and the RealSense color-to-link transform."""
    import pyrealsense2 as rs

    context = rs.context()
    devices = [d for d in context.query_devices()
               if d.get_info(rs.camera_info.serial_number) == serial]
    if len(devices) != 1:
        raise RuntimeError(f"Configured RGB-D camera {serial} is not connected")
    profiles = [p.as_video_stream_profile() for sensor in devices[0].query_sensors()
                for p in sensor.get_stream_profiles() if p.is_video_stream_profile()]
    color = next(p for p in profiles if p.stream_type() == rs.stream.color
                 and p.width() == camera_info.width and p.height() == camera_info.height)
    depth = next(p for p in profiles if p.stream_type() == rs.stream.depth)
    intr = color.get_intrinsics()
    if any(abs(v) > 1e-10 for v in intr.coeffs):
        raise ValueError("Rectified color intrinsics are required")
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=float)
    if not np.allclose(K, np.asarray(camera_info.K).reshape(3, 3)) or np.any(np.asarray(camera_info.D)):
        raise RuntimeError("Live camera intrinsics do not match the connected camera")
    ext = color.get_extrinsics_to(depth)
    link_from_color = np.eye(4)
    link_from_color[:3, :3] = BODY_FROM_OPTICAL @ np.asarray(ext.rotation).reshape(3, 3, order="F")
    link_from_color[:3, 3] = BODY_FROM_OPTICAL @ np.asarray(ext.translation)
    return K, link_from_color


def live_color_frames(seconds: float) -> tuple[list[np.ndarray], object]:
    """Read fresh color frames and camera info from the running RGB-D stream."""
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.protocol.pubsub.impl.zenohpubsub import Topic, ZenohPubSubBase

    received = {"color_image": [], "camera_info": []}
    lock = threading.Lock()

    def receive(data, topic):
        name = str(topic).split("#")[0].split("/")[-1]
        if name not in received:
            return
        message = (CameraInfo if name == "camera_info" else Image).lcm_decode(data)
        with lock:
            received[name].append(message)

    bus = ZenohPubSubBase()
    bus.start()
    unsubscribe = bus.subscribe(Topic("dimos/**"), receive)
    try:
        time.sleep(seconds)
    finally:
        unsubscribe()
        bus.stop()
    if not received["color_image"] or not received["camera_info"]:
        raise RuntimeError("No live RGB camera stream; start the grasp blueprint first")
    info = received["camera_info"][-1]
    frames = [m.to_opencv().copy() for m in received["color_image"]
              if m.frame_id == info.frame_id and m.shape[:2] == (info.height, info.width)]
    if not frames:
        raise RuntimeError("No color frames matched the live camera info")
    return frames, info


def detect_tag_pose(images: list[np.ndarray], K: np.ndarray, tag_id: int,
                    tag_size: float) -> tuple[np.ndarray, int, float]:
    """Detect and average the fixed tag pose in camera optical coordinates."""
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params
    )
    h = tag_size / 2
    corners_3d = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
    poses, errors = [], []
    for image in images:
        corners, ids, _ = detector.detectMarkers(image)
        if ids is None or list(ids.ravel()).count(tag_id) != 1:
            continue
        pixels = corners[list(ids.ravel()).index(tag_id)].reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(corners_3d, pixels, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or tvec[2, 0] <= 0:
            continue
        projected, _ = cv2.projectPoints(corners_3d, rvec, tvec, K, None)
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels) ** 2, axis=1))))
        if rms > 1:
            continue
        pose = np.eye(4)
        pose[:3, :3] = cv2.Rodrigues(rvec)[0]
        pose[:3, 3] = tvec.ravel()
        poses.append(pose)
        errors.append(rms)
    if not poses:
        raise RuntimeError(f"AprilTag {tag_id} was not detected with a valid pose")
    mean = np.eye(4)
    mean[:3, :3] = Rotation.from_matrix(np.stack([p[:3, :3] for p in poses])).mean().as_matrix()
    mean[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    return mean, len(poses), float(np.mean(errors))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=LOCAL / "openyam_bench.json")
    parser.add_argument("--seconds", type=float, default=3, help="Live tag observation duration")
    parser.add_argument("--apply", action="store_true", help="Write the aligned camera pose to the profile")
    args = parser.parse_args()
    if not 0 < args.seconds <= 30:
        parser.error("--seconds must be between 0 and 30")

    before = args.config.read_text()
    config = json.loads(before)
    images, camera_info = live_color_frames(args.seconds)
    K, link_from_color = camera_link_from_color(camera_info, config["camera_serial"])
    tag_id = int(config.get("calibration_tag_id", 0))
    tag_size = float(config.get("calibration_tag_size_m", 0.056))
    camera_from_tag, count, rms = detect_tag_pose(images, K, tag_id, tag_size)

    # Arm frame: +X forward, +Y left, +Z up. Apply the measured 70 mm tag
    # offset plus the requested 70 mm correction toward the arm's right.
    # This makes the tag center Y=-140 mm from the arm axis.
    world_from_tag = np.eye(4)
    world_from_tag[:3, :3] = Rotation.from_quat(
        config.get("calibration_tag_quaternion_xyzw", [0, 0, 0, 1])
    ).as_matrix()
    world_from_tag[:3, 3] = [0.0, -0.140, 0.0]
    world_from_camera_color = world_from_tag @ np.linalg.inv(camera_from_tag)
    world_from_camera_link = world_from_camera_color @ np.linalg.inv(link_from_color)

    config.update(
        camera_translation_m=world_from_camera_link[:3, 3].tolist(),
        camera_quaternion_xyzw=Rotation.from_matrix(world_from_camera_link[:3, :3]).as_quat().tolist(),
        calibration_tag_center_from_arm_axis_m=[0.0, -0.140, 0.0],
        bench_top_z_m=-0.020,
        calibration_reference_notes="Applied tag offset plus 70 mm rightward correction: tag center Y=-140 mm from arm axis; bench top is 20 mm below datum.",
    )
    report = {
        "tag_id": tag_id,
        "tag_center_from_arm_axis_m": [0.0, -0.140, 0.0],
        "arm_axis_from_tag_m": [0.0, 0.140, 0.0],
        "bench_top_z_m": -0.020,
        "observations": count,
        "mean_reprojection_error_px": rms,
        "camera_translation_m": config["camera_translation_m"],
    }
    print(json.dumps(report, indent=2))
    if args.apply:
        if args.config.read_text() != before:
            raise RuntimeError("Workspace profile changed during calibration")
        config["workspace_calibration"] = {
            "method": "live AprilTag alignment with fixed 70 mm tag-to-arm offset",
            "scene_status": "Static scene unchanged; camera alignment updated",
        }
        temporary = args.config.with_name(args.config.name + ".calibration-tmp")
        with temporary.open("x") as stream:
            json.dump(config, stream, indent=2)
            stream.write("\n")
        temporary.replace(args.config)
        print("Workspace profile updated; restart the blueprint to load it.")


if __name__ == "__main__":
    main()
