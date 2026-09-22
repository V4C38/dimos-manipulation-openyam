#!/usr/bin/env python3
"""Capture, inspect, and calibrate this workspace without commanding the arm.

Camera pose is solved afresh from a base-fixed AprilTag. Use --help and README.md.
Only --apply replaces the bench configuration; every run saves replayable evidence.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import threading

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

LOCAL = Path(__file__).resolve().parent
BODY_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def transform(position: list, quaternion: list) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = position
    return matrix


def camera_metadata(color_profile, depth_profile) -> dict:
    intr = color_profile.get_intrinsics()
    if any(abs(v) > 1e-10 for v in intr.coeffs):
        raise ValueError("This utility requires rectified/zero-distortion color intrinsics")
    ext = color_profile.get_extrinsics_to(depth_profile)
    link_from_color = np.eye(4)
    link_from_color[:3, :3] = BODY_FROM_OPTICAL @ np.asarray(ext.rotation).reshape(3, 3, order="F")
    link_from_color[:3, 3] = BODY_FROM_OPTICAL @ np.asarray(ext.translation)
    return {"K": [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
            "link_from_color": link_from_color.tolist(), "width": intr.width, "height": intr.height}


def capture_direct(args, serial: str) -> tuple[np.ndarray, np.ndarray, dict]:
    import pyrealsense2 as rs

    pipeline, config = rs.pipeline(), rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = pipeline.start(config)
    try:
        metadata = camera_metadata(profile.get_stream(rs.stream.color).as_video_stream_profile(),
                                   profile.get_stream(rs.stream.depth).as_video_stream_profile())
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        for _ in range(30):
            pipeline.wait_for_frames(5000)
        colors, depths = [], []
        for _ in range(args.samples):
            frames = align.process(pipeline.wait_for_frames(5000))
            colors.append(np.asanyarray(frames.get_color_frame().get_data()).copy())
            depths.append(np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) * scale)
        metadata.update(serial=serial, source="direct", fps=args.fps)
        return np.stack(colors), np.stack(depths), metadata
    finally:
        pipeline.stop()


def capture_live(args, serial: str) -> tuple[np.ndarray, np.ndarray, dict]:
    import pyrealsense2 as rs
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.protocol.pubsub.impl.zenohpubsub import Topic, ZenohPubSubBase

    frames = {"color_image": [], "depth_image": [], "camera_info": []}
    lock = threading.Lock()

    def receive(data, topic):
        name = str(topic).split("#")[0].split("/")[-1]
        if name in frames:
            message = (CameraInfo if name == "camera_info" else Image).lcm_decode(data)
            with lock:
                frames[name].append(message)

    bus = ZenohPubSubBase()
    bus.start()
    unsubscribe = bus.subscribe(Topic("dimos/**"), receive)
    try:
        threading.Event().wait(args.seconds)
    finally:
        unsubscribe()
        bus.stop()
    if any(not values for values in frames.values()):
        raise RuntimeError("No complete live RGB-D stream; use --source direct when the camera is free")
    info = frames["camera_info"][-1]
    context = rs.context()
    devices = list(context.query_devices())
    if len(devices) != 1 or devices[0].get_info(rs.camera_info.serial_number) != serial:
        raise RuntimeError("Live capture requires one connected camera matching the configured serial")
    profiles = [p.as_video_stream_profile() for s in devices[0].query_sensors()
                for p in s.get_stream_profiles() if p.is_video_stream_profile()]
    color = next(p for p in profiles if p.stream_type() == rs.stream.color
                 and p.width() == info.width and p.height() == info.height)
    depth = next(p for p in profiles if p.stream_type() == rs.stream.depth)
    metadata = camera_metadata(color, depth)
    if not np.allclose(metadata["K"], np.asarray(info.K).reshape(3, 3)) or np.any(np.asarray(info.D)):
        raise RuntimeError("Live camera intrinsics do not match the connected camera")
    colors, depths = [], []
    used = set()
    for image in frames["color_image"]:
        d = min(frames["depth_image"], key=lambda x: abs(x.ts-image.ts))
        if abs(d.ts-image.ts) > 0.03 or d.ts in used:
            continue
        if image.frame_id != d.frame_id or image.frame_id != info.frame_id:
            raise RuntimeError("Live depth must be aligned to the color optical frame")
        raw = d.to_opencv()
        if raw.dtype != np.uint16 or raw.shape != image.shape[:2]:
            raise RuntimeError("Expected aligned native DEPTH16 millimetres")
        used.add(d.ts)
        colors.append(image.to_opencv().copy())
        depths.append(raw.astype(np.float32) / 1000)
    if len(colors) < 12:
        raise RuntimeError(f"Only {len(colors)} synchronized frames; increase --seconds")
    metadata.update(serial=serial, source="live")
    return np.stack(colors), np.stack(depths), metadata


def detect_poses(colors: np.ndarray, K: np.ndarray, tag_id: int, size: float) -> list[dict]:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
    h = size / 2
    points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    rows = []
    for index, image in enumerate(colors):
        corners, ids, _ = detector.detectMarkers(image)
        if ids is None or list(ids.ravel()).count(tag_id) != 1:
            continue
        pixels = corners[list(ids.ravel()).index(tag_id)].reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(points, pixels, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or tvec[2, 0] <= 0:
            continue
        projected, _ = cv2.projectPoints(points, rvec, tvec, K, None)
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2)-pixels)**2, axis=1))))
        if rms > 1:
            continue
        pose = np.eye(4)
        pose[:3, :3] = cv2.Rodrigues(rvec)[0]
        pose[:3, 3] = tvec.ravel()
        rows.append({"frame": index, "pose": pose, "reprojection_rms_px": rms})
    if len(rows) < 12:
        raise RuntimeError(f"Only {len(rows)} usable tag frames; need 12 with reprojection RMS <=1 pixel")
    return rows


def mean_pose(poses: list[np.ndarray]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_matrix(np.stack([p[:3, :3] for p in poses])).mean().as_matrix()
    result[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    return result


def select_axis(image: np.ndarray, axis: str) -> list[float]:
    """Select a directed base edge in the current frame, not an old ROI."""
    points = []
    title = f"Click two points along plate +{axis.upper()} (start then end); Enter accepts, Esc cancels"

    def click(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
            points.append((x, y))

    cv2.namedWindow(title)
    cv2.setMouseCallback(title, click)
    try:
        while True:
            view = image.copy()
            for p in points:
                cv2.circle(view, p, 4, (0, 0, 255), -1)
            cv2.imshow(title, view)
            key = cv2.waitKey(30) & 255
            if key == 27:
                raise RuntimeError("Axis selection cancelled")
            if key in (10, 13) and len(points) == 2:
                return list(np.asarray(points).ravel().astype(float))
    finally:
        cv2.destroyWindow(title)


def plate_reference(pose: np.ndarray, image: np.ndarray, K: np.ndarray,
                    pixels: list[float], axis: str) -> tuple[np.ndarray, dict]:
    """Recover tag orientation from its normal and a directed physical plate edge."""
    endpoints = np.asarray(pixels).reshape(2, 2)
    direction = endpoints[1]-endpoints[0]
    length = np.linalg.norm(direction)
    if length < 40:
        raise ValueError("Select a plate edge at least 40 pixels long")
    direction /= length
    edge_pixels = np.column_stack(np.nonzero(cv2.Canny(image, 50, 150)))[:, ::-1].astype(float)
    delta = edge_pixels-endpoints[0]
    along = delta @ direction
    perpendicular = np.abs(delta[:, 0]*direction[1]-delta[:, 1]*direction[0])
    selected = edge_pixels[(along >= 0) & (along <= length) & (perpendicular <= 4)]
    if len(selected) < 20:
        raise ValueError("Not enough edge pixels near the selected plate line")
    vx, vy, x, y = cv2.fitLine(selected, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    fitted = np.array([vx, vy], dtype=float)
    if fitted @ direction < 0:
        fitted *= -1
    endpoints = np.array([x, y]) + np.array([[-length/2], [length/2]]) * fitted
    rays = np.linalg.solve(K, np.c_[endpoints, np.ones(2)].T).T
    normal = pose[:3, 2]
    denominator = rays @ normal
    if np.any(np.abs(denominator) < 1e-6):
        raise ValueError("Selected rays are parallel to tag plane")
    points = rays * ((normal @ pose[:3, 3]) / denominator[:, None])
    forward = points[1]-points[0]
    forward /= np.linalg.norm(forward)
    if axis == "x":
        optical_from_world = np.column_stack([forward, np.cross(normal, forward), normal])
    else:
        optical_from_world = np.column_stack([np.cross(forward, normal), forward, normal])
    world_from_tag_rotation = optical_from_world.T @ pose[:3, :3]
    return world_from_tag_rotation, {"axis": axis, "fitted_pixels": endpoints.tolist(),
                                     "edge_pixel_count": len(selected)}


def residuals(rows: list[dict], depths: np.ndarray, K: np.ndarray,
              world_from_color: np.ndarray, target: np.ndarray) -> dict:
    positions, angles, depth_errors = [], [], []
    for row in rows:
        observed = world_from_color @ row["pose"]
        positions.append((observed[:3, 3]-target[:3, 3]).tolist())
        angles.append(float(np.degrees(Rotation.from_matrix(target[:3, :3].T @ observed[:3, :3]).magnitude())))
        optical = row["pose"][:3, 3]
        uv = K @ optical
        u, v = np.round(uv[:2]/uv[2]).astype(int)
        depth = depths[row["frame"]]
        if not (2 <= u < depth.shape[1]-2 and 2 <= v < depth.shape[0]-2):
            continue
        patch = depth[v-2:v+3, u-2:u+3]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size:
            point = optical / optical[2] * np.median(valid)
            depth_errors.append(((world_from_color @ np.r_[point, 1])[:3]-target[:3, 3]).tolist())
    return {"frames": len(rows), "position_mean_m": np.mean(positions, axis=0).tolist(),
            "position_rms_m": float(np.sqrt(np.mean(np.sum(np.square(positions), axis=1)))),
            "orientation_rms_deg": float(np.sqrt(np.mean(np.square(angles)))),
            "depth_frames": len(depth_errors),
            "depth_mean_error_m": np.mean(depth_errors, axis=0).tolist() if depth_errors else None}


def update_scene(config: dict, args) -> str:
    """Only a measured old-world -> new-world transform can relocate static geometry."""
    if args.scene_transform:
        matrix = np.asarray(json.loads(args.scene_transform.read_text()), dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]):
            raise ValueError("Scene transform must be a finite homogeneous 4x4 matrix")
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1):
            raise ValueError("Scene transform rotation must be proper and orthonormal")
        for position, orientation in (("bench_center_m", "bench_quaternion_xyzw"),):
            if config.get(position) is not None:
                moved = matrix @ transform(config[position], config[orientation])
                config[position] = moved[:3, 3].tolist()
                config[orientation] = Rotation.from_matrix(moved[:3, :3]).as_quat().tolist()
        wall = config.get("camera_wall")
        if wall:
            moved = matrix @ transform(wall["center_m"], wall["quaternion_xyzw"])
            wall["center_m"] = moved[:3, 3].tolist()
            wall["quaternion_xyzw"] = Rotation.from_matrix(moved[:3, :3]).as_quat().tolist()
        if config.get("place_tcp_m") is not None:
            config["place_tcp_m"] = (matrix @ np.r_[config["place_tcp_m"], 1])[:3].tolist()
        return "Existing static scene transformed by supplied old-world-to-new-world matrix"
    if args.base_moved:
        for key in ("bench_center_m", "camera_wall", "place_tcp_m"):
            config[key] = None
        return "Base moved: bench, camera wall and placement require new measurements (set to null)"
    return "Static scene unchanged; camera mount calibration only"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=LOCAL / "openyam_bench.json")
    parser.add_argument("--mode", choices=("calibrate", "measure", "capture"), default="calibrate")
    parser.add_argument("--source", choices=("direct", "live"), default="direct")
    parser.add_argument("--replay", type=Path, help="Evidence directory from this script; no camera needed")
    parser.add_argument("--evidence", type=Path, help="New output directory; defaults below local-setup/evidence")
    parser.add_argument("--apply", action="store_true", help="Back up and replace the configuration")
    parser.add_argument("--serial")
    parser.add_argument("--tag-id", type=int)
    parser.add_argument("--tag-size", type=float, help="Detected black square edge in metres")
    parser.add_argument("--tag-in-world", type=float, nargs=3, help="Tag center relative to arm base, metres")
    parser.add_argument("--tag-rpy-deg", type=float, nargs=3, help="Physical tag orientation relative to arm base")
    parser.add_argument("--plate-axis-pixels", type=float, nargs=4, metavar=("U1", "V1", "U2", "V2"),
                        help="Fresh image endpoints directed along a physical plate +X or +Y edge")
    parser.add_argument("--plate-axis", choices=("x", "y"), default="x")
    parser.add_argument("--select-plate-axis", action="store_true", help="Select directed +X/+Y edge on fresh image")
    parser.add_argument("--base-moved", action="store_true", help="Invalidate stale scene geometry unless --scene-transform supplied")
    parser.add_argument("--scene-transform", type=Path, help="JSON 4x4 measured old-world -> new-world transform for static scene")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=6)
    parser.add_argument("--samples", type=int, default=36)
    parser.add_argument("--seconds", type=float, default=8, help="Live capture duration")
    args = parser.parse_args()
    if args.apply and args.mode != "calibrate":
        parser.error("--apply requires --mode calibrate")
    if args.samples < 12 or not 0 < args.seconds <= 60 or min(args.width, args.height, args.fps) <= 0:
        parser.error("Use >=12 samples, 0<seconds<=60, and positive stream parameters")
    if args.plate_axis_pixels and args.select_plate_axis:
        parser.error("Choose either numeric or interactive plate-axis selection")
    if (args.plate_axis_pixels or args.select_plate_axis) and args.mode != "calibrate":
        parser.error("Plate reference selection requires --mode calibrate")
    if (args.base_moved or args.scene_transform) and args.mode != "calibrate":
        parser.error("Scene updates require --mode calibrate")
    config_text = args.config.read_text()
    config = json.loads(config_text)
    serial = args.serial or config["camera_serial"]
    evidence = args.evidence or LOCAL / "evidence" / datetime.now(timezone.utc).strftime("calibration-%Y%m%dT%H%M%S-%fZ")
    evidence.mkdir(parents=True, exist_ok=False)
    (evidence / "config-before.json").write_text(config_text)
    if args.replay:
        metadata = json.loads((args.replay / "capture.json").read_text())
        with np.load(args.replay / "capture.npz", allow_pickle=False) as capture:
            colors, depths = capture["color_bgr"], capture["depth_m"]
        if metadata["serial"] != serial:
            raise ValueError("Replay serial does not match configured camera")
    else:
        colors, depths, metadata = (capture_live if args.source == "live" else capture_direct)(args, serial)
    np.savez_compressed(evidence / "capture.npz", color_bgr=colors, depth_m=depths)
    (evidence / "capture.json").write_text(json.dumps(metadata, indent=2) + "\n")
    cv2.imwrite(str(evidence / "scene.png"), colors[-1])
    if args.mode == "capture":
        print(f"Saved {len(colors)} RGB-D frames to {evidence}")
        return
    tag_id = args.tag_id if args.tag_id is not None else config.get("calibration_tag_id", 0)
    size = args.tag_size if args.tag_size is not None else config.get("calibration_tag_size_m", 0.056)
    center = args.tag_in_world if args.tag_in_world is not None else config["calibration_tag_center_from_arm_axis_m"]
    if not np.isfinite(size) or size <= 0 or not np.isfinite(center).all():
        raise ValueError("Tag size must be positive and tag center finite")
    quaternion = config.get("calibration_tag_quaternion_xyzw")
    if args.tag_rpy_deg is not None:
        quaternion = Rotation.from_euler("xyz", args.tag_rpy_deg, degrees=True).as_quat().tolist()
    if quaternion is None and not (args.plate_axis_pixels or args.select_plate_axis):
        raise ValueError("Supply --tag-rpy-deg or select a plate axis to establish physical tag orientation")
    target = transform(center, quaternion or [0, 0, 0, 1])
    K = np.asarray(metadata["K"], dtype=float)
    rows = detect_poses(colors, K, tag_id, size)
    # Every third frame is held out from the fit, preserving coverage over time.
    train, holdout = [r for i, r in enumerate(rows) if i % 3], rows[::3]
    plate = None
    if args.plate_axis_pixels or args.select_plate_axis:
        row = train[-1]
        frame = colors[row["frame"]]
        cv2.imwrite(str(evidence / "plate-reference.png"), frame)
        pixels = select_axis(frame, args.plate_axis) if args.select_plate_axis else args.plate_axis_pixels
        target[:3, :3], plate = plate_reference(row["pose"], frame, K, pixels, args.plate_axis)
    observed = mean_pose([r["pose"] for r in train])
    candidate_color = target @ np.linalg.inv(observed)
    candidate_link = candidate_color @ np.linalg.inv(np.asarray(metadata["link_from_color"]))
    previous_link = transform(config["camera_translation_m"], config["camera_quaternion_xyzw"])
    previous_color = previous_link @ np.asarray(metadata["link_from_color"])
    evaluation_color = previous_color if args.mode == "measure" else candidate_color
    validation = residuals(holdout, depths, K, evaluation_color, target)
    report = {"mode": args.mode, "tag_id": tag_id, "tag_size_m": size,
              "world_from_tag": target.tolist(),
              "candidate_world_from_camera_link": candidate_link.tolist(),
              "evaluated_world_from_camera_link": (previous_link if args.mode == "measure" else candidate_link).tolist(),
              "before": residuals(holdout, depths, K, previous_color, target),
              "validation": validation, "plate_reference": plate,
              "per_frame": [{**r, "pose": r["pose"].tolist()} for r in rows],
              "limitation": "Held-out frames validate repeatability at the tag, not absolute workspace/arm accuracy"}
    stable = validation["position_rms_m"] <= 0.005 and validation["orientation_rms_deg"] <= 2
    report["fit_stable"] = stable
    if args.mode == "calibrate":
        config.update(camera_serial=serial, camera_translation_m=candidate_link[:3, 3].tolist(),
                      camera_quaternion_xyzw=Rotation.from_matrix(candidate_link[:3, :3]).as_quat().tolist(),
                      calibration_tag_id=tag_id, calibration_tag_size_m=size,
                      calibration_tag_center_from_arm_axis_m=list(center),
                      calibration_tag_quaternion_xyzw=Rotation.from_matrix(target[:3, :3]).as_quat().tolist())
        if plate:
            config["calibration_reference_notes"] = (
                "Tag orientation derived from a directed mechanically aligned plate edge; "
                "assumes tag/plate coplanar and parallel to the robot XY plane. "
                "Valid while tag is rigidly fixed relative to arm base.")
        elif args.tag_rpy_deg is not None:
            config["calibration_reference_notes"] = "Operator-supplied physical tag-to-base orientation"
        report["scene_status"] = update_scene(config, args)
        config["workspace_calibration"] = {"evidence": str(evidence.resolve()),
                                           "scene_status": report["scene_status"],
                                           "method": "Multi-frame absolute AprilTag pose, held-out validation"}
        (evidence / "candidate-config.json").write_text(json.dumps(config, indent=2) + "\n")
    (evidence / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "per_frame"}, indent=2))
    print(f"Evidence: {evidence}")
    if args.apply:
        if not stable:
            raise RuntimeError("Unstable calibration; configuration not replaced (see report)")
        if args.config.read_text() != config_text:
            raise RuntimeError("Configuration changed during capture; refusing to overwrite it")
        temporary = args.config.with_name(args.config.name + ".calibration-tmp")
        with temporary.open("x") as stream:
            stream.write(json.dumps(config, indent=2) + "\n")
        temporary.replace(args.config)
        print("Configuration updated; restart the stack through its normal launch to load it. No arm commands sent.")


if __name__ == "__main__":
    main()
