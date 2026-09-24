"""Contact support, insertion clearance, and directional ranking for OpenYAM."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from openyam_coordinator_agentic.gripper_geometry import GripperGeometry


@dataclass(frozen=True)
class GraspQualityConfig:
    min_object_points: int = 150
    voxel_size_m: float = 0.003
    min_occupied_voxels: int = 24
    min_capture_points: int = 50
    min_closing_span_m: float = 0.006
    max_tilt_deg: float = 60.0
    tilt_score_floor: float = 0.5
    min_raw_score: float = 0.7
    min_capture_fraction: float = 0.25
    min_contact_points_per_side: int = 12
    min_contact_normal_alignment: float = 0.25
    jaw_margin_m: float = 0.003
    clearance_m: float = 0.003
    diversity_distance_m: float = 0.01
    diversity_angle_deg: float = 15.0
    max_candidates: int = 100

    def __post_init__(self) -> None:
        for name in ("min_object_points", "min_occupied_voxels", "min_capture_points",
                     "min_contact_points_per_side", "max_candidates"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("voxel_size_m", "min_closing_span_m", "jaw_margin_m", "clearance_m",
                     "diversity_distance_m", "diversity_angle_deg"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 < self.max_tilt_deg < 90:
            raise ValueError("max_tilt_deg must describe an above-object cone")
        for name in ("tilt_score_floor", "min_raw_score", "min_capture_fraction", "min_contact_normal_alignment"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} must be in (0, 1]")


def _pose_matrix(candidate: GraspCandidate) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = candidate.pose.orientation.to_rotation_matrix()
    matrix[:3, 3] = candidate.pose.position.to_list()
    return matrix


def similar_grasp(a: np.ndarray, b: np.ndarray, distance: float, angle_deg: float) -> bool:
    if np.linalg.norm(a[:3, 3] - b[:3, 3]) >= distance:
        return False
    # Parallel jaws are equivalent after a half-turn around their approach axis.
    relative = a[:3, :3].T @ b[:3, :3]
    angles = [np.degrees(np.arccos(np.clip((np.trace(relative @ symmetry) - 1) / 2, -1, 1)))
              for symmetry in (np.eye(3), np.diag([-1, -1, 1]))]
    return min(angles) < angle_deg


class GraspQualityFilter:
    def __init__(self, gripper: dict[str, Any], grasp_frame_to_tcp: list[list[float]],
                 config: GraspQualityConfig, geometry: GripperGeometry) -> None:
        self._open_extents = np.asarray(gripper["extents_open"], dtype=float)
        self._open_offset = np.asarray(gripper["offset_open"], dtype=float)
        self._grasp_from_tcp = np.linalg.inv(np.asarray(grasp_frame_to_tcp, dtype=float))
        self.config = config
        self.geometry = geometry

    def filter(self, candidates: GraspCandidateArray, object_cloud: PointCloud2,
               scene_cloud: PointCloud2, *, support_z: float, approach_distance: float,
               failed_grasps: list[np.ndarray] | None = None) -> tuple[GraspCandidateArray, dict[str, Any]]:
        cfg = self.config
        points = object_cloud.points_f32().astype(float, copy=False)
        report: dict[str, Any] = {"raw_candidates": len(candidates.candidates),
                                 "object_points": len(points), "rejected": {}, "retained": []}
        def reject(reason: str) -> None:
            report["rejected"][reason] = report["rejected"].get(reason, 0) + 1
        def empty(reason: str):
            report["rejected"][reason] = len(candidates.candidates)
            return GraspCandidateArray(candidates.header, []), report
        if candidates.header.frame_id != object_cloud.frame_id or scene_cloud.frame_id != object_cloud.frame_id:
            return empty("frame_mismatch")
        if not np.all(np.isfinite(points)):
            return empty("nonfinite_object_points")
        if len(points) < cfg.min_object_points:
            return empty("insufficient_object_points")
        voxels = np.unique(np.floor(points / cfg.voxel_size_m).astype(np.int64), axis=0)
        report["occupied_voxels"] = len(voxels)
        if len(voxels) < cfg.min_occupied_voxels:
            return empty("insufficient_object_coverage")
        # PCA normals, consistently oriented away from the observed centroid.
        _, neighbors = cKDTree(points).query(points, k=min(20, len(points)))
        patches = points[neighbors]
        patches = patches - patches.mean(axis=1, keepdims=True)
        _, vectors = np.linalg.eigh(np.einsum("nki,nkj->nij", patches, patches))
        normals = vectors[:, :, 0]
        normals *= np.where(np.sum(normals * (points - points.mean(axis=0)), axis=1) < 0, -1, 1)[:, None]
        scene = scene_cloud.points_f32().astype(float, copy=False)
        scene = scene[np.all(np.isfinite(scene), axis=1)]
        if len(scene) < 100:
            return empty("insufficient_scene_geometry")
        tree = cKDTree(scene)
        survivors = []
        for candidate in candidates.candidates:
            tcp = _pose_matrix(candidate)
            if not np.isfinite(candidate.score) or not np.all(np.isfinite(tcp)):
                reject("invalid_proposal")
                continue
            if candidate.score < cfg.min_raw_score:
                reject("low_raw_score")
                continue
            tilt = float(np.degrees(np.arccos(np.clip(tcp[2, 2], -1, 1))))
            if tilt > cfg.max_tilt_deg:
                reject("approach_tilt")
                continue
            if any(similar_grasp(tcp, failed, 2 * cfg.diversity_distance_m, cfg.diversity_angle_deg)
                   for failed in failed_grasps or []):
                reject("previously_failed_grasp")
                continue
            grasp = tcp @ self._grasp_from_tcp
            local = (points - grasp[:3, 3]) @ grasp[:3, :3] - self._open_offset
            local_normals = normals @ grasp[:3, :3]
            # Evaluate fit in the Y/Z contact slab BEFORE clipping the closing axis.
            slab = np.all(np.abs(local[:, 1:]) <= self._open_extents[1:] / 2, axis=1)
            if np.count_nonzero(slab) < cfg.min_capture_points:
                reject("insufficient_capture_support")
                continue
            lo, hi = np.quantile(local[slab, 0], [0.02, 0.98])
            half_width = self._open_extents[0] / 2 - cfg.jaw_margin_m
            if lo < -half_width or hi > half_width or hi - lo < cfg.min_closing_span_m:
                reject("object_does_not_fit")
                continue
            capture = slab & (np.abs(local[:, 0]) <= half_width)
            n = int(capture.sum())
            if n < cfg.min_capture_points or n / len(points) < cfg.min_capture_fraction:
                reject("insufficient_capture_fraction")
                continue
            width = hi - lo
            left = capture & (local[:, 0] <= lo + width * 0.25) & (local_normals[:, 0] <= -cfg.min_contact_normal_alignment)
            right = capture & (local[:, 0] >= hi - width * 0.25) & (local_normals[:, 0] >= cfg.min_contact_normal_alignment)
            if min(int(left.sum()), int(right.sum())) < cfg.min_contact_points_per_side:
                reject("insufficient_opposing_contacts")
                continue
            reason = self.geometry.obstruction(tcp, scene, tree, points, support_z=support_z,
                                               clearance=cfg.clearance_m, approach_distance=approach_distance)
            if reason:
                reject(reason)
                continue
            center = abs(float((lo + hi) / 2))
            center_quality = max(0, 1 - center / half_width)
            balance = min(left.sum(), right.sum()) / max(left.sum(), right.sum())
            tilt_quality = cfg.tilt_score_floor + (1 - cfg.tilt_score_floor) * np.cos(np.radians(tilt)) ** 2
            quality = float(candidate.score) * tilt_quality * (0.5 + 0.5 * np.mean([center_quality, balance, n / len(points)]))
            metrics = {"raw_score": float(candidate.score), "quality_score": quality, "tilt_deg": tilt,
                       "capture_points": n, "capture_fraction": n / len(points), "closing_span_m": float(width),
                       "left_contact_points": int(left.sum()), "right_contact_points": int(right.sum())}
            survivors.append((quality, candidate, tcp, metrics))
        survivors.sort(key=lambda item: item[0], reverse=True)
        selected = []
        for item in survivors:
            if any(similar_grasp(item[2], other[2], cfg.diversity_distance_m, cfg.diversity_angle_deg) for other in selected):
                reject("duplicate_grasp")
                continue
            if len(selected) >= cfg.max_candidates:
                reject("candidate_limit")
                continue
            selected.append(item)
        report["retained"] = [item[3] for item in selected]
        return GraspCandidateArray(candidates.header, [GraspCandidate(item[1].pose, score=item[0]) for item in selected]), report
