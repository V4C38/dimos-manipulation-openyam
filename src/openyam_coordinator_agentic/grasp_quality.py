"""Object-agnostic geometric quality gates for generated parallel-jaw grasps."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class GraspQualityConfig:
    min_object_points: int = 150
    voxel_size_m: float = 0.003
    min_occupied_voxels: int = 24
    min_capture_points: int = 50
    min_closing_span_m: float = 0.006
    top_max_tilt_deg: float = 45.0
    side_max_tilt_deg: float = 30.0
    side_grasp_score_scale: float = 1.0
    max_candidates: int = 10


def _pose_matrix(candidate: GraspCandidate) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = candidate.pose.orientation.to_rotation_matrix()
    matrix[:3, 3] = [candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z]
    return matrix


def _inside_box(points: np.ndarray, center: np.ndarray, extents: np.ndarray) -> np.ndarray:
    return np.all(np.abs(points - center) <= extents / 2.0, axis=1)


class GraspQualityFilter:
    """Reject poorly supported proposals without using object labels."""

    def __init__(self, gripper: dict[str, Any], grasp_frame_to_tcp: list[list[float]], config: GraspQualityConfig) -> None:
        self._open_extents = np.asarray(gripper["extents_open"], dtype=float)
        self._open_offset = np.asarray(gripper["offset_open"], dtype=float)
        self._grasp_from_tcp = np.linalg.inv(np.asarray(grasp_frame_to_tcp, dtype=float))
        self.config = config

    def filter(
        self,
        candidates: GraspCandidateArray,
        object_cloud: PointCloud2,
    ) -> tuple[GraspCandidateArray, dict[str, Any]]:
        object_points = object_cloud.points_f32().astype(float, copy=False)
        report: dict[str, Any] = {
            "raw_candidates": len(candidates.candidates),
            "object_points": len(object_points),
            "rejected": {},
            "retained": [],
        }
        if len(object_points) < self.config.min_object_points:
            report["rejected"] = {"insufficient_object_points": len(candidates.candidates)}
            return GraspCandidateArray(candidates.header, []), report
        voxels = np.unique(np.floor(object_points / self.config.voxel_size_m).astype(np.int64), axis=0)
        report["occupied_voxels"] = len(voxels)
        if len(voxels) < self.config.min_occupied_voxels:
            report["rejected"] = {"insufficient_object_coverage": len(candidates.candidates)}
            return GraspCandidateArray(candidates.header, []), report

        survivors: list[tuple[float, GraspCandidate, dict[str, float]]] = []
        for candidate in candidates.candidates:
            tcp_pose = _pose_matrix(candidate)
            # OpenYAM approaches along TCP -Z: 0 degrees is top-down,
            # 90 degrees is horizontal, and >90 approaches from below.
            tilt_deg = float(np.degrees(np.arccos(np.clip(tcp_pose[2, 2], -1.0, 1.0))))
            if tilt_deg > self.config.top_max_tilt_deg and not (
                90.0 - self.config.side_max_tilt_deg <= tilt_deg <= 90.0
            ):
                continue

            grasp_pose = tcp_pose @ self._grasp_from_tcp
            rotation = grasp_pose[:3, :3]
            translation = grasp_pose[:3, 3]
            local_object = (object_points - translation) @ rotation
            capture = _inside_box(local_object, self._open_offset, self._open_extents)
            capture_points = local_object[capture]
            if len(capture_points) < self.config.min_capture_points:
                self._reject(report, "insufficient_capture_support")
                continue
            x_span = float(np.ptp(capture_points[:, 0]))
            x_center = float(abs(np.mean(capture_points[:, 0] - self._open_offset[0])))
            if x_span < self.config.min_closing_span_m or x_span > self._open_extents[0] * 0.92:
                self._reject(report, "implausible_closing_span")
                continue
            if x_center > self._open_extents[0] * 0.28:
                self._reject(report, "off_center_capture")
                continue
            support_quality = min(1.0, len(capture_points) / (self.config.min_capture_points * 3))
            span_quality = min(1.0, x_span / (self._open_extents[0] * 0.55))
            center_quality = max(0.0, 1.0 - x_center / (self._open_extents[0] * 0.28))
            quality = float(candidate.score) * (0.35 + 0.65 * np.mean(
                [support_quality, span_quality, center_quality]
            ))
            if tilt_deg > self.config.top_max_tilt_deg:
                quality *= self.config.side_grasp_score_scale
            metrics = {
                "raw_score": float(candidate.score), "quality_score": quality,
                "tilt_deg": tilt_deg,
                "capture_points": float(len(capture_points)), "closing_span_m": x_span,
                "center_offset_m": x_center,
            }
            survivors.append((quality, candidate, metrics))

        survivors.sort(key=lambda item: item[0], reverse=True)
        survivors = survivors[: self.config.max_candidates]
        report["retained"] = [metrics for _, _, metrics in survivors]
        return GraspCandidateArray(
            candidates.header,
            [GraspCandidate(candidate.pose, score=quality) for quality, candidate, _ in survivors],
        ), report

    @staticmethod
    def _reject(report: dict[str, Any], reason: str) -> None:
        rejected = report["rejected"]
        rejected[reason] = int(rejected.get(reason, 0)) + 1
