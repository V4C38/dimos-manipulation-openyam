"""Collision geometry in TCP coordinates for grasp and insertion filtering."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, cKDTree
import trimesh

from openyam_coordinator_agentic.collision_model import origin_matrix


class GripperGeometry:
    def __init__(self, model_path: Path) -> None:
        root = ET.parse(model_path).getroot()
        tip_joint = root.find("joint[@name='gripper_tip_joint']")
        gripper_link = root.find("link[@name='gripper']")
        if tip_joint is None or gripper_link is None:
            raise ValueError("OpenYAM gripper/TCP links are missing")
        self.gripper_from_tcp = origin_matrix(tip_joint.find("origin"))
        tcp_from_gripper = np.linalg.inv(self.gripper_from_tcp)
        self.parts: list[tuple[str, np.ndarray, np.ndarray]] = []
        self._bounds: dict[str, tuple[np.ndarray, float, np.ndarray, np.ndarray]] = {}
        for collision in gripper_link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            path = Path(mesh.get("filename"))
            if not path.is_absolute():
                path = model_path.parent / path
            vertices = np.asarray(trimesh.load(path, force="mesh").vertices)
            vertices = vertices * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            transform = tcp_from_gripper @ origin_matrix(collision.find("origin"))
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            hull = ConvexHull(vertices)
            name = collision.get("name", "")
            vertices = vertices[hull.vertices]
            center = vertices.mean(axis=0)
            self._bounds[name] = (center, float(np.linalg.norm(vertices - center, axis=1).max()),
                                  vertices.min(axis=0), vertices.max(axis=0))
            self.parts.append((name, vertices, hull.equations))
        if not self.parts:
            raise ValueError("Gripper collision geometry is missing")
        self.vertices = np.concatenate([part[1] for part in self.parts])

    def validate_capture(self, gripper: dict, grasp_frame_to_tcp: list) -> None:
        grasp_from_gripper = np.asarray(grasp_frame_to_tcp) @ np.linalg.inv(self.gripper_from_tcp)
        grasp_from_tcp = np.asarray(grasp_frame_to_tcp)
        points = self.vertices @ grasp_from_tcp[:3, :3].T + grasp_from_tcp[:3, 3]
        distal = float(points[:, 2].max())
        upper_bounds = [float(gripper[f"offset_{aperture}"][2] + gripper[f"extents_{aperture}"][2] / 2)
                        for aperture in ("open", "half_open")]
        if (abs(distal - gripper["fingertip_depth"]) > 0.001
                or any(abs(distal - upper) > 0.001 for upper in upper_bounds)):
            raise ValueError("Capture depth/TCP transform disagree with collision fingertips")
        if not np.allclose(grasp_from_gripper[:3, 3], 0, atol=1e-6):
            raise ValueError("OpenYAM grasp frame must share the gripper body origin")
        if not np.allclose(grasp_from_gripper[:3, :3], [[0, 1, 0], [1, 0, 0], [0, 0, -1]], atol=1e-6):
            raise ValueError("OpenYAM grasp closing/approach axes disagree with the gripper body")

    def _contains(self, name: str, planes: np.ndarray, points: np.ndarray, clearance: float) -> bool:
        _, _, low, high = self._bounds[name]
        points = points[np.all((points >= low - clearance) & (points <= high + clearance), axis=1)]
        # Keep the temporary point/plane matrix bounded for detailed finger hulls.
        for start in range(0, len(points), 128):
            distances = points[start:start + 128] @ planes[:, :3].T + planes[:, 3]
            if np.any(np.all(distances <= clearance, axis=1)):
                return True
        return False

    def obstruction(
        self, tcp_pose: np.ndarray, scene: np.ndarray, scene_tree: cKDTree | None,
        object_points: np.ndarray, *, support_z: float, clearance: float,
        approach_distance: float, step: float = 0.005,
    ) -> str | None:
        rotation = tcp_pose[:3, :3]
        # A half-space covers the support plane under the object even where
        # object exclusion removed its depth pixels.
        at_contact = self.vertices @ rotation.T + tcp_pose[:3, 3]
        if float(at_contact[:, 2].min()) < support_z + clearance:
            return "support_plane_collision"
        for distance in np.linspace(0, approach_distance, int(np.ceil(approach_distance / step)) + 1):
            position = tcp_pose[:3, 3] + rotation[:, 2] * distance
            for name, vertices, planes in self.parts:
                center, radius, _, _ = self._bounds[name]
                if scene_tree is not None:
                    indices = scene_tree.query_ball_point(position + rotation @ center, radius + clearance)
                    local = (scene[indices] - position) @ rotation
                    if self._contains(name, planes, local, clearance):
                        return "scene_insertion_collision"
                # Sweeps cover closed as well as open fingers and cannot be
                # used to forbid intended finger/object contact. The palm can.
                if name == "gripper_cad":
                    local = (object_points - position) @ rotation
                    if self._contains(name, planes, local, clearance):
                        return "palm_object_collision"
        return None
