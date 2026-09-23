"""Manufacturer CAD collision model with conservative full-stroke finger coverage.

Arm and housing use unchanged triangles. Each connected finger component is
covered by the convex hull of its closed/open vertices, a superset of its entire
linear sweep. Sweeps belong to the gripper body, not fictitious moving links.
This avoids overlapping per-link AABBs and requires no new self exclusions.
"""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.openyam.config import make_openyam_model_config

STROKE_M = 0.04695


def finger_position(opening: float) -> float:
    """Map calibrated hardware opening (0 closed, 1 open) to pinned URDF metres."""
    if not np.isfinite(opening) or not 0 <= opening <= 1:
        raise ValueError("Gripper opening must be finite and in [0, 1]")
    return -STROKE_M * opening


def origin_matrix(element: ET.Element | None) -> np.ndarray:
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
        matrix[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    return matrix


def build_collision_model(
    output: Path, provenance_path: Path, joint_zero_offsets_rad: dict[str, float] | None = None
) -> dict:
    """Generate external artifacts only; reject source changes or missing proof."""
    provenance = json.loads(provenance_path.read_text())
    if provenance.get("commit") != "5d47b358bafb30c65e397f2ece506550a0db4594" or not provenance.get(
        "urdf_matches_except_mesh_paths_and_dimos_tcp"
    ):
        raise ValueError("Manufacturer provenance must be verified first")
    description = make_openyam_model_config().model.load()
    root = ET.fromstring(description.xml)
    if root.find(".//collision") is not None:
        raise ValueError("Refusing to overwrite an authored collision model")
    output.mkdir(parents=True, exist_ok=False)
    links = {link.get("name"): link for link in root.findall("link")}
    report = {
        "source_commit": provenance["commit"],
        "arm_geometry": "unchanged manufacturer triangles",
        "finger_geometry": "per-connected-component convex full-stroke sweep",
        "stroke_m": STROKE_M,
        "new_self_exclusion_pairs": [],
        "sweeps": [],
        "joint_zero_offsets_rad": joint_zero_offsets_rad or {},
    }
    for name, offset in (joint_zero_offsets_rad or {}).items():
        joint = next((item for item in root.findall("joint") if item.get("name") == name), None)
        if joint is None or joint.get("type") != "revolute" or not np.isfinite(offset):
            raise ValueError(f"Invalid calibrated arm joint offset: {name}={offset}")
        origin = joint.find("origin")
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        rotation = Rotation.from_euler("xyz", rpy) * Rotation.from_rotvec(axis * offset)
        origin.set("rpy", " ".join(map(str, rotation.as_euler("xyz"))))
    for name, link in links.items():
        for visual in list(link.findall("visual")):
            mesh_element = visual.find("geometry/mesh")
            if mesh_element is None:
                raise ValueError("Expected manufacturer mesh visual")
            source = Path(mesh_element.get("filename"))
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            if digest != provenance["meshes"][source.name]["sha256"]:
                raise ValueError(f"Unverified mesh: {source}")
            if name not in {"tip_left", "tip_right"}:
                collision = ET.SubElement(link, "collision", name=f"{name}_cad")
                for tag in ("origin", "geometry"):
                    collision.append(deepcopy(visual.find(tag)))
                continue
            joint = next(j for j in root.findall("joint") if j.find("child").get("link") == name)
            if (
                joint.get("type") != "prismatic"
                or abs(float(joint.find("limit").get("lower")) + STROKE_M) > 1e-8
            ):
                raise ValueError("Unexpected finger mechanism")
            mount = origin_matrix(joint.find("origin"))
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            displacement = mount[:3, :3] @ axis * finger_position(1.0)
            mesh = trimesh.load(source, force="mesh")
            mesh.apply_scale(np.fromstring(mesh_element.get("scale", "1 1 1"), sep=" "))
            mesh.apply_transform(mount @ origin_matrix(visual.find("origin")))
            components = mesh.split(only_watertight=False)
            for index, component in enumerate(components):
                vertices = np.vstack((component.vertices, component.vertices + displacement))
                hull = trimesh.convex.convex_hull(vertices)
                # A hull must include both endpoints; convexity covers every intermediate point.
                planes = (
                    vertices[:, None, :] - hull.triangles_center[None, :, :]
                ) * hull.face_normals[None, :, :]
                containment_error = float(planes.sum(axis=2).max())
                if containment_error > 1e-8:
                    raise ValueError("Finger sweep failed endpoint containment")
                path = output / f"{name}_sweep_{index}.obj"
                hull.export(path)
                collision = ET.SubElement(
                    links["gripper"], "collision", name=f"{name}_sweep_{index}"
                )
                geometry = ET.SubElement(collision, "geometry")
                ET.SubElement(geometry, "mesh", filename=str(path.resolve()))
                report["sweeps"].append(
                    {
                        "name": path.name,
                        "source_vertices": len(component.vertices),
                        "hull_vertices": len(hull.vertices),
                        "containment_error_m": containment_error,
                    }
                )
            # Show the physical open shape while collision geometry covers all apertures.
            opened = mount @ origin_matrix(visual.find("origin"))
            opened[:3, 3] += displacement
            copy = deepcopy(visual)
            origin = copy.find("origin")
            origin.set("xyz", " ".join(map(str, opened[:3, 3])))
            origin.set(
                "rpy", " ".join(map(str, Rotation.from_matrix(opened[:3, :3]).as_euler("xyz")))
            )
            links["gripper"].append(copy)
            link.remove(visual)
            joint.set("type", "fixed")
            for tag in ("axis", "limit"):
                joint.remove(joint.find(tag))
    path = output / "yam_collision.urdf"
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")
    report["model_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    report["collision_geometry_count"] = len(root.findall(".//collision"))
    (output / "yam_collision.manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def model_config(path: Path, expected_joint_zero_offsets_rad: dict[str, float] | None = None):
    manifest = json.loads(path.with_name(f"{path.stem}.manifest.json").read_text())
    if hashlib.sha256(path.read_bytes()).hexdigest() != manifest["model_sha256"]:
        raise ValueError("Generated collision model no longer matches its manifest")
    if (expected_joint_zero_offsets_rad is not None
            and manifest.get("joint_zero_offsets_rad", {}) != expected_joint_zero_offsets_rad):
        raise ValueError("Collision model joint offsets do not match the workspace profile")
    config = make_openyam_model_config()
    config.model = RobotModel.from_file(path).with_default_joint_acceleration_limit(2.0)
    return config


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--provenance", type=Path, required=True)
    parser.add_argument("--joint-zero-offsets", type=Path,
                        help="JSON mapping of measured arm joint offsets in radians")
    args = parser.parse_args()
    offsets = json.loads(args.joint_zero_offsets.read_text()) if args.joint_zero_offsets else None
    if offsets is not None:
        offsets = offsets.get("joint_zero_offsets_rad", offsets)
    print(json.dumps(build_collision_model(args.output, args.provenance, offsets), indent=2))
