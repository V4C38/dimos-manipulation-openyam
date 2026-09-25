"""Configurable GraspGenX facade using a repository-owned isolated project."""

import os
from pathlib import Path

from pydantic import Field

from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig, GraspGenXModule
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.data import get_project_root
import openyam_coordinator_agentic


class GraspGenXSamplingConfig(BaseConfig):
    num_grasps: int = Field(default=400, ge=40, le=2000)
    min_raw_score: float = Field(default=0.7, ge=0, le=1)
    remove_outliers: bool = False


class ConfiguredGraspGenXConfig(GraspGenXConfig, GraspGenXSamplingConfig):
    """Upstream gripper contract plus configurable proposal sampling."""


class ConfiguredGraspGenXModule(GraspGenXModule):
    config: ConfiguredGraspGenXConfig
    project_dir = str(Path(__file__).resolve().parent / "project")
    implementation = (
        "openyam_coordinator_agentic.grasping.grasp_gen_x.runtime:ConfiguredGraspGenXRuntimeModule"
    )

    def _runtime_env(self) -> dict[str, str]:
        env = super()._runtime_env()
        package_parent = Path(openyam_coordinator_agentic.__file__).resolve().parent.parent
        # Reuse the pinned upstream implementation as a read-only dependency.
        paths = [str(package_parent), str(get_project_root() / "native/python/graspgenx")]
        if env.get("PYTHONPATH"):
            paths.append(env["PYTHONPATH"])
        env["PYTHONPATH"] = os.pathsep.join(paths)
        return env
