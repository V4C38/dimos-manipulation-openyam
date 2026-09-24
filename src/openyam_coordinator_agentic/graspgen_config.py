"""Configurable GraspGenX facade using a repository-owned isolated project."""

from pathlib import Path
import os

from pydantic import Field

from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig, GraspGenXModule
from dimos.utils.data import get_project_root


class ConfiguredGraspGenXConfig(GraspGenXConfig):
    num_grasps: int = Field(default=400, ge=40, le=2000)
    min_raw_score: float = Field(default=0.7, ge=0, le=1)
    remove_outliers: bool = False


class ConfiguredGraspGenXModule(GraspGenXModule):
    config: ConfiguredGraspGenXConfig
    project_dir = str(Path(__file__).resolve().parents[2] / "runtime" / "graspgenx")
    implementation = "openyam_coordinator_agentic.graspgen_runtime:ConfiguredGraspGenXRuntimeModule"

    def _runtime_env(self) -> dict[str, str]:
        env = super()._runtime_env()
        root = Path(__file__).resolve().parents[2]
        # Reuse the pinned upstream implementation as a read-only dependency.
        paths = [str(root / "src"), str(get_project_root() / "native/python/graspgenx")]
        if env.get("PYTHONPATH"):
            paths.append(env["PYTHONPATH"])
        env["PYTHONPATH"] = os.pathsep.join(paths)
        env["UV_PROJECT_ENVIRONMENT"] = str(root / "workspace-config/temp/graspgenx-venv")
        return env
