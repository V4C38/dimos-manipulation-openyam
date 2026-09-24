"""Loaded only inside the GraspGenX environment; preserve upstream pose validation."""

import numpy as np

from dimos.core.core import rpc
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from graspgenx_runtime.runtime import GraspGenXRuntimeModule

from openyam_coordinator_agentic.graspgen_config import (
    ConfiguredGraspGenXConfig,
    ConfiguredGraspGenXModule,
)

logger = setup_logger()


class ConfiguredInference:
    def __init__(self, runtime, config: ConfiguredGraspGenXConfig) -> None:
        self._sampler = runtime._sampler
        self.config = config

    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        from graspgenx.grasp_server import GraspGenXSampler

        # Keep every generated proposal until scene/orientation/contact gates.
        poses, scores = GraspGenXSampler.run_inference(
            points, self._sampler, num_grasps=self.config.num_grasps,
            grasp_threshold=self.config.min_raw_score, topk_num_grasps=-1,
            min_grasps=1, max_tries=1, remove_outliers=self.config.remove_outliers,
        )
        return poses.detach().cpu().numpy(), scores.detach().cpu().numpy()


class ConfiguredGraspGenXRuntimeModule(GraspGenXRuntimeModule, ConfiguredGraspGenXModule):
    config: ConfiguredGraspGenXConfig

    @rpc
    def start(self) -> None:
        super().start()
        if not isinstance(self._runtime, ConfiguredInference):
            self._runtime = ConfiguredInference(self._runtime, self.config)
            logger.info("OpenYAM GraspGenX configured", num_grasps=self.config.num_grasps,
                        min_raw_score=self.config.min_raw_score, remove_outliers=self.config.remove_outliers,
                        max_candidates=self.config.max_candidates, gripper=self.config.gripper.model_dump())

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        return super().propose_grasps(object_pointcloud)
