"""Configured workspace obstacles and fixed-camera TF publication."""

from typing import Any

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from openyam_coordinator_agentic.checked_motion import (
    CheckedManipulationConfig,
    CheckedManipulationModule,
)
from openyam_coordinator_agentic.collision_safety import (
    filter_static_bench_overlap,
    require_collision_coverage,
)
from openyam_coordinator_agentic.workspace_geometry import WorkspaceBoxConfig


class WorkspaceMountConfig(StaticTfPublisherConfig):
    mount: Transform


class WorkspaceMountModule(StaticTfPublisher):
    config: WorkspaceMountConfig

    def transforms(self) -> list[Transform]:
        return [self.config.mount]


class WorkspaceManipulationConfig(CheckedManipulationConfig):
    obstacles: tuple[WorkspaceBoxConfig, ...]


class WorkspaceManipulationModule(CheckedManipulationModule):
    """Checked motion in the configured bench and camera-wall workspace."""

    config: WorkspaceManipulationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        require_collision_coverage(self.config.model.model.load().xml)

    def _initialize_planning(self) -> None:
        super()._initialize_planning()
        assert self._world_monitor is not None
        for obstacle in self.config.obstacles:
            self._world_monitor.add_obstacle(obstacle.to_obstacle())
        filter_static_bench_overlap(self._world_monitor.world)
