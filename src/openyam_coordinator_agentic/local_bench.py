"""Optional local calibrated-bench OpenYAM planner blueprint."""

import json
import os
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.openyam.blueprints.basic import coordinator_openyam
from dimos.robot.manipulators.openyam.config import make_openyam_model_config
from openyam_coordinator_agentic.collision_model import model_config
from openyam_coordinator_agentic.collision_safety import (
    filter_static_bench_overlap,
    require_collision_coverage,
)
from openyam_coordinator_agentic.local_bench_geometry import bench_obstacles


def _config() -> dict[str, Any]:
    path = Path(os.environ["OPENYAM_LOCAL_SETUP"])
    return json.loads(path.read_text(encoding="utf-8"))


def _model_config():
    path = os.environ.get("OPENYAM_COLLISION_MODEL")
    return model_config(Path(path)) if path else make_openyam_model_config()


class OpenYamBenchPlanner(ManipulationModule):
    """Planner containing only the configured local collision obstacles."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Validate during construction, before blueprint modules start hardware.
        require_collision_coverage(self.config.model.model.load().xml)

    def _initialize_planning(self) -> None:
        super()._initialize_planning()
        assert self._world_monitor is not None
        for obstacle in bench_obstacles(_config()):
            self._world_monitor.add_obstacle(obstacle)
        filter_static_bench_overlap(self._world_monitor.world)


openyam_bench_planner_coordinator = autoconnect(
    OpenYamBenchPlanner.blueprint(
        model=_model_config(),
        default_speed_scale=0.1,
        linear_speed_scale=0.1,
        visualization={"backend": "viser"},
    ),
    coordinator_openyam,
)
