"""Bench-local object-cloud tuning without modifying the DimOS checkout."""

from __future__ import annotations

from typing import Any

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.experimental.object import Object, Object as DetObject, aggregate_pointclouds, to_detection3d_array
from dimos.perception.experimental.objectDB import ObjectDB
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationConfig,
    ObjectSceneRegistrationModule,
)


class LatestObservationObjectDB(ObjectDB):
    """Keep tracked identities while using only the latest observed grasp geometry."""

    def _update_existing(self, existing: Object, obj: Object, now: float) -> bool:
        updated = super()._update_existing(existing, obj, now)
        if updated:
            # The base database accumulates world-frame clouds. A moved object
            # would otherwise leave old points in the next grasp's target.
            existing.pointcloud = obj.pointcloud
        return updated


class DenseObjectSceneRegistrationConfig(ObjectSceneRegistrationConfig):
    """Object-cloud parameters appropriate for close-range grasp generation."""

    object_voxel_downsample_m: float = 0.002
    object_mask_erode_pixels: int = 1
    object_outlier_neighbors: int = 12
    object_outlier_std_ratio: float = 0.75


class DenseObjectSceneRegistrationModule(ObjectSceneRegistrationModule):
    """Use denser, configurable object clouds while retaining DimOS detection/tracking."""

    config: DenseObjectSceneRegistrationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._object_db = LatestObservationObjectDB(
            distance_threshold=self.config.distance_threshold,
            min_detections_for_permanent=self.config.min_detections_for_permanent,
        )

    def _process_3d_detections(
        self,
        detections_2d: Any,
        color_image: Image,
        depth_image: Image,
        camera_transform: Transform | None,
    ) -> list[DetObject]:
        if self._camera_info is None:
            return []
        if self._target_frame != color_image.frame_id and camera_transform is None:
            return []

        self._latest_scene_snapshot = (depth_image, camera_transform)
        objects = Object.from_2d_to_list(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
            camera_transform=camera_transform,
            voxel_downsample=self.config.object_voxel_downsample_m,
            mask_erode_pixels=self.config.object_mask_erode_pixels,
            statistical_nb_neighbors=self.config.object_outlier_neighbors,
            statistical_std_ratio=self.config.object_outlier_std_ratio,
            max_distance=self._max_distance,
            use_aabb=self._use_aabb,
            max_obstacle_width=self._max_obstacle_width,
        )
        observed_objects = self._object_db.add_objects(objects)
        if not objects:
            return []
        all_permanent = self._object_db.get_objects()
        self.detections_3d.publish(
            to_detection3d_array(all_permanent, frame_id=self._target_frame, ts=color_image.ts)
        )
        self.objects.publish(all_permanent)
        self.pointcloud.publish(aggregate_pointclouds(all_permanent))
        return observed_objects
