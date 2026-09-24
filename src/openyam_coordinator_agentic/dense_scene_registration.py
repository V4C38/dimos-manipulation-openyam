"""Bench-local object-cloud tuning without modifying the DimOS checkout."""

from __future__ import annotations

from typing import Any
import time

import numpy as np
from pydantic import Field

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
    detector_image_size: int = Field(default=1280, ge=320)
    max_frame_age_s: float = Field(default=2.0, gt=0)
    max_rgb_depth_skew_s: float = Field(default=0.03, gt=0)
    support_plane_z_m: float = -0.02
    object_surface_margin_m: float = Field(default=0.003, ge=0)


class DenseObjectSceneRegistrationModule(ObjectSceneRegistrationModule):
    """Use denser, configurable object clouds while retaining DimOS detection/tracking."""

    config: DenseObjectSceneRegistrationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._object_db = LatestObservationObjectDB(
            distance_threshold=self.config.distance_threshold,
            min_detections_for_permanent=self.config.min_detections_for_permanent,
        )

    def _create_segmenter(self):
        # Upstream calls this after constructing YOLOE and before subscribing
        # to frames. Native masks undo letterboxing before DimOS reads them.
        if self._detector_backend == "yoloe" and self._detector is not None:
            self._detector.model.overrides.update(
                imgsz=self.config.detector_image_size, retina_masks=True,
            )
        return super()._create_segmenter()

    def _process_images(self, color_msg: Image, depth_msg: Image) -> list[DetObject]:
        now = time.time()
        ages = [now - color_msg.ts, now - depth_msg.ts]
        if any(not np.isfinite(age) or age < -0.1 or age > self.config.max_frame_age_s for age in ages):
            raise RuntimeError("RGB-D frames are stale or have invalid timestamps")
        if abs(color_msg.ts - depth_msg.ts) > self.config.max_rgb_depth_skew_s:
            raise RuntimeError("RGB and depth timestamps do not match")
        return super()._process_images(color_msg, depth_msg)

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
        # Remove support-surface samples before remembering grasp geometry.
        valid_objects = []
        for obj in objects:
            points = obj.pointcloud.points_f32()
            keep = np.flatnonzero(
                np.all(np.isfinite(points), axis=1)
                & (points[:, 2] > self.config.support_plane_z_m + self.config.object_surface_margin_m)
            )
            if len(keep) < 10:
                continue
            obj.pointcloud.pointcloud = obj.pointcloud.pointcloud.select_by_index(keep.tolist())
            valid_objects.append(obj)
        observed_objects = self._object_db.add_objects(valid_objects)
        # Publish only this observation. Historical IDs remain in the DB for
        # association, but are not ghost obstacles/clouds in the live scene.
        self.detections_3d.publish(
            to_detection3d_array(observed_objects, frame_id=self._target_frame, ts=color_image.ts)
        )
        self.objects.publish(observed_objects)
        aggregate = aggregate_pointclouds(observed_objects)
        aggregate.frame_id = self._target_frame
        aggregate.ts = color_image.ts
        self.pointcloud.publish(aggregate)
        return observed_objects
