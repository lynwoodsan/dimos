# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Modular ROI Grasp Pipeline Module for DimOS.

This module implements the complete pipeline:
Camera → UI bbox → MobileSAM → masked ROI point cloud → cleanup → Contact-GraspNet → top-K grasps
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
from typing import Literal

import numpy as np
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.perception.grasp_generation.backends import (
    ContactGraspNetBackend,
    GraspGeneratorBackend,
    HeuristicAntipodalBackend,
)
from dimos.perception.grasp_generation.grasp_filters import (
    ParallelJawGripperModel,
    filter_grasps,
)
from dimos.perception.grasp_generation.roi_cleanup import (
    RoiCleanupConfig,
    cleanup_roi_pointcloud,
)
from dimos.perception.grasp_generation.roi_grasp_pipeline import (
    RoiGraspPipeline,
    RoiGraspPipelineConfig,
)
from dimos.perception.grasp_generation.roi_types import (
    BBoxXYXY,
    UserBBoxSelection,
)
from dimos.perception.grasp_generation.segmenters import (
    BBoxMaskSegmenter,
    BBoxOnlySegmenter,
    SamBBoxSegmenter,
    UltralyticsSamBBoxSegmenter,
)
from dimos.perception.pointcloud.utils import (
    create_point_cloud_and_extract_masks,
    load_camera_matrix_from_yaml,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class RoiGraspModuleConfig(ModuleConfig):
    """Configuration for ROI Grasp Module."""

    # Segmenter config
    segmenter_backend: Literal["sam_predictor", "ultralytics", "bbox_only"] = "sam_predictor"
    sam_checkpoint: str | None = None
    sam_model_type: str = "vit_t"
    sam_device: str = "cuda"
    use_sam: bool = True  # kept for backward compatibility; prefer segmenter_backend
    ultralytics_sam_checkpoint: str | None = None
    ultralytics_device: str = "cuda"

    # Contact-GraspNet config
    grasp_backend: Literal["contact_graspnet", "heuristic", "none"] = "contact_graspnet"
    contact_graspnet_checkpoint: str = "models_contact_graspnet"
    contact_graspnet_device: str = "cuda"
    contact_graspnet_forward_passes: int = 1
    contact_graspnet_z_range: list[float] | None = None
    heuristic_device: str = "cpu"
    heuristic_num_grasps: int = 10

    # Pipeline config
    depth_scale: float = 1.0
    depth_trunc: float = 2.0
    top_k: int = 10
    collision_check: bool = True
    collision_margin: float = 0.002

    # Cleanup config
    cleanup_voxel_size: float = 0.005
    cleanup_enable_statistical: bool = True
    cleanup_statistical_nb_neighbors: int = 30
    cleanup_statistical_std_ratio: float = 2.0
    cleanup_enable_radius: bool = True
    cleanup_radius_nb_points: int = 20
    cleanup_radius_radius: float = 0.02
    cleanup_enable_dbscan: bool = True
    cleanup_dbscan_eps: float = 0.02
    cleanup_dbscan_min_points: int = 50

    # Kinematic feasibility callback (optional)
    kinematic_feasibility: Callable[[dict], bool] | None = None


class RoiGraspModule(Module):
    """Modular ROI Grasp Pipeline Module.

    This module implements the complete end-to-end grasp generation pipeline:
    1. Receives RGB, depth, and camera_info from camera module
    2. Receives bbox selection from UI or RPC
    3. Segments object using MobileSAM/SAM
    4. Extracts ROI point cloud
    5. Cleans up point cloud (outlier removal, clustering)
    6. Generates grasps using Contact-GraspNet
    7. Filters and returns top-K grasps

    Inputs:
        - color_image: RGB image stream
        - depth_image: Depth image stream (DEPTH16 in mm)
        - camera_info: Camera intrinsics
        - bbox_selection: User bbox selection (optional, can also use RPC)

    Outputs:
        - grasps: Top-K filtered grasps
        - mask: Segmentation mask
        - roi_pointcloud: Cleaned ROI point cloud
    """

    # Inputs
    color_image: In[Image] = None
    depth_image: In[Image] = None
    camera_info: In[CameraInfo] = None
    bbox_selection: In[UserBBoxSelection] = None

    # Outputs
    grasps: Out[list[dict]] = None  # type: ignore[type-arg]
    mask: Out[np.ndarray] = None  # type: ignore[type-arg]
    roi_pointcloud: Out[dict] = None  # type: ignore[type-arg]

    default_config = RoiGraspModuleConfig

    _pipeline: RoiGraspPipeline | None = None
    _current_selection: UserBBoxSelection | None = None
    _latest_camera_info: CameraInfo | None = None
    _subscriptions: list[Disposable] = []

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._subscriptions = []

    def _build_pipeline(self) -> RoiGraspPipeline:
        """Build the ROI grasp pipeline with configured backends."""
        # Build segmenter
        segmenter: BBoxMaskSegmenter
        backend = self.config.segmenter_backend

        # Back-compat: if user sets use_sam=True but doesn't set segmenter_backend explicitly,
        # we keep the old behavior (sam_predictor if checkpoint provided, otherwise bbox_only).
        if backend == "sam_predictor":
            if self.config.use_sam and self.config.sam_checkpoint:
                segmenter = SamBBoxSegmenter(
                    checkpoint_path=self.config.sam_checkpoint,
                    model_type=self.config.sam_model_type,
                    device=self.config.sam_device,
                )
            else:
                segmenter = BBoxOnlySegmenter()
        elif backend == "ultralytics":
            if not self.config.ultralytics_sam_checkpoint:
                raise ValueError(
                    "segmenter_backend='ultralytics' requires ultralytics_sam_checkpoint to be set"
                )
            segmenter = UltralyticsSamBBoxSegmenter(
                checkpoint_path=self.config.ultralytics_sam_checkpoint,
                device=self.config.ultralytics_device,
            )
        else:
            segmenter = BBoxOnlySegmenter()

        # Build grasp backend
        grasp_backend: GraspGeneratorBackend | None = None
        if self.config.grasp_backend == "none":
            grasp_backend = None
        elif self.config.grasp_backend == "heuristic":
            grasp_backend = HeuristicAntipodalBackend(
                device=self.config.heuristic_device,
                num_grasps=self.config.heuristic_num_grasps,
            )
        else:
            try:
                grasp_backend = ContactGraspNetBackend(
                    checkpoint_dir=self.config.contact_graspnet_checkpoint,
                    device=self.config.contact_graspnet_device,
                    forward_passes=self.config.contact_graspnet_forward_passes,
                    z_range=self.config.contact_graspnet_z_range,
                )
            except Exception as e:
                logger.warning(f"Failed to initialize Contact-GraspNet backend: {e}")
                logger.warning("Falling back to heuristic backend (CPU-first)")
                grasp_backend = HeuristicAntipodalBackend(
                    device="cpu",
                    num_grasps=self.config.heuristic_num_grasps,
                )

        # Build cleanup config
        from dimos.perception.grasp_generation.roi_cleanup import RoiCleanupConfig

        cleanup_config = RoiCleanupConfig(
            voxel_size=self.config.cleanup_voxel_size,
            enable_statistical=self.config.cleanup_enable_statistical,
            statistical_nb_neighbors=self.config.cleanup_statistical_nb_neighbors,
            statistical_std_ratio=self.config.cleanup_statistical_std_ratio,
            enable_radius=self.config.cleanup_enable_radius,
            radius_nb_points=self.config.cleanup_radius_nb_points,
            radius_radius=self.config.cleanup_radius_radius,
            enable_dbscan_main_cluster=self.config.cleanup_enable_dbscan,
            dbscan_eps=self.config.cleanup_dbscan_eps,
            dbscan_min_points=self.config.cleanup_dbscan_min_points,
        )

        # Build pipeline config
        pipeline_config = RoiGraspPipelineConfig(
            depth_scale=self.config.depth_scale,
            depth_trunc=self.config.depth_trunc,
            cleanup=cleanup_config,
            top_k=self.config.top_k,
            collision_check=self.config.collision_check,
            collision_margin=self.config.collision_margin,
            gripper=ParallelJawGripperModel(),
        )

        # Get camera intrinsics from latest camera_info
        if self._latest_camera_info is None:
            raise RuntimeError("No camera_info received yet. Start camera module first.")

        camera_matrix = np.array(self._latest_camera_info.K).reshape(3, 3)

        # Build pipeline
        pipeline = RoiGraspPipeline(
            camera_intrinsics=camera_matrix,
            segmenter=segmenter,
            grasp_backend=grasp_backend,
            config=pipeline_config,
            kinematic_feasibility=self.config.kinematic_feasibility,
        )

        return pipeline

    @rpc
    def set_bbox_selection(self, x1: float, y1: float, x2: float, y2: float) -> str:
        """Set bbox selection via RPC.

        Args:
            x1, y1, x2, y2: Bounding box coordinates in pixel space

        Returns:
            Status message
        """
        selection = UserBBoxSelection(
            bbox_xyxy=(x1, y1, x2, y2),
            source="rpc",
        )
        self._current_selection = selection
        if self._pipeline is not None:
            self._pipeline.set_selection(selection)
        return f"Bbox selection set: ({x1}, {y1}, {x2}, {y2})"

    @rpc
    def clear_selection(self) -> str:
        """Clear current bbox selection."""
        self._current_selection = None
        if self._pipeline is not None:
            self._pipeline.set_selection(None)
        return "Selection cleared"

    @rpc
    def start(self) -> str:
        """Start the module and begin processing frames."""
        if self._subscriptions:
            return "already started"

        # Wait for camera_info to initialize pipeline
        if self._latest_camera_info is None:
            logger.warning("No camera_info yet. Pipeline will initialize on first frame.")

        # Subscribe to bbox selections
        if self.bbox_selection is not None:
            sub = self.bbox_selection.observable().subscribe(
                lambda sel: self._handle_bbox_selection(sel)
            )
            self._subscriptions.append(sub)

        # Subscribe to camera_info updates
        if self.camera_info is not None:
            sub = self.camera_info.observable().subscribe(
                lambda info: self._handle_camera_info(info)
            )
            self._subscriptions.append(sub)

        # Process frames when we have RGB + depth + selection
        if self.color_image is not None and self.depth_image is not None:
            # Combine latest RGB and depth frames
            rgb_stream = self.color_image.observable()
            depth_stream = self.depth_image.observable()

            # Combine streams and process
            combined = rx.combine_latest(rgb_stream, depth_stream).pipe(
                ops.filter(lambda _: self._current_selection is not None),
                ops.map(lambda frames: self._process_frame(frames[0], frames[1])),
            )

            sub = combined.subscribe(lambda _: None)
            self._subscriptions.append(sub)

        return "started"

    def _handle_camera_info(self, info: CameraInfo) -> None:
        """Handle camera_info updates."""
        self._latest_camera_info = info
        # Rebuild pipeline if needed (camera_info changed)
        if self._pipeline is None and self._latest_camera_info is not None:
            try:
                self._pipeline = self._build_pipeline()
                logger.info("ROI Grasp Pipeline initialized")
            except Exception as e:
                logger.error(f"Failed to initialize pipeline: {e}")

    def _handle_bbox_selection(self, selection: UserBBoxSelection) -> None:
        """Handle bbox selection updates."""
        self._current_selection = selection
        if self._pipeline is not None:
            self._pipeline.set_selection(selection)

    def _process_frame(self, rgb: Image, depth: Image) -> None:
        """Process a single RGB-D frame."""
        if self._pipeline is None:
            # Try to initialize if we have camera_info now
            if self._latest_camera_info is not None:
                try:
                    self._pipeline = self._build_pipeline()
                except Exception as e:
                    logger.error(f"Failed to initialize pipeline: {e}")
                    return
            else:
                return

        if self._current_selection is None:
            return

        try:
            # Convert depth from mm to meters
            depth_mm = depth.data
            depth_m = depth_mm.astype(np.float32) / 1000.0  # mm to meters

            # Process frame
            result = self._pipeline.process_frame(
                rgb=rgb.data,
                depth_m=depth_m,
                selection=self._current_selection,
            )

            # Publish outputs
            if self.grasps is not None:
                self.grasps.publish(result["grasps_topk"])

            if self.mask is not None:
                self.mask.publish(result["mask"])

            if self.roi_pointcloud is not None:
                import open3d as o3d  # type: ignore[import-untyped]

                roi_pcd = result["roi_pcd_clean"]
                points = np.asarray(roi_pcd.points) if roi_pcd.has_points() else np.array([])
                colors = (
                    np.asarray(roi_pcd.colors) if roi_pcd.has_colors() else None
                )

                self.roi_pointcloud.publish(
                    {
                        "points": points.tolist() if len(points) > 0 else [],
                        "colors": colors.tolist() if colors is not None else None,
                    }
                )

        except Exception as e:
            logger.error(f"Error processing frame: {e}", exc_info=True)

    @rpc
    def stop(self) -> None:
        """Stop the module."""
        for sub in self._subscriptions:
            sub.dispose()
        self._subscriptions = []
        self._pipeline = None
        super().stop()


roi_grasp_module = RoiGraspModule.blueprint

__all__ = ["RoiGraspModule", "RoiGraspModuleConfig", "roi_grasp_module"]

