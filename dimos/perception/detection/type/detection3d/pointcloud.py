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

from __future__ import annotations

from dataclasses import dataclass
import functools
from typing import TYPE_CHECKING, Any

from dimos_lcm.builtin_interfaces import Duration
from dimos_lcm.foxglove_msgs import CubePrimitive, SceneEntity, TextPrimitive
from dimos_lcm.geometry_msgs import Point, Pose, Quaternion, Vector3 as LCMVector3
import numpy as np
import open3d as o3d

from dimos.msgs.foxglove_msgs.Color import Color
import cv2
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion as DimosQuaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection3D as ROSDetection3D
from dimos_lcm.vision_msgs import ObjectHypothesis, ObjectHypothesisWithPose
from dimos.msgs.geometry_msgs import Pose, Quaternion
from dimos.msgs.std_msgs import Header
from dimos.perception.detection.type.detection2d import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.detection3d.base import Detection3D
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    PointCloudFilter,
    radius_outlier,
    raycast,
    statistical,
)
from dimos.types.timestamped import to_ros_stamp

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.perception.detection.type.detection2d import Detection2DBBox
    from dimos.perception.detection.type.detection3d.imageDetections3DPC import (
        ImageDetections3DPC,
    )


@dataclass
class Detection3DPC(Detection3D):
    pointcloud: PointCloud2

    @functools.cached_property
    def center(self) -> Vector3:
        return Vector3(*self.pointcloud.center)

    @functools.cached_property
    def pose(self) -> PoseStamped:
        """Convert detection to a PoseStamped using pointcloud center.

        Returns pose in world frame with identity rotation.
        The pointcloud is already in world frame.
        """
        return PoseStamped(
            ts=self.ts,
            frame_id=self.frame_id,
            position=self.center,
            orientation=(0.0, 0.0, 0.0, 1.0),  # Identity quaternion
        )

    def get_bounding_box(self):
        """Get axis-aligned bounding box of the detection's pointcloud."""
        return self.pointcloud.get_axis_aligned_bounding_box()

    def get_oriented_bounding_box(self):
        """Get oriented bounding box of the detection's pointcloud."""
        return self.pointcloud.get_oriented_bounding_box()

    def get_bounding_box_dimensions(self) -> tuple[float, float, float]:
        """Get dimensions (width, height, depth) of the detection's bounding box."""
        return self.pointcloud.get_bounding_box_dimensions()

    def bounding_box_intersects(self, other: Detection3DPC) -> bool:
        """Check if this detection's bounding box intersects with another's."""
        return self.pointcloud.bounding_box_intersects(other.pointcloud)

    def to_repr_dict(self) -> dict[str, Any]:
        # Calculate distance from camera
        # The pointcloud is in world frame, and transform gives camera position in world
        center_world = self.center
        # Camera position in world frame is the translation part of the transform
        camera_pos = self.transform.translation
        # Use Vector3 subtraction and magnitude
        distance = (center_world - camera_pos).magnitude()

        parent_dict = super().to_repr_dict()
        # Remove bbox key if present
        parent_dict.pop("bbox", None)

        return {
            **parent_dict,
            "dist": f"{distance:.2f}m",
            "points": str(len(self.pointcloud)),
        }

    def to_foxglove_scene_entity(self, entity_id: str | None = None) -> SceneEntity:
        """Convert detection to a Foxglove SceneEntity with cube primitive and text label.

        Args:
            entity_id: Optional custom entity ID. If None, generates one from name and hash.

        Returns:
            SceneEntity with cube bounding box and text label
        """

        # Create a cube primitive for the bounding box
        cube = CubePrimitive()

        # Get the axis-aligned bounding box
        aabb = self.get_bounding_box()

        # Set pose from axis-aligned bounding box
        cube.pose = Pose()
        cube.pose.position = Point()
        # Get center of the axis-aligned bounding box
        aabb_center = aabb.get_center()
        cube.pose.position.x = aabb_center[0]
        cube.pose.position.y = aabb_center[1]
        cube.pose.position.z = aabb_center[2]

        # For axis-aligned box, use identity quaternion (no rotation)
        cube.pose.orientation = Quaternion()
        cube.pose.orientation.x = 0
        cube.pose.orientation.y = 0
        cube.pose.orientation.z = 0
        cube.pose.orientation.w = 1

        # Set size from axis-aligned bounding box
        cube.size = LCMVector3()
        aabb_extent = aabb.get_extent()
        cube.size.x = aabb_extent[0]  # width
        cube.size.y = aabb_extent[1]  # height
        cube.size.z = aabb_extent[2]  # depth

        # Set color based on name hash
        cube.color = Color.from_string(self.name, alpha=0.2)

        # Create text label
        text = TextPrimitive()
        text.pose = Pose()
        text.pose.position = Point()
        text.pose.position.x = aabb_center[0]
        text.pose.position.y = aabb_center[1]
        text.pose.position.z = aabb_center[2] + aabb_extent[2] / 2 + 0.1  # Above the box
        text.pose.orientation = Quaternion()
        text.pose.orientation.x = 0
        text.pose.orientation.y = 0
        text.pose.orientation.z = 0
        text.pose.orientation.w = 1
        text.billboard = True
        text.font_size = 20.0
        text.scale_invariant = True
        text.color = Color()
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = self.scene_entity_label()

        # Create scene entity
        entity = SceneEntity()
        entity.timestamp = to_ros_stamp(self.ts)
        entity.frame_id = self.frame_id
        entity.id = str(self.track_id)
        entity.lifetime = Duration()
        entity.lifetime.sec = 0  # Persistent
        entity.lifetime.nanosec = 0
        entity.frame_locked = False

        # Initialize all primitive arrays
        entity.metadata_length = 0
        entity.metadata = []
        entity.arrows_length = 0
        entity.arrows = []
        entity.cubes_length = 1
        entity.cubes = [cube]
        entity.spheres_length = 0
        entity.spheres = []
        entity.cylinders_length = 0
        entity.cylinders = []
        entity.lines_length = 0
        entity.lines = []
        entity.triangles_length = 0
        entity.triangles = []
        entity.texts_length = 1
        entity.texts = [text]
        entity.models_length = 0
        entity.models = []

        return entity

    def to_ros_detection3d(self) -> ROSDetection3D:
        """Convert to ROS Detection3D message."""
        msg = ROSDetection3D()
        msg.header = Header(self.ts, self.frame_id)

        # Results
        msg.results = [
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(
                    class_id=str(self.class_id),
                    score=self.confidence,
                )
            )
        ]

        # Bounding Box
        dims = self.get_bounding_box_dimensions()
        msg.bbox.center = Pose(
            position=self.center,
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        msg.bbox.size = Vector3(dims[0], dims[1], dims[2])

        return msg

    def scene_entity_label(self) -> str:
        return f"{self.track_id}/{self.name} ({self.confidence:.0%})"

    @classmethod
    def from_2d_depth(
        cls,
        detections_2d: ImageDetections2D,
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
        depth_scale: float = 1.0,
        depth_trunc: float = 10.0,
        statistical_nb_neighbors: int = 10,
        statistical_std_ratio: float = 0.5,
    ) -> ImageDetections3DPC:
        """Create 3D pointcloud detections from 2D detections and RGBD images.

        Uses Open3D's optimized RGBD projection for efficient processing.

        Args:
            detections_2d: 2D detections with segmentation masks
            color_image: RGB color image
            depth_image: Depth image (in meters if depth_scale=1.0)
            camera_info: Camera intrinsics
            depth_scale: Scale factor for depth (1.0 for meters, 1000.0 for mm)
            depth_trunc: Maximum depth value in meters
            statistical_nb_neighbors: Neighbors for statistical outlier removal
            statistical_std_ratio: Std ratio for statistical outlier removal
        """
        color_cv = color_image.to_opencv()
        if color_cv.ndim == 3 and color_cv.shape[2] == 3:
            color_cv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2RGB)

        depth_cv = depth_image.to_opencv()
        h, w = depth_cv.shape[:2]

        # Build Open3D camera intrinsics
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
        intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

        identity_transform = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=DimosQuaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=depth_image.frame_id,
            child_frame_id=depth_image.frame_id,
            ts=depth_image.ts,
        )

        detections_3d = []

        for det in detections_2d.detections:
            # Get mask (from segmentation or bbox)
            if isinstance(det, Detection2DSeg):
                mask = det.mask
            else:
                mask = np.zeros((h, w), dtype=np.uint8)
                x1, y1, x2, y2 = map(int, det.bbox)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)
                mask[y1:y2, x1:x2] = 255

            # Apply mask to depth - set non-masked pixels to 0
            depth_masked = depth_cv.copy()
            depth_masked[mask == 0] = 0

            # Use Open3D's optimized RGBD-to-pointcloud (single C++ call)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(color_cv.astype(np.uint8)),
                o3d.geometry.Image(depth_masked.astype(np.float32)),
                depth_scale=depth_scale,
                depth_trunc=depth_trunc,
                convert_rgb_to_intensity=False,
            )
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)

            if len(pcd.points) < 4:
                continue

            # Single statistical outlier removal (efficient)
            pcd_filtered, _ = pcd.remove_statistical_outlier(
                nb_neighbors=statistical_nb_neighbors,
                std_ratio=statistical_std_ratio,
            )

            if len(pcd_filtered.points) < 4:
                continue

            # Wrap in PointCloud2
            pc = PointCloud2(
                pcd_filtered,
                frame_id=depth_image.frame_id,
                ts=depth_image.ts,
            )

            detections_3d.append(
                cls(
                    image=det.image,
                    bbox=det.bbox,
                    track_id=det.track_id,
                    class_id=det.class_id,
                    confidence=det.confidence,
                    name=det.name,
                    ts=det.ts,
                    pointcloud=pc,
                    transform=identity_transform,
                    frame_id=depth_image.frame_id,
                )
            )

        return ImageDetections3DPC(
            detections=detections_3d,
            image=color_image,
        )

    @classmethod
    def from_2d(
        cls,
        det: Detection2DBBox,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        filters: list[PointCloudFilter] | None = None,
    ) -> Detection3DPC | None:
        """Create a Detection3D from a 2D detection by projecting world pointcloud."""
        if filters is None:
            filters = [
                raycast(),
                radius_outlier(),
                statistical(),
            ]

        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
        image_width = camera_info.width
        image_height = camera_info.height

        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        world_points = world_pointcloud.as_numpy()

        points_homogeneous = np.hstack([world_points, np.ones((world_points.shape[0], 1))])
        extrinsics_matrix = world_to_optical_transform.to_matrix()
        points_camera = (extrinsics_matrix @ points_homogeneous.T).T

        valid_mask = points_camera[:, 2] > 0
        points_camera = points_camera[valid_mask]
        world_points = world_points[valid_mask]

        if len(world_points) == 0:
            return None

        points_2d_homogeneous = (camera_matrix @ points_camera[:, :3].T).T
        points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]

        in_image_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < image_width)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < image_height)
        )
        points_2d = points_2d[in_image_mask]
        world_points = world_points[in_image_mask]

        if len(world_points) == 0:
            return None

        x_min, y_min, x_max, y_max = det.bbox

        margin = 5
        in_box_mask = (
            (points_2d[:, 0] >= x_min - margin)
            & (points_2d[:, 0] <= x_max + margin)
            & (points_2d[:, 1] >= y_min - margin)
            & (points_2d[:, 1] <= y_max + margin)
        )

        detection_points = world_points[in_box_mask]

        if detection_points.shape[0] == 0:
            return None

        initial_pc = PointCloud2.from_numpy(
            detection_points,
            frame_id=world_pointcloud.frame_id,
            timestamp=world_pointcloud.ts,
        )

        detection_pc = initial_pc
        for filter_func in filters:
            result = filter_func(det, detection_pc, camera_info, world_to_optical_transform)
            if result is None:
                return None
            detection_pc = result

        if len(detection_pc.pointcloud.points) == 0:
            return None

        return cls(
            image=det.image,
            bbox=det.bbox,
            track_id=det.track_id,
            class_id=det.class_id,
            confidence=det.confidence,
            name=det.name,
            ts=det.ts,
            pointcloud=detection_pc,
            transform=world_to_optical_transform,
            frame_id=world_pointcloud.frame_id,
        )
