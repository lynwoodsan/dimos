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

from typing import TYPE_CHECKING

import cv2
import numpy as np

from builtin_interfaces.msg import Time as ROSTime
from std_msgs.msg import Header as ROSHeader
from vision_msgs.msg import (
    BoundingBox2D as ROSBoundingBox2D,
    Detection2D as ROSDetection2D,
    Detection2DArray as ROSDetection2DArray,
    ObjectHypothesisWithPose as ROSObjectHypothesisWithPose,
    Point2D as ROSPoint2D,
    Pose2D as ROSPose2D,
)

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.person import Detection2DPerson
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from ultralytics.engine.results import Results


class ImageDetections2D(ImageDetections[Detection2D]):
    @classmethod
    def from_ros_detection2d_array(  # type: ignore[no-untyped-def]
        cls, image: Image, ros_detections: Detection2DArray, **kwargs
    ) -> ImageDetections2D:
        """Convert from ROS Detection2DArray message to ImageDetections2D object."""
        detections: list[Detection2D] = []
        for ros_det in ros_detections.detections:
            detection = Detection2DBBox.from_ros_detection2d(ros_det, image=image, **kwargs)
            if detection.is_valid():  # type: ignore[attr-defined]
                detections.append(detection)

        return cls(image=image, detections=detections)

    @classmethod
    def from_ultralytics_result(  # type: ignore[no-untyped-def]
        cls, image: Image, results: list[Results], **kwargs
    ) -> ImageDetections2D:
        """Create ImageDetections2D from ultralytics Results.

        Dispatches to appropriate Detection2D subclass based on result type:
        - If masks present: creates Detection2DSeg
        - If keypoints present: creates Detection2DPerson
        - Otherwise: creates Detection2DBBox

        Args:
            image: Source image
            results: List of ultralytics Results objects
            **kwargs: Additional arguments passed to detection constructors

        Returns:
            ImageDetections2D containing appropriate detection types
        """

        detections: list[Detection2D] = []
        for result in results:
            if result.boxes is None:
                continue

            num_detections = len(result.boxes.xyxy)
            for i in range(num_detections):
                detection: Detection2D
                if result.masks is not None:
                    # Segmentation detection with mask
                    detection = Detection2DSeg.from_ultralytics_result(result, i, image)
                elif result.keypoints is not None:
                    # Pose detection with keypoints
                    detection = Detection2DPerson.from_ultralytics_result(result, i, image)
                else:
                    # Regular bbox detection
                    detection = Detection2DBBox.from_ultralytics_result(result, i, image)
                if detection.is_valid():
                    detections.append(detection)

        return cls(image=image, detections=detections)

    def to_ros_detection2d_array(self) -> ROSDetection2DArray:
        """Convert ImageDetections2D to ROS Detection2DArray message.

        Uses the image's timestamp and frame_id for the header. Set these
        on the image using image.from_ros_header(header) before calling.

        Returns:
            Detection2DArray ROS message containing all detections
        """
        detection_array = ROSDetection2DArray()

        # Set header
        detection_array.header = ROSHeader()
        detection_array.header.frame_id = self.image.frame_id or ""
        detection_array.header.stamp = ROSTime()
        if self.image.ts:
            detection_array.header.stamp.sec = int(self.image.ts)
            detection_array.header.stamp.nanosec = int((self.image.ts % 1) * 1_000_000_000)

        # Convert each detection
        for det in self.detections:
            x1, y1, x2, y2 = det.bbox
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            size_x = x2 - x1
            size_y = y2 - y1

            ros_detection = ROSDetection2D()
            ros_detection.bbox = ROSBoundingBox2D()
            ros_detection.bbox.center = ROSPose2D()
            ros_detection.bbox.center.position = ROSPoint2D()
            ros_detection.bbox.center.position.x = center_x
            ros_detection.bbox.center.position.y = center_y
            ros_detection.bbox.center.theta = 0.0
            ros_detection.bbox.size_x = size_x
            ros_detection.bbox.size_y = size_y

            # Add class hypothesis if available
            if hasattr(det, "name") and det.name:
                hypothesis = ROSObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = det.name
                hypothesis.hypothesis.score = det.confidence if hasattr(det, "confidence") else 0.0
                ros_detection.results.append(hypothesis)

            # Set tracking ID if available
            if hasattr(det, "track_id") and det.track_id >= 0:
                ros_detection.id = str(det.track_id)

            detection_array.detections.append(ros_detection)

        return detection_array

    def overlay(self, alpha: float = 0.4) -> Image:
        """Overlay detection bboxes and masks onto the image.

        Args:
            alpha: Transparency for mask overlay (default: 0.4)

        Returns:
            Image with detection bboxes, masks, and labels drawn
        """
        img = self.image.to_opencv().copy()

        for i, det in enumerate(self.detections):
            track_id = det.track_id if hasattr(det, "track_id") else i
            name = det.name if hasattr(det, "name") else ""
            confidence = det.confidence if hasattr(det, "confidence") else 0.0

            # Generate consistent color from track_id
            np.random.seed(abs(track_id) if track_id >= 0 else i)
            color = tuple(np.random.randint(50, 255, 3).tolist())

            # Draw mask if available
            if hasattr(det, "mask") and det.mask is not None:
                mask = det.mask
                mask_indices = mask > 0
                if np.any(mask_indices):
                    # Create colored mask overlay
                    colored_mask = np.zeros_like(img)
                    colored_mask[mask_indices] = color
                    img[mask_indices] = cv2.addWeighted(
                        img[mask_indices], 1 - alpha, colored_mask[mask_indices], alpha, 0
                    )

                    # Draw contour
                    mask_uint8 = mask.astype(np.uint8) if mask.dtype == bool else mask
                    contours, _ = cv2.findContours(
                        mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )
                    cv2.drawContours(img, contours, -1, color, 2)

            # Draw bbox
            x1, y1, x2, y2 = map(int, det.bbox)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            # Draw label
            label = f"{name}" if name else f"ID:{track_id}"
            if track_id >= 0:
                label += f":{track_id}"
            if confidence > 0:
                label += f" ({confidence:.2f})"

            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            cv2.rectangle(
                img, (x1, y1 - text_h - baseline - 2), (x1 + text_w, y1), color, -1
            )
            cv2.putText(
                img,
                label,
                (x1, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        return Image.from_numpy(img, format=self.image.format, ts=self.image.ts, frame_id=self.image.frame_id)
