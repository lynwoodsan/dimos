# Copyright 2025-2026 Dimensional Inc.
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

"""Lightweight tracking wrapper around EdgeTAMProcessor.

Obtain via ``Detection2DBBox.track()`` rather than constructing directly::

    tracker = detect("person", image).track()

    # In loop:
    while running:
        dets = tracker.update(camera.get_next())
        if dets:
            cmd_vel.publish(dets[0].servo(cam_info))

    tracker.stop()  # releases EdgeTAM resources
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


class Tracker:
    """Wrap EdgeTAMProcessor in a simple update-loop API.

    Obtain via ``detection.track()``::

        tracker = detect("person", image).track()

        while running:
            dets = tracker.update(camera.get_next())
            if dets:
                cmd_vel.publish(dets[0].servo(cam_info))

        tracker.stop()
    """

    def __init__(
        self,
        processor: EdgeTAMProcessor,
        source_detection: Detection2DBBox,
    ) -> None:
        self._processor = processor
        self._source = source_detection

    @classmethod
    def from_detection(
        cls,
        det: Detection2DBBox,
        image: Image | None = None,
    ) -> Tracker:
        """Initialize EdgeTAM from a Detection2DBBox.

        Args:
            det: Initial detection to track.
            image: Frame on which *det* was computed. Defaults to ``det.image``.

        Returns:
            Tracker ready to accept ``.update()`` calls.

        Raises:
            RuntimeError: If EdgeTAM is unavailable (no CUDA or missing model).
        """
        from dimos.models.segmentation.edge_tam import EdgeTAMProcessor

        init_image = image if image is not None else det.image
        x1, y1, x2, y2 = det.bbox
        box = np.array([x1, y1, x2, y2], dtype=np.float32)

        processor = EdgeTAMProcessor()
        processor.init_track(image=init_image, box=box, obj_id=1)
        return cls(processor, det)

    def update(self, image: Image) -> list[Detection2DBBox]:
        """Process a new frame and return tracked detections.

        Returns ``Detection2DSeg`` instances (subclass of ``Detection2DBBox``) so
        the caller can keep chaining, e.g. ``det.servo(cam_info)``.  The name
        and class_id from the original detection are propagated onto each result
        so the caller knows what was tracked.

        Args:
            image: Next video frame to process.

        Returns:
            List of ``Detection2DBBox`` for each tracked object.
            Empty list when the object was lost.
        """
        result = self._processor.process_image(image)
        if not result.detections:
            return []

        out = []
        for seg in result.detections:
            # Carry the semantic name/class from the original detection so
            # callers don't have to track it separately.
            seg.name = self._source.name
            seg.class_id = self._source.class_id
            out.append(seg)
        return out  # type: ignore[return-value]  # Detection2DSeg IS-A Detection2DBBox

    def best(self, image: Image) -> Detection2DBBox | None:
        """Process a frame and return the largest (closest) detection.

        Convenience method for the common pattern of taking the best
        detection from ``update()``::

            det = tracker.best(image)
            if det:
                cmd_vel.publish(det.servo(cam_info))

        Args:
            image: Next video frame.

        Returns:
            Detection with largest bounding-box area, or None if lost.
        """
        dets = self.update(image)
        if not dets:
            return None
        return max(dets, key=lambda d: d.bbox_2d_volume())

    def stop(self) -> None:
        """Release EdgeTAM resources (GPU memory, inference state)."""
        self._processor.stop()
