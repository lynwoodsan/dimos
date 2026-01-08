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

from types import TracebackType
from typing import Any

import cv2
from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import pyrealsense2 as rs  # type: ignore[import-not-found]
from reactivex import interval

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3

# Import LCM message types
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.std_msgs import Header
from dimos.protocol.tf import TF
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RealSenseCamera:
    """Intel RealSense D415 camera capture class."""

    def __init__(  # type: ignore[no-untyped-def]
        self,
        device_id: int | str | None = None,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        enable_depth: bool = True,
        enable_color: bool = True,
        **kwargs,
    ) -> None:
        """
        Initialize RealSense Camera.

        Args:
            device_id: Camera device ID (serial number, index, or None for first device)
            width: Color image width (default: 1280)
            height: Color image height (default: 720)
            fps: Camera frame rate (default: 30)
            enable_depth: Enable depth stream (default: True)
            enable_color: Enable color stream (default: True)
        """
        try:
            import pyrealsense2 as rs  # type: ignore[import-not-found]
        except ImportError:
            raise ImportError("RealSense SDK not installed. Please install pyrealsense2 package.")

        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth
        self.enable_color = enable_color

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure streams
        if self.enable_color:
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        if self.enable_depth:
            self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        # Select device if specified
        if device_id is not None:
            ctx = rs.context()
            devices = ctx.query_devices()
            if isinstance(device_id, str):
                # Device ID is serial number
                for dev in devices:
                    if dev.get_info(rs.camera_info.serial_number) == device_id:
                        self.config.enable_device(device_id)
                        break
                else:
                    logger.warning(f"Device with serial {device_id} not found, using first available")
            elif isinstance(device_id, int):
                # Device ID is index
                if device_id < len(devices):
                    serial = devices[device_id].get_info(rs.camera_info.serial_number)
                    self.config.enable_device(serial)
                else:
                    logger.warning(f"Device index {device_id} not found, using first available")

        # Align depth to color
        self.align_to_color = rs.align(rs.stream.color)
        

        # Intrinsics and extrinsics (set after pipeline start)
        self.color_intrinsics: rs.intrinsics | None = None
        self.depth_intrinsics: rs.intrinsics | None = None
        self.depth_to_color_extrinsics: rs.extrinsics | None = None

        self.is_opened = False
        self.profile: rs.pipeline_profile | None = None

    def open(self) -> bool:
        """Open the RealSense camera and start streaming."""
        try:
            # Start pipeline
            self.profile = self.pipeline.start(self.config)

            # Get intrinsics
            if self.enable_color:
                color_stream = self.profile.get_stream(rs.stream.color)
                self.color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

            if self.enable_depth:
                depth_stream = self.profile.get_stream(rs.stream.depth)
                self.depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

                # Get depth to color extrinsics
                depth_profile = depth_stream.as_video_stream_profile()
                color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.depth_to_color_extrinsics = depth_profile.get_extrinsics_to(color_profile)

            self.is_opened = True

            # Get device info
            device = self.profile.get_device()
            logger.info(f"RealSense Camera opened successfully")
            logger.info(f"Device Name: {device.get_info(rs.camera_info.name)}")
            logger.info(f"Serial Number: {device.get_info(rs.camera_info.serial_number)}")
            logger.info(f"Firmware: {device.get_info(rs.camera_info.firmware_version)}")

            return True

        except Exception as e:
            logger.error(f"Error opening RealSense camera: {e}")
            return False

    def capture_frame(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:  # type: ignore[type-arg]
        """
        Capture a frame from RealSense camera.

        Returns:
            Tuple of (color_image, depth_image, aligned_depth) as numpy arrays
            Returns (None, None, None) if capture fails
        """
        if not self.is_opened:
            logger.error("RealSense camera not opened")
            return None, None, None

        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()

            # Align depth to color
            aligned_frames = self.align_to_color.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                logger.error(f"Depth or color frame not acquired and aligned")
                return None, None, None
                
            logger.info(f"Depth and color frames acquired and aligned")

            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())

            return color_image, depth_image, depth_image

        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None, None, None

    def get_camera_info(self) -> dict[str, Any]:
        """Get RealSense camera information and calibration parameters."""
        if not self.is_opened or not self.color_intrinsics:
            return {}

        try:
            return {
                "model": "Intel RealSense D415",
                "serial_number": self.profile.get_device().get_info(rs.camera_info.serial_number) if self.profile else "unknown",
                "firmware": self.profile.get_device().get_info(rs.camera_info.firmware_version) if self.profile else "unknown",
                "resolution": {
                    "width": self.color_intrinsics.width,
                    "height": self.color_intrinsics.height,
                },
                "fps": self.fps,
                "color_cam": {
                    "fx": self.color_intrinsics.fx,
                    "fy": self.color_intrinsics.fy,
                    "cx": self.color_intrinsics.ppx,
                    "cy": self.color_intrinsics.ppy,
                    "k1": self.color_intrinsics.coeffs[0] if len(self.color_intrinsics.coeffs) > 0 else 0.0,
                    "k2": self.color_intrinsics.coeffs[1] if len(self.color_intrinsics.coeffs) > 1 else 0.0,
                    "p1": self.color_intrinsics.coeffs[2] if len(self.color_intrinsics.coeffs) > 2 else 0.0,
                    "p2": self.color_intrinsics.coeffs[3] if len(self.color_intrinsics.coeffs) > 3 else 0.0,
                    "k3": self.color_intrinsics.coeffs[4] if len(self.color_intrinsics.coeffs) > 4 else 0.0,
                },
                "depth_cam": {
                    "fx": self.depth_intrinsics.fx if self.depth_intrinsics else 0.0,
                    "fy": self.depth_intrinsics.fy if self.depth_intrinsics else 0.0,
                    "cx": self.depth_intrinsics.ppx if self.depth_intrinsics else 0.0,
                    "cy": self.depth_intrinsics.ppy if self.depth_intrinsics else 0.0,
                },
            }
        except Exception as e:
            logger.error(f"Error getting camera info: {e}")
            return {}

    def close(self) -> None:
        """Close the RealSense camera."""
        if self.is_opened:
            self.pipeline.stop()
            self.is_opened = False
            logger.info("RealSense camera closed")

    def __enter__(self):  # type: ignore[no-untyped-def]
        """Context manager entry."""
        if not self.open():
            raise RuntimeError("Failed to open RealSense camera")
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """Context manager exit."""
        self.close()


class RealSenseModule(Module):
    """
    Dask module for RealSense camera that publishes sensor data via LCM.

    Publishes:
        - /realsense/color_image: RGB camera images
        - /realsense/depth_image: Depth images
        - /realsense/camera_info: Camera calibration information
    """

    # Define LCM outputs
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        device_id: int | str | None = None,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        enable_depth: bool = True,
        enable_color: bool = True,
        publish_rate: float = 30.0,
        frame_id: str = "realsense_camera",
        **kwargs,
    ) -> None:
        """
        Initialize RealSense Module.

        Args:
            device_id: Camera device ID (serial number, index, or None for first device)
            width: Color image width (default: 1280)
            height: Color image height (default: 720)
            fps: Camera frame rate
            enable_depth: Enable depth stream
            enable_color: Enable color stream
            publish_rate: Rate to publish messages (Hz)
            frame_id: TF frame ID for messages
        """
        super().__init__(**kwargs)

        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth
        self.enable_color = enable_color
        self.publish_rate = publish_rate
        self.frame_id = frame_id

        # Internal state
        self.realsense_camera = None
        self._running = False
        self._subscription = None
        self._sequence = 0

        # Initialize TF publisher
        self.tf = TF()

        logger.info(f"RealSenseModule initialized for device {device_id}")

    @rpc
    def start(self) -> None:
        """Start the RealSense module and begin publishing data."""
        if self._running:
            logger.warning("RealSense module already running")
            return

        super().start()

        try:
            # Initialize RealSense camera
            self.realsense_camera = RealSenseCamera(
                device_id=self.device_id,
                width=self.width,
                height=self.height,
                fps=self.fps,
                enable_depth=self.enable_depth,
                enable_color=self.enable_color,
            )

            # Open camera
            if not self.realsense_camera.open():
                logger.error("Failed to open RealSense camera")
                return

            # Publish camera info once at startup
            self._publish_camera_info()

            # Start periodic frame capture and publishing
            self._running = True
            publish_interval = 1.0 / self.publish_rate

            self._subscription = interval(publish_interval).subscribe(
                lambda _: self._capture_and_publish()
            )

            logger.info(f"RealSense module started, publishing at {self.publish_rate} Hz")

        except Exception as e:
            logger.error(f"Error starting RealSense module: {e}")
            self._running = False

    @rpc
    def stop(self) -> None:
        """Stop the RealSense module."""
        if not self._running:
            return

        self._running = False

        # Stop subscription
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        # Close camera
        if self.realsense_camera:
            self.realsense_camera.close()
            self.realsense_camera = None

        super().stop()

    def _capture_and_publish(self) -> None:
        """Capture frame and publish all data."""
        if not self._running or not self.realsense_camera:
            return

        try:
            # Capture frame
            color_img, depth_img, _ = self.realsense_camera.capture_frame()

            if color_img is None or depth_img is None:
                return

            # Create header
            header = Header(self.frame_id)
            self._sequence += 1

            # Publish color image
            self._publish_color_image(color_img, header)

            # Publish depth image
            self._publish_depth_image(depth_img, header)

            # Publish camera info periodically
            self._publish_camera_info()

        except Exception as e:
            logger.error(f"Error in capture and publish: {e}")

    def _publish_color_image(self, image: np.ndarray, header: Header) -> None:  # type: ignore[type-arg]
        """Publish color image as LCM message."""
        try:
            # Convert BGR to RGB if needed
            if len(image.shape) == 3 and image.shape[2] == 3:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                image_rgb = image

            # Create LCM Image message
            msg = Image(
                data=image_rgb,
                format=ImageFormat.RGB,
                frame_id=header.frame_id,
                ts=header.ts,
            )

            self.color_image.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing color image: {e}")

    def _publish_depth_image(self, depth: np.ndarray, header: Header) -> None:  # type: ignore[type-arg]
        """Publish depth image as LCM message."""
        try:
            # Depth is uint16 in millimeters, convert to float32 meters
            depth_meters = depth.astype(np.float32) / 1000.0

            msg = Image(
                data=depth_meters,
                format=ImageFormat.DEPTH,
                frame_id=header.frame_id,
                ts=header.ts,
            )
            self.depth_image.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing depth image: {e}")

    def _publish_camera_info(self) -> None:
        """Publish camera calibration information."""
        try:
            info = self.realsense_camera.get_camera_info()
            if not info:
                return

            # Get calibration parameters
            color_cam = info.get("color_cam", {})
            resolution = info.get("resolution", {})

            # Create CameraInfo message
            header = Header(self.frame_id)

            # Create camera matrix K (3x3)
            K = [
                color_cam.get("fx", 0),
                0,
                color_cam.get("cx", 0),
                0,
                color_cam.get("fy", 0),
                color_cam.get("cy", 0),
                0,
                0,
                1,
            ]

            # Distortion coefficients
            D = [
                color_cam.get("k1", 0),
                color_cam.get("k2", 0),
                color_cam.get("p1", 0),
                color_cam.get("p2", 0),
                color_cam.get("k3", 0),
            ]

            # Identity rotation matrix
            R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

            # Projection matrix P (3x4)
            P = [
                color_cam.get("fx", 0),
                0,
                color_cam.get("cx", 0),
                0,
                0,
                color_cam.get("fy", 0),
                color_cam.get("cy", 0),
                0,
                0,
                0,
                1,
                0,
            ]

            msg = CameraInfo(
                D_length=len(D),
                header=header,
                height=resolution.get("height", 0),
                width=resolution.get("width", 0),
                distortion_model="plumb_bob",
                D=D,
                K=K,
                R=R,
                P=P,
                binning_x=0,
                binning_y=0,
            )

            self.camera_info.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing camera info: {e}")

    @rpc
    def get_camera_info(self) -> dict[str, Any]:
        """Get camera information and calibration parameters."""
        if self.realsense_camera:
            return self.realsense_camera.get_camera_info()
        return {}