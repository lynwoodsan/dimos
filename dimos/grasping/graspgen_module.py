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

"""
GraspGen Module: Bridge to Docker-based GraspGen Service

This module provides a native Dimos interface to the GraspGen grasp generation
system running in an isolated Docker container. It manages the container lifecycle
and translates between Dimos LCM messages and the HTTP API of the service.

Architecture:
    - Subscribes to PointCloud2 messages via LCM
    - Converts point clouds to the service format (Base64)
    - Calls the Docker service via HTTP POST
    - Publishes PoseArray results via LCM

The Docker container is automatically started on module initialization and
stopped on module shutdown, ensuring proper resource management.
"""

from __future__ import annotations

import base64
import subprocess
import time
from typing import Any

import numpy as np
import requests
from dataclasses import dataclass

from dimos.core.module import Module, rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.utils.logging_config import setup_logger
from dimos.grasping.gripper_adapter import GripperAdapter, get_recommended_adapter

logger = setup_logger()


@dataclass
class GraspGenConfig:
    """Configuration for GraspGen module.

    Args:
        docker_image: Name of the Docker image to use
        container_name: Name for the running container
        service_port: Port to expose the service on
        service_url: URL to connect to the service
        startup_timeout: Seconds to wait for service startup
        gripper_type: Gripper type (robotiq_2f_140, franka_panda, ufactory_xarm)
        num_grasps: Number of grasp candidates to generate
        topk_num_grasps: Number of top grasps to return
        grasp_threshold: Quality threshold for filtering grasps (-1 = auto)
        filter_collisions: Whether to filter collision-prone grasps
    """

    docker_image: str = "dimos-graspgen"
    container_name: str = "dimos_graspgen_service"
    service_port: int = 8094
    service_url: str = "http://localhost:8094"
    startup_timeout: int = 60  # seconds
    gripper_type: str = "robotiq_2f_140"  # robotiq_2f_140, franka_panda, ufactory_xarm
    num_grasps: int = 400
    topk_num_grasps: int = 100
    grasp_threshold: float = -1.0
    filter_collisions: bool = False


class GraspGenModule(Module):
    """
    GraspGen Bridge Module
    
    Manages a Docker container running the GraspGen service and provides
    standard Dimos LCM interfaces for grasp generation.
    
    Inputs:
        pointcloud (PointCloud2): Object point cloud for grasp generation
    
    Outputs:
        grasps (PoseArray): Generated grasp poses
    
    RPC Methods:
        start(): Start the Docker container
        stop(): Stop the Docker container and free GPU memory
        get_status(): Get container status
    """
    
    # LCM Inputs/Outputs
    pointcloud: In[PointCloud2]
    grasps: Out[PoseArray]
    
    def __init__(self, config: GraspGenConfig | None = None, **kwargs: Any) -> None:
        """Initialize the GraspGen module.
        
        Args:
            config: Configuration for the module
            **kwargs: Additional arguments passed to Module
        """
        super().__init__(**kwargs)
        self.config = config or GraspGenConfig()
        self._container_running = False
        
        # Subscribe to point cloud input
        self.pointcloud.subscribe(self._on_pointcloud)
        
        logger.info("GraspGenModule initialized", config=self.config)
    
    @rpc
    def start(self) -> None:
        """Start the GraspGen Docker container."""
        if self._container_running:
            logger.warning("Container already running")
            return
        
        logger.info("Starting GraspGen Docker container", 
                   image=self.config.docker_image,
                   port=self.config.service_port)
        
        # Check if container already exists (from previous run)
        try:
            result = subprocess.run(
                ["docker", "inspect", self.config.container_name],
                capture_output=True,
                text=True,
                check=False
            )
            
            if result.returncode == 0:
                logger.info("Removing existing container")
                subprocess.run(
                    ["docker", "rm", "-f", self.config.container_name],
                    check=True
                )
        except subprocess.CalledProcessError as e:
            logger.error(f"Error checking existing container: {e}")
        
        # Start new container
        try:
            subprocess.run([
                "docker", "run",
                "--gpus", "all",
                "-d",
                "--rm",
                "-p", f"{self.config.service_port}:8094",
                "-e", f"DEFAULT_GRIPPER={self.config.gripper_type}",
                "--name", self.config.container_name,
                self.config.docker_image
            ], check=True)
            
            logger.info("Docker container started, waiting for readiness")
            
            # Wait for health check
            self._wait_for_service()
            self._container_running = True
            
            logger.info("GraspGen service ready")
            
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to start Docker container: {e}")
            raise RuntimeError(f"Failed to start GraspGen container: {e}")
    
    @rpc
    def stop(self) -> None:
        """Stop the GraspGen Docker container."""
        if not self._container_running:
            logger.warning("Container not running")
            return
        
        logger.info("Stopping GraspGen Docker container")
        
        try:
            subprocess.run([
                "docker", "stop", self.config.container_name
            ], check=True, timeout=10)
            
            self._container_running = False
            logger.info("GraspGen container stopped")
            
        except subprocess.CalledProcessError as e:
            logger.error(f"Error stopping container: {e}")
        except subprocess.TimeoutExpired:
            logger.warning("Container stop timeout, forcing removal")
            subprocess.run(["docker", "rm", "-f", self.config.container_name], check=False)
            self._container_running = False
    
    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get the status of the GraspGen service.
        
        Returns:
            Status dictionary with container and service information
        """
        status = {
            "container_running": self._container_running,
            "service_url": self.config.service_url,
            "service_healthy": False,
        }
        
        if self._container_running:
            try:
                response = requests.get(
                    f"{self.config.service_url}/health",
                    timeout=2
                )
                status["service_healthy"] = response.status_code == 200
                status["health_data"] = response.json()
            except Exception as e:
                logger.warning(f"Health check failed: {e}")
        
        return status
    
    def _wait_for_service(self) -> None:
        """Wait for the service to become ready."""
        start_time = time.time()
        
        while time.time() - start_time < self.config.startup_timeout:
            try:
                response = requests.get(
                    f"{self.config.service_url}/health",
                    timeout=2
                )
                if response.status_code == 200:
                    return
            except requests.exceptions.RequestException:
                pass
            
            time.sleep(1)
        
        raise TimeoutError(
            f"GraspGen service failed to start within {self.config.startup_timeout}s"
        )
    
    def _on_pointcloud(self, msg: PointCloud2) -> None:
        """Callback for incoming point cloud messages.
        
        Data Flow:
            1. Receive PointCloud2 (binary XYZ data)
            2. Validate and convert to numpy array (N, 3) float32
            3. Base64 encode for HTTP transport
            4. POST to Docker service
            5. Parse JSON response
            6. Convert to PoseArray
            7. Publish via LCM
        
        Input Format (PointCloud2):
            - Binary data: width × height × point_step bytes
            - point_step: 12 bytes (3 floats × 4 bytes)
            - Data layout: [X1,Y1,Z1, X2,Y2,Z2, ..., XN,YN,ZN]
            - Units: METERS (not millimeters)
            - Minimum: 10 points, Recommended: 100+ points
        
        Args:
            msg: Input point cloud message
        """
        if not self._container_running:
            logger.warning("Received point cloud but container not running")
            return
        
        try:
            # Convert PointCloud2 to numpy array
            points = self._pointcloud2_to_numpy(msg)
            
            if len(points) < 10:
                logger.warning("Point cloud too small for grasp generation",
                             num_points=len(points))
                return
            
            logger.info("Generating grasps", num_points=len(points))
            
            # Encode for HTTP POST
            points_b64 = base64.b64encode(points.astype(np.float32).tobytes()).decode()
            
            # Call service
            response = requests.post(
                f"{self.config.service_url}/generate",
                json={
                    "point_cloud_b64": points_b64,
                    "gripper_type": self.config.gripper_type,
                    "num_grasps": self.config.num_grasps,
                    "topk_num_grasps": self.config.topk_num_grasps,
                    "grasp_threshold": self.config.grasp_threshold,
                    "filter_collisions": self.config.filter_collisions,
                },
                timeout=30
            )
            
            if response.status_code != 200:
                logger.error("Grasp generation failed", 
                           status=response.status_code,
                           detail=response.text)
                return
            
            # Parse response
            data = response.json()
            grasps_data = data["grasps"]
            
            logger.info("Grasps generated", count=len(grasps_data))
            
            # Convert to PoseArray
            pose_array = self._create_pose_array(grasps_data, msg.header.frame_id)
            
            # Publish
            self.grasps.emit(pose_array)
            
        except Exception as e:
            logger.error(f"Error processing point cloud: {e}", exc_info=True)
    
    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to Nx3 numpy array.
        
        PointCloud2 Binary Format:
            The 'data' field is a packed byte array:
            - Total size: width × height × point_step bytes
            - Each point: point_step bytes (typically 12 for XYZ float32)
            - Layout: [X1,Y1,Z1, X2,Y2,Z2, ..., XN,YN,ZN]
        
        Assumptions:
            - Points are stored as float32 (4 bytes per coordinate)
            - First 3 values per point are XYZ coordinates
            - Additional fields (RGB, intensity, etc.) are ignored
            - Coordinates are in METERS
        
        Args:
            msg: PointCloud2 message with binary point data
            
        Returns:
            Nx3 numpy array of XYZ points in meters
            
        Raises:
            ValueError: If point cloud cannot be parsed or has < 10 points
        """
        # Parse point cloud data
        # PointCloud2 format: binary data with fields x, y, z, ...
        # For simplicity, we assume XYZ float32 format
        
        # Convert binary blob to float32 array
        data = np.frombuffer(msg.data, dtype=np.float32)
        
        # Compute number of points and floats per point
        num_points = msg.width * msg.height
        point_step = msg.point_step // 4  # bytes to float32 count
        
        # Reshape to (N, point_step)
        # Example: 1000 points × 4 floats = (1000, 4) for XYZI data
        data = data.reshape(num_points, point_step)
        
        # Extract XYZ (first 3 columns), ignore additional fields
        # Result shape: (N, 3) where each row is [X, Y, Z] in meters
        points = data[:, :3]
        
        # Validation: Check for NaN/Inf values
        if not np.isfinite(points).all():
            raise ValueError("Point cloud contains NaN or Inf values")
        
        # Validation: Check minimum point count
        if len(points) < 10:
            raise ValueError(f"Too few points: {len(points)} < 10 minimum")
        
        # Warn if point count is low
        if len(points) < 100:
            logger.warning(
                f"Low point count: {len(points)} points (recommend 100+)"
            )
        
        return points
    
    def _create_pose_array(self, grasps_data: list[dict], frame_id: str) -> PoseArray:
        """Create a PoseArray from grasp transforms.
        
        Input Format (from Docker service):
            grasps_data = [
                {
                    "transform": [16 floats],  # 4x4 SE(3) matrix, row-major
                    "score": 1.23,              # Quality score (higher = better)
                    "collision_free": True      # Optional flag
                },
                ...
            ]
        
        Transform Matrix Structure:
            [[R11, R12, R13, tx],
             [R21, R22, R23, ty],
             [R31, R32, R33, tz],
             [  0,   0,   0,  1]]
            Where:
                - R = 3x3 rotation matrix (SO(3))
                - t = [tx, ty, tz] translation vector in meters
        
        Output Format (PoseArray):
            - position: Vector3(x=tx, y=ty, z=tz) in meters
            - orientation: Quaternion(x, y, z, w) normalized
        
        Args:
            grasps_data: List of grasp dictionaries from Docker service
            frame_id: Reference frame ID (e.g., "world", "camera_link")
            
        Returns:
            PoseArray with grasps sorted by score (best first)
        """
        from dimos.msgs.std_msgs.Header import Header
        
        poses = []
        
        for grasp in grasps_data:
            # Transform is a 4x4 matrix flattened to 16 floats (row-major)
            # Input: [R11,R12,R13,tx, R21,R22,R23,ty, R31,R32,R33,tz, 0,0,0,1]
            # Reshape to: [[R R R t], [R R R t], [R R R t], [0 0 0 1]]
            transform_flat = grasp["transform"]
            transform = np.array(transform_flat).reshape(4, 4)
            
            # Extract position from translation vector (last column)
            # Units: meters
            position = Vector3(
                x=float(transform[0, 3]),  # tx
                y=float(transform[1, 3]),  # ty
                z=float(transform[2, 3])   # tz
            )
            
            # Extract 3x3 rotation matrix (upper-left block)
            rotation_matrix = transform[:3, :3]
            
            # Convert rotation matrix to quaternion
            # Returns (x, y, z, w) normalized to unit length
            quat = self._rotation_matrix_to_quaternion(rotation_matrix)
            orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            
            # Validation: Ensure quaternion is normalized
            norm = np.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)
            if abs(norm - 1.0) > 0.01:
                logger.warning(f"Quaternion not normalized: {norm:.4f}")
            
            poses.append(Pose(position=position, orientation=orientation))
        
        header = Header(frame_id=frame_id)
        return PoseArray(header=header, poses=poses)
    
    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
        """Convert 3x3 rotation matrix to quaternion (x, y, z, w).
        
        Algorithm:
            Uses Shepperd's method for numerical stability.
            Chooses the quaternion component with largest magnitude to avoid
            division by small numbers.
        
        Input:
            R: 3x3 orthogonal rotation matrix (SO(3))
               [[R11, R12, R13],
                [R21, R22, R23],
                [R31, R32, R33]]
        
        Output:
            Quaternion (x, y, z, w) satisfying:
                - ||q|| = 1 (unit quaternion)
                - R = quaternion_to_rotation_matrix(q)
        
        Args:
            R: 3x3 rotation matrix
            
        Returns:
            Normalized quaternion as (x, y, z, w)
        """
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return (float(x), float(y), float(z), float(w))
