# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the License);
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an AS IS BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
GraspGen Module Blueprints

Provides blueprint definitions for integrating GraspGen into Dimos applications.
"""

from dimos.core.blueprints import create_module_blueprint, ModuleBlueprintSet
from dimos.grasping.graspgen_module import GraspGenModule, GraspGenConfig


def create_graspgen_blueprint(
    docker_image: str = "dimos-graspgen",
    service_port: int = 8094,
    num_grasps: int = 400,
    topk_num_grasps: int = 100,
    **kwargs
) -> ModuleBlueprintSet:
    """Create a blueprint for the GraspGen module.
    
    Args:
        docker_image: Name of the Docker image to use
        service_port: Port to expose the service on
        num_grasps: Number of grasp candidates to generate
        topk_num_grasps: Number of top grasps to return
        **kwargs: Additional configuration options
    
    Returns:
        ModuleBlueprintSet for the GraspGen module
    
    Example:
        >>> from dimos.core.blueprints import autoconnect
        >>> from dimos.grasping.blueprints import create_graspgen_blueprint
        >>> from dimos.perception.detection.blueprints import create_yolo_blueprint
        >>> 
        >>> blueprint = autoconnect(
        ...     create_yolo_blueprint(),
        ...     create_graspgen_blueprint(num_grasps=200),
        ... )
        >>> coordinator = blueprint.build()
    """
    config = GraspGenConfig(
        docker_image=docker_image,
        service_port=service_port,
        num_grasps=num_grasps,
        topk_num_grasps=topk_num_grasps,
        **kwargs
    )
    
    return create_module_blueprint(GraspGenModule, config=config)


# Convenience alias
graspgen = create_graspgen_blueprint
