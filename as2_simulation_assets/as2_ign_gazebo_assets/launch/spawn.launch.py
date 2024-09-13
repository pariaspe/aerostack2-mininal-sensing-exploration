"""
launch_simulation.py
"""

# Copyright 2022 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Pedro Arias Pérez, Javier Melero Deza, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import os
import json
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ign_assets.world import World, spawn_args


def spawn(world: World) -> List[Node]:
    """Spawn models (drones and objects) of world"""
    models = world.drones + world.objects
    launch_processes = []
    for model in models:
        # ros2 run ros_gz_sim create -world ARG -file FILE
        ignition_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=spawn_args(world, model)
        )
        launch_processes.append(ignition_spawn_entity)

    return launch_processes


def world_bridges():
    """Create world bridges. Mainly clock if sim_time enabled."""
    world_bridges_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('as2_ign_gazebo_assets'), 'launch'),
            '/world_bridges.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )
    return [world_bridges_]


def object_bridges():
    """Create object bridges."""
    object_bridges_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('as2_ign_gazebo_assets'), 'launch'),
            '/object_bridges.py']),
        launch_arguments={
            'simulation_config_file': LaunchConfiguration('simulation_config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )
    return [object_bridges_]


def launch_spawn(context: LaunchContext):
    """Return processes needed for launching the simulation.
    Simulator + Spawning Models + Bridges.
    """
    config_file = LaunchConfiguration(
        'simulation_config_file').perform(context)

    with open(config_file, 'r', encoding='utf-8') as stream:
        config = json.load(stream)
        world = World(**config)

    launch_processes = []
    launch_processes.extend(spawn(world))
    launch_processes.extend(world_bridges() + object_bridges())
    return launch_processes


def generate_launch_description():
    """Generate Launch description with GzSim launch + Models Spawning + World/Object bridges
    """
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'simulation_config_file',
            description='Launch config file (JSON or YAML format).'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Deactivates clock bridge and object publishes tf in sys clock time.'),
        # Launch processes
        OpaqueFunction(function=launch_spawn),
    ])
