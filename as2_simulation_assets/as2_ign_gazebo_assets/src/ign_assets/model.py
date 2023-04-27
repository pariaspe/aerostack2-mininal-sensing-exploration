"""
model.py
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


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import os
import codecs
import subprocess
from enum import Enum
from pathlib import Path
from typing import Union
from pydantic import BaseModel, conlist, root_validator
from ign_assets.bridge import Bridge
import ign_assets.bridges
import ign_assets.custom_bridges
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


class Entity(BaseModel):
    """Gz Entity data model
    """
    model_name: str
    model_type: str
    xyz: conlist(float, min_items=3, max_items=3) = [0, 0, 0]
    rpy: conlist(float, min_items=3, max_items=3) = [0, 0, 0]

    def __str__(self) -> str:
        return f"{self.model_name}[{self.model_type}]"

    def generate(self, world) -> tuple[str, str]:
        """Abstrac method, childs should generate SDF by executing JINJA and populating templates

        :return python3 jinja command and path to model_sdf generated
        """
        raise NotImplementedError(
            "Abstract method, override this method in child class")


class CameraTypeEnum(str, Enum):
    """Valid camera model types"""
    VGA_CAM = 'vga_camera'
    HD_CAM = 'hd_camera'
    SEMANTIC_CAM = 'semantic_camera'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> list[Bridge]:
        """Return bridges needed for camera model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_assets.bridges.image(world_name, model_name, sensor_name,
                                     payload, model_prefix),
            ign_assets.bridges.camera_info(world_name, model_name,
                                           sensor_name, payload, model_prefix)
        ]
        return bridges


class DepthCameraTypeEnum(str, Enum):
    """Valid depth camera model types"""
    RGBD_CAM = 'rgbd_camera'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> list[Bridge]:
        """Return bridges needed for depth camera model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_assets.bridges.image(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_assets.bridges.camera_info(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_assets.bridges.depth_image(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_assets.bridges.camera_points(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class LidarTypeEnum(str, Enum):
    """Valid lidar model types"""
    POINT_LIDAR = 'point_lidar'  # FIXME: not working
    PLANAR_LIDAR = 'planar_lidar'
    LIDAR_3D = 'lidar_3d'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> list[Bridge]:
        """Return bridges needed for lidar model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_assets.bridges.lidar_scan(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_assets.bridges.lidar_points(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class GpsTypeEnum(str, Enum):
    """Valid GPS model types"""
    GPS = 'gps'

    @staticmethod
    def nodes(world_name: str, model_name: str, payload: str,
              sensor_name: str, model_prefix: str = '') -> list[Node]:
        """Return custom bridges (nodes) needed for gps model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        nodes = [ign_assets.custom_bridges.gps_node(
            world_name, model_name, sensor_name, payload)
        ]
        return nodes

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> list[Bridge]:
        """Return bridges needed for gps model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        # FIXME: current version of standard gz navsat bridge is not working properly
        bridges = [
            ign_assets.bridges.navsat(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class GripperTypeEnum(str, Enum):
    """Valid gripper model types"""
    SUCTION_GRIPPER = 'suction_gripper'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> list[Bridge]:
        """Return bridges needed for gripper model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_assets.bridges.gripper_suction_control(model_name),
            ign_assets.bridges.gripper_contact(model_name, 'center'),
            ign_assets.bridges.gripper_contact(model_name, 'left'),
            ign_assets.bridges.gripper_contact(model_name, 'right'),
            ign_assets.bridges.gripper_contact(model_name, 'top'),
            ign_assets.bridges.gripper_contact(model_name, 'bottom')
        ]
        return bridges


class Payload(Entity):
    """Gz Payload Entity

    Use model_type as sensor_type
    """
    model_type: Union[CameraTypeEnum, DepthCameraTypeEnum, LidarTypeEnum,
                      GpsTypeEnum, GripperTypeEnum]

    def bridges(self, world_name, drone_model_name) -> tuple[list[Bridge], list[Node]]:
        """Return bridges from payload model

        :param world_name: world name
        :param drone_model_name: drone model name
        :param model_prefix: ros topic prefix name, defaults to ''
        :return ([bridges], [nodes])
        bridges -> standard bridges
        nodes -> custom bridges
        """
        bridges = []
        nodes = []

        if isinstance(self.model_type, GpsTypeEnum):
            # FIXME: current version of standard gz navsat bridge is not working properly
            # custom bridge
            nodes = self.model_type.nodes(world_name, drone_model_name, self.model_type.value,
                                          self.model_name, self.model_name)
        else:
            self.model_type.bridges(world_name, drone_model_name, self.model_type.value,
                                    self.model_name, self.model_name)
        return bridges, nodes

    def generate(self, world) -> tuple[str, str]:
        """Not model generated from payload, use drone instead"""
        return "", ""


class DroneTypeEnum(str, Enum):
    """Valid drone model types"""
    QUADROTOR = 'quadrotor_base'
    HEXROTOR = 'hexrotor_base'


class Drone(Entity):
    """Gz Drone Entity"""
    model_type: DroneTypeEnum
    flight_time: int = 0  # in minutes
    battery_capacity: float = 0  # Ah
    payload: list[Payload] = []

    @root_validator
    def set_battery_capacity(cls, values: dict) -> dict:
        """Set battery capacity with a given flight time"""
        flight_time = float(values.get('flight_time', 0))

        # calculate battery capacity from time
        # capacity (Ah) = flight time (in hours) * load (watts) / voltage
        # assume constant voltage for battery to keep things simple for now.
        battery_capacity = (flight_time / 60) * 6.6 / 12.694
        values['battery_capacity'] = battery_capacity
        return values

    def __str__(self) -> str:
        pld_str = ""
        for pld in self.payload:
            pld_str += f" {pld}"
        return f"{super().__str__()}:{pld_str}"

    def get_index(self, world: 'World') -> int:
        """From a world.drones list which instance am I"""
        return world.drones.index(self)

    def bridges(self, world_name: str) -> tuple[list[Bridge], list[Node]]:
        """Return gz_to_ros bridges needed for the drone to fly

        :return ([bridges], [nodes])
        bridges -> standard bridges
        nodes -> custom bridges
        """
        bridges = [
            # IMU
            ign_assets.bridges.imu(
                world_name, self.model_name, 'imu', 'internal'),
            # Magnetometer
            ign_assets.bridges.magnetometer(
                world_name, self.model_name, 'magnetometer', 'internal'),
            # Air Pressure
            ign_assets.bridges.air_pressure(
                world_name, self.model_name, 'air_pressure', 'internal'),
            # odom: deprecated; not used, use ground_truth instead
            # ign_assets.bridges.odom(self.model_name),
            # pose
            ign_assets.bridges.tf_pose(self.model_name),
            # pose static
            ign_assets.bridges.tf_pose_static(self.model_name),
            # twist
            ign_assets.bridges.cmd_vel(self.model_name),
            # arm
            ign_assets.bridges.arm(self.model_name)
        ]
        if self.battery_capacity != 0:
            bridges.append(ign_assets.bridges.battery(self.model_name))

        nodes = [
            # Odom --> ground_truth
            ign_assets.custom_bridges.ground_truth_node(self.model_name),
            # Deprecated
            # ign_assets.custom_bridges.tf_broadcaster_node(world_name, self.model_name)
        ]

        bridges_, nodes_ = self.payload_bridges(world_name)

        bridges.extend(bridges_)
        nodes.extend(nodes_)

        return bridges, nodes

    def payload_bridges(self, world_name: str) -> tuple[list[Bridge], list[Node]]:
        """Get bridges from payload"""
        bridges = []
        nodes = []
        for pld in self.payload:
            bridges_, nodes_ = pld.bridges(
                world_name, drone_model_name=self.model_name)
            bridges.extend(bridges_)
            nodes.extend(nodes_)

        return bridges, nodes

    def get_model_jinja_template(self) -> Path:
        """Return Path of self jinja template"""
        # Concatenate the model directory and the IGN_GAZEBO_RESOURCE_PATH environment variable
        model_dir = Path(get_package_share_directory(
            'as2_ign_gazebo_assets'), 'models')
        resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH')

        paths = [model_dir]
        if resource_path:
            paths += [Path(p) for p in resource_path.split(':')]

        # Define the filename to look for
        filename = f'{self.model_type}/{self.model_type}.sdf.jinja'

        # Loop through each directory and check if the file exists
        for path in paths:
            filepath = path / filename
            if filepath.is_file():
                # If the file exists, return the path
                return filepath
        raise FileNotFoundError(
            f'{filename} not found in {paths}. Does the model jinja template exists?')

    def generate(self, world) -> tuple[str, str]:
        """Generate SDF by executing JINJA and populating templates

        :raises RuntimeError: if jinja fails
        :return: python3 jinja command and path to model_sdf generated
        """

        # Concatenate the model directory and the IGN_GAZEBO_RESOURCE_PATH environment variable
        model_dir = Path(get_package_share_directory(
            'as2_ign_gazebo_assets'), 'models')
        jinja_script = os.path.join(
            get_package_share_directory('as2_ign_gazebo_assets'), 'scripts')

        payload = ""
        for pld in self.payload:
            x_s, y_s, z_s = pld.xyz
            roll_s, pitch_s, yaw_s = pld.rpy

            payload += f"{pld.model_name} {pld.model_type} {x_s} {y_s} {z_s} "
            payload += f"{roll_s} {pitch_s} {yaw_s} "

        output_file_sdf = f"/tmp/{self.model_type}_{self.get_index(world)}.sdf"
        command = ['python3', f'{jinja_script}/jinja_gen.py', self.get_model_jinja_template(),
                   f'{model_dir}/..', '--namespace', f'{self.model_name}',
                   '--sensors', f'{payload}', '--battery', f'{self.flight_time}',
                   '--output-file', f'{output_file_sdf}']

        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output to see if there were undefined variables
        # for the JINJA process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        return command, output_file_sdf


class ObjectBridgesTypeEnum(str, Enum):
    """Valid drone model types"""
    GPS = 'gps'
    AZIMUTH = 'azimuth'
    POSE = 'pose'

    def bridges(self, world_name: str, model_name: str) -> tuple[list[Bridge], list[Node]]:
        """Return associated bridge or custom bridge to BridgeType.
        First list of bridges, the list of nodes.
        """
        if self.name == self.GPS.name:
            return [], [ign_assets.custom_bridges.gps_node(
                world_name, model_name, 'gps', 'gps')]
        if self.name == self.AZIMUTH.name:
            return [], [ign_assets.custom_bridges.azimuth_node(model_name)]
        if self.name == self.POSE.name:
            return [ign_assets.bridges.pose(model_name)], []
        return [], []


class Object(Entity):
    """Gz Object Entity"""
    joints: list[str]
    object_bridges: list[ObjectBridgesTypeEnum]
    tf_broadcaster: bool = True
    use_sim_time: bool = True

    def bridges(self, world_name: str):
        """Object bridges
        """
        bridges = self.joint_bridges()
        nodes = []
        if self.tf_broadcaster:
            nodes.append(ign_assets.custom_bridges.tf_broadcaster_node(
                world_name, self.model_name, 'earth', self.use_sim_time))

        for bridge in self.object_bridges:
            bridges_, nodes_ = bridge.bridges(world_name, self.model_name)
            bridges.extend(bridges_)
            nodes.extend(nodes_)
        return bridges, nodes

    def joint_bridges(self) -> list[Bridge]:
        """Return gz_to_ros bridges needed for the object to move"""
        bridges = []
        for joint in self.joints:
            bridges.append(ign_assets.bridges.joint_cmd_vel(
                self.model_name, joint))
        return bridges

    def generate(self, world) -> tuple[str, str]:
        """Object are not jinja templates, no need for creating, using base one"""
        model_dir = os.path.join(
            get_package_share_directory('as2_ign_gazebo_assets'), 'models')

        model_sdf = f'{model_dir}/{self.model_type}/{self.model_type}.sdf'
        return "", model_sdf


class World(BaseModel):
    """Gz World"""
    world_name: str
    drones: list[Drone] = []
    objects: list[Object] = []

    def __str__(self) -> str:
        drones_str = ""
        for drone in self.drones:
            drones_str += f"\n\t{drone}"
        return f"{self.world_name}:{drones_str}"

    def get_drone_index(self, drone: Drone) -> int:
        """Get drone index"""
        return self.drones.index(drone)

    def get_object_index(self, object_: Object) -> int:
        """Get object index"""
        return self.drones.index(object_)


def spawn_args(world: World, model: Union[Drone, Object]) -> list[str]:
    """Return args to spawn model_sdf in Gz"""
    command, model_sdf = model.generate(world)

    return ['-world', world.world_name,
            '-file', model_sdf,
            '-name', model.model_name,
            '-allow_renaming', 'false',
            '-x', str(model.xyz[0]),
            '-y', str(model.xyz[1]),
            '-z', str(model.xyz[2]),
            '-R', str(model.rpy[0]),
            '-P', str(model.rpy[1]),
            '-Y', str(model.rpy[2])]


def dummy_world() -> World:
    """Create dummy world
    """
    drone = Drone(model_name="dummy",
                  model_type=DroneTypeEnum.QUADROTOR, flight_time=60)
    cam = Payload(model_name="front_camera", model_type="hd_camera")
    gps = Payload(model_name="gps0", model_type="gps")
    drone.payload.append(cam)
    drone.payload.append(gps)

    world = World(world_name="empty", drones=[drone])
    return world


if __name__ == "__main__":
    WORLD_JSON = """
    {
        "world_name": "empty",
        "drones": [
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_sim_0",
            "xyz": [ 0.0, 0.0, 0.2 ],
            "rpy": [ 0, 0, 1.57 ],
            "flight_time": 60,
            "payload": [
                {
                    "model_name": "front_camera",
                    "model_type": "hd_camera",
                    "xyz": [0.1, 0.2, 0.3]
                },
                {
                    "model_name": "lidar_0",
                    "model_type": "lidar_3d",
                    "rpy": [ 0.0, 0.0, 0.0 ]
                }
            ]
        },
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_sim_1",
            "xyz": [ 3.0, 0.0, 0.2 ],
            "rpy": [ 0, 0, 1.57 ],
            "payload": [
                {
                    "model_name": "camera",
                    "model_type": "hd_camera",
                    "rpy": [ 0.0, 0.0, 0.0 ]
                },
                {
                    "model_name": "gps0",
                    "model_type": "gps",
                    "xyz": [ 0.0, 0.0, 0.08 ]
                }
            ]
        }
        ]
    }
    """
    world_model = World.parse_raw(WORLD_JSON)
    print(world_model)

    # for drone in world_model.drones:
    #     _, sdf = drone.generate(world_model)
    #     print(sdf)
    print(dummy_world())
    print(dict(dummy_world()))
