# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import codecs
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import vrx_gz.bridges
import vrx_gz.payload_bridges
import pathlib
import shutil

import yaml

UAVS = [
    'vrx_hexrotor',
    'vrx_quadrotor'
]

USVS = [
    'usv',
    'wam-v',
]

WAVEFIELD_SIZE = {'sydney_regatta': 1000,}


class Model:

    def __init__(self, model_name, model_type, position):
        self.model_name = model_name
        self.model_type = model_type
        self.position = position
        self.battery_capacity = 0
        self.wavefield_size = 0
        self.payload = {}

    def is_UAV(self):
        return self.model_type in UAVS

    def is_USV(self):
        return self.model_type in USVS

    def bridges(self, world_name):
        custom_launches = []
        nodes = []
        bridges = [
            # IMU
            vrx_gz.bridges.imu(world_name, self.model_name),
            # pose
            vrx_gz.bridges.pose(self.model_name),
            # pose static
            vrx_gz.bridges.pose_static(self.model_name),
            # comms tx
            vrx_gz.bridges.comms_tx(self.model_name),
            # comms rx
            vrx_gz.bridges.comms_rx(self.model_name),
        ]
        if self.is_UAV():
            bridges.extend([
                # Magnetometer
                vrx_gz.bridges.magnetometer(world_name, self.model_name),
                # Air Pressure
                vrx_gz.bridges.air_pressure(world_name, self.model_name),
            ])
            bridges.extend([
                # twist
                vrx_gz.bridges.cmd_vel(self.model_name)
            ])
        elif self.is_USV():
            bridges.extend([
                # thrust cmd
                vrx_gz.bridges.thrust(self.model_name, 'left'),
                vrx_gz.bridges.thrust(self.model_name, 'right'),
                # thrust joint pos cmd
                vrx_gz.bridges.thrust_joint_pos(self.model_name, 'left'),
                vrx_gz.bridges.thrust_joint_pos(self.model_name, 'right'),
            ])

        return [bridges, nodes, custom_launches]

    def payload_bridges(self, world_name, payloads=None):
        # payloads on usv and uav
        if not payloads:
            payloads = self.payload
        bridges, nodes, payload_launches = self.payload_bridges_impl(world_name, payloads)

        return [bridges, nodes, payload_launches]

    def payload_bridges_impl(self, world_name, payloads):
        bridges = []
        nodes = []
        payload_launches = []
        for (idx, k) in enumerate(sorted(payloads.keys())):
            p = payloads[k]
            if not p['sensor'] or p['sensor'] == 'None' or p['sensor'] == '':
                continue

            # check if it is a custom payload
            if self.is_custom_model(p['sensor']):
                payload_launch = self.custom_payload_launch(world_name, self.model_name,
                                                            p['sensor'], idx)
                if payload_launch is not None:
                    payload_launches.append(payload_launch)

            # if not custom payload, add our own bridges and nodes
            else:
                model_prefix = ''
                ros_slot_prefix = f'slot{idx}'
                bridges.extend(
                    vrx_gz.payload_bridges.payload_bridges(
                        world_name, self.model_name, p['sensor'], idx, model_prefix))

                if p['sensor'] in vrx_gz.payload_bridges.camera_models():
                    nodes.append(Node(
                        package='vrx_ros',
                        executable='optical_frame_publisher',
                        arguments=['1'],
                        remappings=[('input/image', f'{ros_slot_prefix}/image_raw'),
                                    ('output/image', f'{ros_slot_prefix}/optical/image_raw'),
                                    ('input/camera_info', f'{ros_slot_prefix}/camera_info'),
                                    ('output/camera_info',
                                     f'{ros_slot_prefix}/optical/camera_info')]))
                elif p['sensor'] in vrx_gz.payload_bridges.rgbd_models():
                    nodes.append(Node(
                        package='vrx_ros',
                        executable='optical_frame_publisher',
                        arguments=['1'],
                        remappings=[('input/image', f'{ros_slot_prefix}/image_raw'),
                                    ('output/image', f'{ros_slot_prefix}/optical/image_raw'),
                                    ('input/camera_info', f'{ros_slot_prefix}/camera_info'),
                                    ('output/camera_info',
                                     f'{ros_slot_prefix}/optical/camera_info')]))
                    nodes.append(Node(
                        package='vrx_ros',
                        executable='optical_frame_publisher',
                        arguments=['1'],
                        remappings=[('input/image', f'{ros_slot_prefix}/depth'),
                                    ('output/image', f'{ros_slot_prefix}/optical/depth')]))
        return [bridges, nodes, payload_launches]

    def is_custom_model(self, model):
        if not model:
            return False
        try:
            get_package_share_directory(model)
        except PackageNotFoundError:
            return False
        return True

    def custom_model_launch(self, world_name, model_name, model):
        custom_launch = None
        path = os.path.join(
            get_package_share_directory(model), 'launch')
        if os.path.exists(path):
            custom_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([path, '/bridge.launch.py']),
                launch_arguments={'world_name': world_name,
                                  'model_name': model_name}.items())
        return custom_launch

    def custom_payload_launch(self, world_name, model_name, payload, idx):
        payload_launch = None
        path = os.path.join(
            get_package_share_directory(payload), 'launch')
        if os.path.exists(path):
            payload_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([path, '/bridge.launch.py']),
                launch_arguments={'world_name': world_name,
                                  'model_name': model_name,
                                  'slot_idx': str(idx)}.items())
        return payload_launch

    def set_flight_time(self, flight_time):
        # UAV specific, sets flight time

        # calculate battery capacity from time
        # capacity (Ah) = flight time (in hours) * load (watts) / voltage
        # assume constant voltage for battery to keep things simple for now.
        self.battery_capacity = (float(flight_time) / 60) * 6.6 / 12.694

    def set_payload(self, payload):
        # UAV specific
        self.payload = payload

    def set_wavefield(self, world_name):
        if world_name not in WAVEFIELD_SIZE:
            print(f'Wavefield size not found for {world_name}')
        else:
            self.wavefield_size = WAVEFIELD_SIZE[world_name]

    def generate(self):
        # Generate SDF by executing ERB and populating templates
        template_file = os.path.join(
            get_package_share_directory('vrx_gz'),
            'models', self.model_type, 'model.sdf.erb')

        model_dir = os.path.join(get_package_share_directory('vrx_gz'), 'models')
        model_tmp_dir = os.path.join(model_dir, 'tmp')

        command = ['erb']
        command.append(f'name={self.model_name}')

        for (slot, payload) in self.payload.items():
            if payload['sensor'] and payload['sensor'] != 'None':
                command.append(f"{slot}={payload['sensor']}")
            if 'rpy' in payload:
                if type(payload['rpy']) is str:
                    r, p, y = payload['rpy'].split(' ')
                else:
                    r, p, y = payload['rpy']
                command.append(f'{slot}_pos={r} {p} {y}')

        if self.model_type in UAVS:
            if self.battery_capacity == 0:
                raise RuntimeError('Battery Capacity is zero, was flight_time set?')
            command.append(f'capacity={self.battery_capacity}')

        if self.model_type in USVS:
            command.append(f'wavefieldSize={self.wavefield_size}')

        command.append(template_file)
        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output to see if there were undefined variables
        # for the ERB process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        stdout = process.communicate()[0]
        model_sdf = codecs.getdecoder('unicode_escape')(stdout)[0]
        print(command)

        return command, model_sdf

    def spawn_args(self, model_sdf=None):
        if not model_sdf:
            [command, model_sdf] = self.generate()

        return ['-string', model_sdf,
                '-name', self.model_name,
                '-allow_renaming', 'false',
                '-x', str(self.position[0]),
                '-y', str(self.position[1]),
                '-z', str(self.position[2]),
                '-R', str(self.position[3]),
                '-P', str(self.position[4]),
                '-Y', str(self.position[5])]

    @classmethod
    def FromConfig(cls, stream):
        # Generate a Model instance (or multiple instances) from a stream
        # Stream can be either a file input or string
        config = yaml.safe_load(stream)

        if type(config) == list:
            return cls._FromConfigList(config)
        elif type(config) == dict:
            return cls._FromConfigDict(config)

    @classmethod
    def _FromConfigList(cls, entries):
        # Parse an array of configurations
        ret = []
        for entry in entries:
            ret.append(cls._FromConfigDict(entry))
        return ret

    @classmethod
    def _FromConfigDict(cls, config):
        # Parse a single configuration
        if 'model_name' not in config:
            raise RuntimeError('Cannot construct model without model_name in config')
        if 'model_type' not in config:
            raise RuntimeError('Cannot construct model without model_type in config')

        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if 'position' not in config:
            print('Position not found in config, defaulting to (0, 0, 0), (0, 0, 0)')
        else:
            if 'xyz' in config['position']:
                xyz = config['position']['xyz']
            if 'rpy' in config['position']:
                rpy = config['position']['rpy']
        model = cls(config['model_name'], config['model_type'], [*xyz, *rpy])

        if 'flight_time' in config:
            model.set_flight_time(config['flight_time'])

        if 'payload' in config:
            model.set_payload(config['payload'])

        return model