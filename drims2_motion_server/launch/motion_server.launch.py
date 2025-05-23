
# Copyright 2024 National Research Council STIIMA
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_configs():
    srdf_path = os.path.join(get_package_share_directory('ur_robotiq_moveit_config'), 'config', 'ur_robotiq.srdf')
    joint_limits_path = os.path.join(get_package_share_directory('ur_robotiq_moveit_config'), 'config', 'joint_limits.yaml')
    moveit_controllers_path = os.path.join(get_package_share_directory('ur_robotiq_bringup'), 'config', 'moveit_controllers.yaml')
    pilz_limits_path = os.path.join(get_package_share_directory('ur_robotiq_moveit_config'), 'config', 'pilz_cartesian_limits.yaml')
    robot_description_path = os.path.join(get_package_share_directory('ur_robotiq_description'), 'urdf', 'ur_robotiq.urdf.xacro') 

    moveit_config = (
        MoveItConfigsBuilder('ur_robotiq', package_name='ur_robotiq_moveit_config')
        .robot_description(file_path=robot_description_path,
                            mappings= {"fake_ur": 'true',
                                       "fake_gripper": 'true', 
                                       "ur_type": 'ur10e', 
                                       "robot_name": 'ur10e', 
                                       "tf_prefix": 'ur10e_', 
                                       "tool_device_name": '/tmp/ttyUR', 
                                       "use_tool_communication": 'false', 
                                       "tool_tcp_port": '54321', 
                                       "headless_mode": 'true',})
        .robot_description_semantic(file_path=srdf_path)
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True)
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
        .pilz_cartesian_limits(file_path=pilz_limits_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=moveit_controllers_path)
        .robot_description_kinematics()
        .moveit_cpp(
            file_path=get_package_share_directory("python_examples")
            + "/config/motion_planning_python.yaml"
        )

        .to_moveit_configs()
    )
    return moveit_config.to_dict()

def generate_launch_description():
    pkg_dir = get_package_share_directory('drims2_motion_server')

    motion_server_config_path_cmd = DeclareLaunchArgument(
        'motion_server_config_path',
        default_value=pkg_dir + '/config/motion_server_config.yaml',
        description='Full path to the config file')

    motion_server_node = Node(
        package='drims2_motion_server',
        executable='motion_server',
        name='motion_server_node',
        output='screen',
        parameters=[
            LaunchConfiguration('motion_server_config_path'),
            get_moveit_configs()
        ])
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(motion_server_config_path_cmd)
    ld.add_action(motion_server_node)
    return ld