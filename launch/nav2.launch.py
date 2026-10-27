# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
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
import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_config.platform.drivetrain import DrivetrainConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic',
                          default_value='',
                          description='Override the default 2D laserscan topic'),
    DeclareLaunchArgument('slam', default_value='false',
                          choices=['true', 'false'],
                          description='Use a rolling global costmap for SLAM instead of the '
                                      'full static map used for localization'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically start up the Nav2 stack'),
    DeclareLaunchArgument('use_composition', default_value='false',
                          choices=['true', 'false'],
                          description='Load the Nav2 nodes into a single component container'),
    DeclareLaunchArgument('use_respawn', default_value='false',
                          choices=['true', 'false'],
                          description='Respawn crashed nodes when use_composition is false'),
    DeclareLaunchArgument('log_level', default_value='info',
                          description='Log level')
]

SCAN_TOPIC_PARAMS = [
    'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic',
    'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic',
    'collision_monitor.ros__parameters.scan.topic',
]

MOTION_MODEL_PARAM = 'controller_server.ros__parameters.FollowPath.motion_model'
TWIRLING_CRITIC_PARAM = 'controller_server.ros__parameters.FollowPath.TwirlingCritic.enabled'
GLOBAL_ROLLING_WINDOW_PARAM = 'global_costmap.global_costmap.ros__parameters.rolling_window'


def launch_setup(context, *args, **kwargs):
    # Packages
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    scan_topic = LaunchConfiguration('scan_topic')
    autostart = LaunchConfiguration('autostart')
    # nav2_bringup evaluates this with PythonExpression, so it must be True/False
    use_composition = str(LaunchConfiguration('use_composition').perform(context) == 'true')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    container_name = 'nav2_container'

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    # see if we've overridden the scan_topic
    eval_scan_topic = scan_topic.perform(context)
    if len(eval_scan_topic) == 0:
        eval_scan_topic = f'/{namespace}/sensors/lidar2d_0/scan'

    file_parameters = PathJoinSubstitution([
        pkg_clearpath_nav2_demos,
        'config',
        platform_model,
        'nav2.yaml'])

    param_rewrites = {param: eval_scan_topic for param in SCAN_TOPIC_PARAMS}
    # Mecanum platforms can be configured in robot.yaml to drive as diff-drive
    if clearpath_config.platform.drivetrain.control != DrivetrainConfig.OMNI_4WD:
        param_rewrites[MOTION_MODEL_PARAM] = 'DiffDrive'
        # TwirlingCritic penalizes all rotation, which a diff-drive robot needs to steer
        param_rewrites[TWIRLING_CRITIC_PARAM] = 'false'
    if LaunchConfiguration('slam').perform(context) != 'true':
        # The static layer then sizes the global costmap to the whole map
        param_rewrites[GLOBAL_ROLLING_WINDOW_PARAM] = 'false'

    rewritten_parameters = RewrittenYaml(
        source_file=file_parameters,
        param_rewrites=param_rewrites,
        convert_types=True
    )

    # The container creates the costmap sub-nodes, so it needs namespaced params
    container_parameters = RewrittenYaml(
        source_file=rewritten_parameters,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    launch_nav2 = PathJoinSubstitution(
      [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),

        Node(
            condition=IfCondition(use_composition),
            package='rclcpp_components',
            executable='component_container_isolated',
            name=container_name,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[container_parameters,
                        {'autostart': autostart, 'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', rewritten_parameters),
                ('autostart', autostart),
                ('use_composition', use_composition),
                ('container_name', container_name),
                ('use_respawn', use_respawn),
                ('log_level', log_level),
                ('namespace', namespace)
              ]
        ),
    ])

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
