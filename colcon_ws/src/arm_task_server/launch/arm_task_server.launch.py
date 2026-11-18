"""arm_task_server for this robot: namespace and platform from /etc/clearpath/robot.yaml, the shared defaults, the
platform's ready_state, then config/robots/<namespace>.yaml when it exists. Included by mtu32_bringup's
bringup_main.launch.py next to the grid cutter (robots with an arm), on sim and real robots alike.

    ros2 launch arm_task_server arm_task_server.launch.py [use_sim_time:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# GO READY's SRDF state per platform: j100_0921's cut_init is next to the cutter's IK seed; the a200's and a300's
# named poses are all far from it, so `ready` (the arm's built-in) only unfolds the arm.
PLATFORM_READY = {'j100': 'cut_init', 'a200': 'ready', 'a300': 'ready'}


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path').perform(context)
    config = read_yaml(os.path.join(setup_path, 'robot.yaml'))
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace
    platform = clearpath_config.platform.get_platform_model()
    share = get_package_share_directory('arm_task_server')
    parameters = [
        os.path.join(share, 'config', 'arm_task_server.yaml'),
        {'ready_state': PLATFORM_READY.get(platform, 'ready'), 'srdf_path': os.path.join(setup_path, 'robot.srdf')},
    ]
    robot_file = os.path.join(share, 'config', 'robots', f'{namespace}.yaml')
    if os.path.isfile(robot_file):
        parameters.append(robot_file)
    parameters.append({'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'})
    return [Node(
        package='arm_task_server',
        executable='arm_task_server',
        name='arm_task_server',
        namespace=namespace,
        parameters=parameters,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/', description='Clearpath setup path'),
        DeclareLaunchArgument('use_sim_time', default_value=os.environ.get('USE_SIM_TIME', 'false'),
                              choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
