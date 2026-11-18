import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
    DeclareLaunchArgument('log_level', default_value='info', description='log level'),
    # the simulator's clock for both nodes (liveness and dwell timers): $USE_SIM_TIME in the base station container
    DeclareLaunchArgument('use_sim_time', default_value=os.environ.get('USE_SIM_TIME', 'false'),
                          choices=['true', 'false'], description='Run on the simulator clock'),
]


def generate_launch_description():
    LogInfo(msg="Launching nodes..."),
    remappings_tf=[
            ('/tf','tf'),
            ('/tf_static','tf_static'),
        ]
    
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')

    group_action = GroupAction(
        actions=[
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            Node(
                package='status_server',
                executable='status_server',
                name='status_server',
                output='screen',
            ),
            # Task Manager Node
            Node(
                package='status_server',
                executable='task_manager',
                name='task_manager',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(group_action)

    return ld
