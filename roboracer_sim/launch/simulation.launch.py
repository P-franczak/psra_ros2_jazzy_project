import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    pkg_dir = get_package_share_directory('roboracer_sim')
    description_dir = get_package_share_directory('roboracer_description')

    world_file = os.path.join(pkg_dir, 'worlds', 'track.sdf')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')
    xacro_file = PathJoinSubstitution([description_dir, 'urdf', 
        PythonExpression([
            "'roboracer_gym.urdf.xacro' if '",
            LaunchConfiguration('model'),
            "' == 'gym' else 'roboracer.urdf.xacro'"
        ])
    ])
    rviz_config = os.path.join(description_dir, 'rviz', 'roboracer.rviz')


    body_arg = DeclareLaunchArgument('body', default_value='shell', choices=['shell', 'open'])
    model_arg = DeclareLaunchArgument('model', default_value='gym', choices=['classic', 'gym'])
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')


    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' body:=', LaunchConfiguration('body')]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'roboracer',
                   '-x', '0.0', '-y', '-2.0', '-z', '0.10'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config, '-f', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    line_follower = Node(
    package='roboracer_sim',
    executable='line_follower_pure_pursuit',
    parameters=[{'use_sim_time': True,
                 'camera_height': 0.12,      # from URDF
                 'camera_pitch_deg': 0.0,   # from URDF
                 'line_rgb': [250, 215, 20]}],
    output='screen'
)

    return LaunchDescription([
        body_arg,
        model_arg,
        rviz_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        rviz,
    ])