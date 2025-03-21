import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.actions
import os
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='bringup').find('bringup')
    default_model_path = os.path.join(pkg_share, 'src/description/diff_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'world/my_world.sdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )
    local_costmap_node = launch_ros.actions.Node(
        package='local_costmap',
        executable='local_costmap',
        name='local_costmap',
        output='screen',
    )
    my_map_node = launch_ros.actions.Node(
        package='my_map',
        executable='my_map',
        name='my_map',
        output='screen',
    )
    subp3d_pubtf_node = launch_ros.actions.Node(
        package='subp3d_pubtf',
        executable='tf_publisher',
        name='subp3d_pubtf',
        output='screen',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        # 启动 Gazebo
        start_gazebo,
        local_costmap_node,
        my_map_node,
        subp3d_pubtf_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
    ])