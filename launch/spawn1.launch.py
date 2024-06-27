import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    world_file_name = 'house.world'
    world_file_path = os.path.join(get_package_share_directory('multi_robot_system'), 'worlds', world_file_name)
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    pkg_path = os.path.join(get_package_share_directory('multi_robot_system'))

    xacro_file = os.path.join(pkg_path, 'urdf', 'bot1.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'world': world_file_path,
                        'use_sim_time': 'true'
    }.items(),
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot1/robot_description',
                                   '-entity', 'robot1',
                                   '-x', '1',
                                   '-y', '-1',
                                   '-z', '0'],
                        output='screen')
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[params]
    )
    rviz_config_file = os.path.join(pkg_path, 'config', 'lidarcheck.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='/robot1', 
            description='Namespace for the robot'),

        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node
    ])
