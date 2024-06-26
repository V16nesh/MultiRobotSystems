import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace1 = LaunchConfiguration('namespace1')
    namespace2 = LaunchConfiguration('namespace2')
   
    pkg_path = os.path.join(get_package_share_directory('multi_robot_system'))
    rviz_config_file = os.path.join(pkg_path, 'config', 'multi_spawn.rviz')
    world_file_name = 'house.world'
    world_file_path = os.path.join(pkg_path, 'worlds', world_file_name)
    xacro_file1 = os.path.join(pkg_path, 'urdf', 'bot1.urdf.xacro')
    xacro_file2 = os.path.join(pkg_path, 'urdf', 'bot2.urdf.xacro')
    robot_description_config1 = xacro.process_file(xacro_file1)
    robot_description_config2 = xacro.process_file(xacro_file2)
    params1 = {'robot_description': robot_description_config1.toxml(), 'use_sim_time': use_sim_time}
    params2 = {'robot_description': robot_description_config2.toxml(), 'use_sim_time': use_sim_time}


    node_robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace1,
        parameters=[params1]
    )
    node_robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace2,
        parameters=[params2]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items(),
        

    )
    spawn_entity1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot1/robot_description',
                                   '-entity', 'robot1',
                                   '-x', '0',
                                   '-y', '2',
                                   '-z', '0'],
                        output='screen')

    spawn_entity2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot2/robot_description',
                                   '-entity', 'robot2',
                                   '-x', '2',
                                   '-y', '0',
                                   '-z', '0'],
                        output='screen')
    def create_teleop_node(robotname, cmd_vel_topic):
        return Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name=f'teleop_{robotname}',
            output='screen',
            prefix=['xterm -e'],
            remappings=[('/cmd_vel', cmd_vel_topic)]
        )
    
    teleop_robot1 = create_teleop_node("robot1", "/robot1/cmd_vel")
    teleop_robot2 = create_teleop_node("robot2", "/robot2/cmd_vel")

   
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'namespace1',
            default_value='/robot1',  
            description='Namespace for robot1'),
        DeclareLaunchArgument(
            'namespace2',
            default_value='/robot2', 
            description='Namespace for robot2'),

        gazebo_launch,
        node_robot_state_publisher1,
        node_robot_state_publisher2,
        rviz_node,
        spawn_entity1,
        spawn_entity2,
        teleop_robot1,
        teleop_robot2
        
    ])
