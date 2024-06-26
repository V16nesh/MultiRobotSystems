import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='multi_robot_system' 
    world_file_name = 'house.world'
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_state_pub.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path}.items(),
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'Single_robot'],
                        output='screen')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'slam_toolbox_param.yaml')]
    )
    def create_teleop_node(namespace):
        return Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix=['xterm -e']
        )
    
    teleop_robot = create_teleop_node("Robot")

    

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        slam_toolbox,
        teleop_robot,
    ])