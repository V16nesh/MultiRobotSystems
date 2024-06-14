import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = 'multi_robot_system'
    world_file_name = 'house.world'
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)

    coordinates = [(-1.5, 1.0), (-1.0, 1.0)]
    last_action = None

    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )
    ld.add_action(gazebo)

    def create_spawn_entity_node(name, namespace, x, y):
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'{namespace}/robot_description',
                '-entity', name,
                '-robot_namespace', namespace,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.01'
            ],
            output='screen'
        )
    def create_teleop_node(namespace):
        return Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            namespace=namespace,
            output='screen',
            remappings=[('/cmd_vel', f'{namespace}/cmd_vel')],
            prefix=['xterm -e']
        )

    for index, (x, y) in enumerate(coordinates):
        name = f"robot{index}"
        namespace = f"/robot{index}"

        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'robot_state_pub.launch.py'
            )]), launch_arguments={
                'use_sim_time': 'true',
                'namespace': namespace
            }.items()
        )

        spawn_robot = create_spawn_entity_node(name, namespace, x, y)
        teleop_robot = create_teleop_node(namespace)

        ld.add_action(rsp)
        
        if last_action is None:
            ld.add_action(spawn_robot)
        else:
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_robot],
                )
            )
            ld.add_action(spawn_robot_event)

        last_action = spawn_robot

        ld.add_action(teleop_robot)

    return ld
