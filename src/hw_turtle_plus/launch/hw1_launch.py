from launch.actions import  ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

def generate_launch_description():

    package_dir = get_package_share_directory('turtle_controller')
    via_point_dir = os.path.abspath(os.path.join(package_dir, '..', '..', '..', '..', 'src', 'path', 'pizzapath.yaml'))

    turtlesim_plus_namespace_list = ["template_window", "copy_window"]
    turtle_name_list = ['iron', 'humble', 'foxy', 'galactic', 'noetic']
    via_point_name_list = ['save_count_1', 'save_count_2', 'save_count_3', 'save_count_4']

    launch_description = LaunchDescription()

    for i in range(len(turtlesim_plus_namespace_list)):
        turtlesimPlus = Node(
            package='turtlesim_plus',
            executable='turtlesim_plus_node.py',
            name='copy_window',
            namespace=turtlesim_plus_namespace_list[i]
        )
        launch_description.add_action(turtlesimPlus)

        remove_turtle1 = ExecuteProcess(
            cmd=["ros2 service call /"+turtlesim_plus_namespace_list[i]+"/remove_turtle turtlesim/srv/Kill \"name: 'turtle1'\""],
            shell=True,
        )
        launch_description.add_action(remove_turtle1)

    for i in range(len(turtle_name_list)):
        if i == 0: idx = 0
        else: idx = 1
        spawn_turtle = ExecuteProcess(
            cmd=["ros2 service call /"+turtlesim_plus_namespace_list[idx]+"/spawn_turtle turtlesim/srv/Spawn \"{x: 1, y: 1, theta: 0.0, name: " + turtle_name_list[i] + "}\""],
            shell=True,
        )
        controller = Node(
            package='turtle_controller',
            executable='controller_namespace.py',
            namespace=turtlesim_plus_namespace_list[idx] + '/' + turtle_name_list[i]
        )
        if i == 0:
            scheduler = Node(
                package='turtle_teleop',
                executable='teleop_schedule.py',
                namespace=turtlesim_plus_namespace_list[idx] + '/' + turtle_name_list[i],
                parameters=[{
                    'via_point_file': via_point_dir,
                    'pizza_max': 150
                }],
                remappings=[
                    ('/spawn_pizza', '/' + turtlesim_plus_namespace_list[idx] + '/spawn_pizza'),
                    ('/' + turtlesim_plus_namespace_list[idx] + '/' + turtle_name_list[i] + '/noti', '/' + turtlesim_plus_namespace_list[idx] + '/' + turtle_name_list[i] + '/reach_notify'),
                    ('/' + turtlesim_plus_namespace_list[idx] + '/' + turtle_name_list[i] + '/teleop_target', '/' + turtlesim_plus_namespace_list[idx] + '/' + turtle_name_list[i] + '/target'),
                ]
            )
        else:
            scheduler = Node(
                package='turtle_controller',
                executable='scheduler_namespace.py',
                namespace=turtlesim_plus_namespace_list[idx] + '/' + turtle_name_list[i],
                parameters=[{
                    'via_point_file': via_point_dir,
                    'via_point_name': via_point_name_list[i - 1],
                    'max_turtle': len(turtle_name_list) - 1
                }],
                remappings=[
                    ('/spawn_pizza', '/' + turtlesim_plus_namespace_list[idx] + '/spawn_pizza')
                ]
            )
        launch_description.add_action(spawn_turtle)
        launch_description.add_action(controller)
        launch_description.add_action(scheduler)

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        sys.exit()


if __name__ == "__main__":
    main()