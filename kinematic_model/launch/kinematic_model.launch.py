from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    wheel_velocities_publisher_node = Node(
        package="wheel_velocities_publisher",
        executable="wheel_velocities_publisher",
        output="screen",
        parameters = [{"use_sim_time" : True}],
        emulate_tty=True
    )

    kinematic_model_node = Node(
        package="kinematic_model",
        executable="kinematic_model",
        output="screen",
        parameters = [{"use_sim_time" : True}],
        emulate_tty=True
    )

    return LaunchDescription([
        wheel_velocities_publisher_node,
        kinematic_model_node
    ])