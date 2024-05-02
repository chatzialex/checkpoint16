from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    eight_trajectory_node = Node(
        package="eight_trajectory",
        executable="eight_trajectory",
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
        eight_trajectory_node,
        kinematic_model_node
    ])