import launch
import launch_ros


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="py_rrt",
                executable="python_rrt",
                output="both",
                emulate_tty=True,
            ),
        ]
    )
