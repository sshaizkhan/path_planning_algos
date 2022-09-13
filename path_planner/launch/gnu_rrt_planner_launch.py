import launch
import launch_ros


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gnu_rrt_plotting",
                executable="gnu_rrt_plotting",
                output="both",
                emulate_tty=True,
            ),
        ]
    )
