import launch
import launch_ros


def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(package='rrt_v2',
                                executable='rrt_v2',
                                output='both',
                                emulate_tty=True),
    ])
