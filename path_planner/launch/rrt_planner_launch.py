import launch
import launch_ros


def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(package='rrt',
                                executable='rrt',
                                output='both',
                                emulate_tty=True),
    ])
