import launch
import launch_ros.actions


# Launch simulation only
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='sheep_simulation',
            executable='master_node',
            name='simulation'
        ),
        launch_ros.actions.Node(
            package="sheep_simulation",
            executable="sheep_node",
            name="sheep_node"
        ),
        launch_ros.actions.Node(
            package="sheep_simulation",
            executable="wolf_node",
            name="wolf_node"
        )
    ])