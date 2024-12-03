import launch
import launch_ros.actions


# Launch simulation and rviz2 with preset configuration
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/root/ros2_ws/CS4048_SheepHerding/src/sheep_simulation/config/sheep_simulation_config.rviz']
        ),
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