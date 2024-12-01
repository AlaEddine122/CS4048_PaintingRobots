import launch
import launch_ros.actions


# Launch simulation only
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/root/ros2_ws/CS4048_SheepHerding/src/sheep_simulation/config/sheep_simulation_config.rviz']
        )
    ])