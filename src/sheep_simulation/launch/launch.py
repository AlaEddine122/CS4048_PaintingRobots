import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


# Launch simulation and rviz2 with preset configuration
def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('sheep_simulation')
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_share_dir, 'sheep_simulation_config.rviz')]
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