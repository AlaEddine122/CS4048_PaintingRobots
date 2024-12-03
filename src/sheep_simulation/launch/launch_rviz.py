import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


# Launch simulation only
def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('sheep_simulation')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_share_dir, 'sheep_simulation_config.rviz')]
        )
    ])