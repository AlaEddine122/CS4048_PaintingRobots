# CS4048_SheepHerding

## Setup

build package
```js
colcon build
```

source installation
```js
source install/local_setup.bash
```

run desired launch file
```js
ros2 launch sheep_simulation launch.py // runs simulation + rviz2 with preloaded config

// development
// run launch files on separate terminals
// simulation terminal can be interrupted and re-launched after building, to run simulation again

ros2 launch sheep_simulation launch_rviz.py // runs rviz2 with preloaded config
ros2 launch sheep_simulation launch_sim.py // runs simulation only
```