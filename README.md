# CS4048_SheepHerding


[Google Doc](https://docs.google.com/document/d/1pJ0U8vCnQ_odnONFk349ONtStWuJV8kWYHxC4bvhFdw/edit?usp=sharing)


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
ros2 launch sheep_simulation launch_sim.py // runs simulation only
```