# vikings_bot_map_server

<hr>

### Description
This package contains `vikings_bot` project map server related files and allows to publish map.
*To create the map, refer to `vikings_bot_cartographer_slam` package*

<hr>

## Installation

Download source and install dependencies:
```
cd <path/to/your/ros_ws>
git clone git@github.com:Hercogs/vikings_bot_map_server.git src/vikings_bot_map_server
rosdep update
rosdep install --ignore-src --default-yes --from-path src
```

Build package:
```
colcon build
source install/setup.bash
```

<hr>

### Usage

To publish map:
```ros2 launch vikings_bot_map_server map_server.launch.py```
#### Parameters:
- `vikings_bot_name`: namespace of robot - [vikings_bot_1 or vikings_bot_2] -> *string*, default *-*
- `map_file`: map file name -> *string*, default *simulation_map.yaml*
- `use_rviz`: whether to automatically open `rviz2` -> *bool*, default *false*
- `use_sim`: whether to use simulated or real robot -> *bool*, default *true*



