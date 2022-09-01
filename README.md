# ros_system_model

Build the packages

	colcon build --symlink-install

Source all packages

	source install/setup.bash

Launch the system

	ros2 launch system_model launch_system.launch.py system_model:=$CONFIG_FILE_NAME

If you use the launch file without a specified config file, it will use the empty.yaml config file

	ros2 launch system_model launch_system.launch.py

Adjust the systems using the config files

	TODO Overview of all node types

