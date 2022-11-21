
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

You can use the following executor types:

	single_threaded_executor
	static_single_threaded_executor
	multi_threaded_executor

You can use the following node types:

	filter # receives and sends messages with subscriber
		name # node name
		wcet # subscription callback worst-case execution time in milliseconds
		subscription_topic # subscription topic
		publisher topic # publisher topic
		subscription_topic_buffer_size # subscription buffer size
		publisher_buffer_size # publisher buffer size
	sensor # creates and sends a message using a timer and publisher
		name # node name
		period # period in milliseconds
		wcet # timer callback worst-case execution time in nanoseconds
		publisher_topic # publisher topic
		publisher_buffer_size # publisher buffer size
	subscription_actuator # receives a message and processes it
		name # node name
		wcet # subscription callback worst-case execution time
		subscription_topic # subscription topic
		subscription_topic_buffer_size # subscription buffer size

Please note that that all arguments need to be provided for all nodes.
The systems can be specified using .yaml files in the config folder.
They have the following structure:

	executor_type_1:
		node_type_1:
			argument_type_1:
			argument_type_2:
			...
		node_type_2:
			argument_type_1:
			argument_type_2:
	executor_type_2:
		node_type_1:
			argument_type_1:
			argument_type_2:
			...
		node_type_2:
			argument_type_1:
			argument_type_2:

The script will create an executor with all specified nodes. After that , it launches one thread per specified executor to run the systems.

## Addition for running on isolated Cores
Install and build a custom Linux kernel with preempt-rt patches, following this [Repo](https://github.com/nilhoel1/preempt_rt_scripts_ROS2). Also setup coresponding kernel parameters.

When installed use the run.sh script as reference for running ROS2 instances on isolated cores. The script runs two ROS2 executors one on CPU2 and one on CPU3.
