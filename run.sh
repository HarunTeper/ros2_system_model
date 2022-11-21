#!/bin/bash

#Script for running some setups on isolated Cores on a Linux with preempt-rt patches.

source install/setup.bash


nohup taskset 0x8 perf record -o perf.1 ros2 run system_model system_composition -system  ~/ros_system_model/src/system_model/config/empty2.yaml >/dev/null 2>&1 &
nohup taskset 0x4 perf record -o perf.2 ros2 run system_model system_composition -system  ~/ros_system_model/src/system_model/config/empty3.yaml >/dev/null 2>&1 &

#capture Data packages
#sudo tcpdump -i any -X udp portrange 7401-7500

#capture discovery packages
#sudo tcpdump -i any -X udp portrange 7400
