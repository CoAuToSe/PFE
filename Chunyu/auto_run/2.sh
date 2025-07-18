#!/bin/bash

# Function to call a ROS2 service
call_service() {
    ros2 service call /tello_action tello_msgs/TelloAction "{cmd: '$1'}"
}

# Source ROS2 and workspace setup files
source /home/chunyu/ros2_foxy/ros2-linux/setup.bash
cd /home/chunyu/tello_ros_ws
source install/setup.bash

# Define the function to record flight data
record_flight_data() {
    # Record flight data for a specified duration
    local duration=$1
    local output_file=$2
    timeout "$duration" ros2 topic echo /flight_data | tee "$output_file"
}

f=1

record_flight_data 5 tl_initial.txt

# Takeoff & Land
b=5
for ((i=0; i<b; i++)); do
    call_service "takeoff"
    sleep 10
    call_service "land"
    sleep 10
done

record_flight_data 5 tl_final.txt


