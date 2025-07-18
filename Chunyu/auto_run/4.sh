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
    local duration=$1
    local output_file=$2
    timeout "$duration" ros2 topic echo /flight_data | tee "$output_file"
}

# Function to perform the movement along the x-axis
move_x_axis() {
    local duration=$1
    local v=$2
    
    local f=1
    local n=$(echo "$duration / $f" | bc)
    
    record_flight_data 5 "fb_initial_${v}_${duration}.txt"
    
	for ((j=0; j<n; j++)); do
	    call_service "rc $v 0 0 0"
	    sleep $f
	done

    call_service "rc 0 0 0 0"
    
    record_flight_data 5 "fb_final_${v}_${duration}.txt"
    
}

v=-100

call_service "takeoff"

# Perform movements and record data
move_x_axis 10 $v

# Land
call_service "land"
