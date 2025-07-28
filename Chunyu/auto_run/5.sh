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

# Function to perform the movement along the z-axis
move_z_axis() {
    local duration=$1
    local v=$2
    
    local f=1
    local n=$(echo "$duration / $f" | bc)
    local d=10

    # Define a function to move in the z-axis
    move() {
        local cycles=$1
        local velocity=$2
        for ((i=0; i<cycles; i++)); do
            call_service "rc 0 0 $velocity 0"
            sleep $f
        done
    }

    record_flight_data 5 "ud_initial_${v}_${duration}.txt"
    
    move $n $v
    move $n $v
    move $n -$v
    move $n -$v

    call_service "rc 0 0 0 0"
    record_flight_data 5 "ud_middle_${v}_${duration}.txt"

    move $n $v
    move $n $v
    move $n $v
    move $n $v
    move $n -$v

    call_service "rc 0 0 0 0"
    record_flight_data 5 "ud_final_${v}_${duration}.txt"
}

v=40

call_service "takeoff"

# Perform movements and record data
move_z_axis 2 $v

# Land
call_service "land"

