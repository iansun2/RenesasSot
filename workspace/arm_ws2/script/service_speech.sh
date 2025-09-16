#/bin/bash
#source /opt/ros/humble/setup.bash
#ros2 topic pub --once /speech_recognition std_msgs/msg/Int32 "{data: 2}"



# Source the ROS 2 setup file to make ROS commands available
source /opt/ros/humble/setup.bash

# ros2 topic pub --once /speech_recognition std_msgs/msg/Int32 "{data: 5}"
# sleep 10

echo "Publishing message 2..."
ros2 topic pub --once /speech_recognition std_msgs/msg/Int32 "{data: 2}"

# Wait for user input before proceeding
read -p "Press Enter to publish the next message..."

echo "Publishing message 3..."
ros2 topic pub --once /speech_recognition std_msgs/msg/Int32 "{data: 3}"

read -p "Press Enter to publish the next message..."

echo "Publishing message 4..."
ros2 topic pub --once /speech_recognition std_msgs/msg/Int32 "{data: 4}"

read -p "Press Enter to publish the next message..."

echo "Publishing message 5..."
ros2 topic pub --once /speech_recognition std_msgs/msg/Int32 "{data: 5}"

echo "All messages published."
