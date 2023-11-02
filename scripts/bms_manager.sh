#!/bin/bash



sudo bash -c "export ROS_MASTER_URI=$ROS_MASTER_URI &&  source /opt/ros/noetic/setup.bash && source /home/ubuntu/bms_manager_ws/devel/setup.bash && rosrun bms_manager bms_manager_node.py "
