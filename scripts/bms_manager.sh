#!/bin/bash


sudo bash -c "export ROS_MASTER_URI=http://roscore:11311 &&  source /opt/ros/noetic/setup.bash && source /home/ubuntu/bms_manager_ws/devel/setup.bash && python3 /home/ubuntu/bms_manager_ws/src/bms_manager/scripts/bms_manager_node.py"

#sudo bash -c "export ROS_MASTER_URI=$ROS_MASTER_URI &&  source /opt/ros/noetic/setup.bash && source /home/ubuntu/bms_manager_ws/devel/setup.bash && rosrun bms_manager bms_manager_node.py "
