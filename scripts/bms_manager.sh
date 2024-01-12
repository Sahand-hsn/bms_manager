#!/bin/bash

sudo bash -c "export ROS_MASTER_URI=http://localhost:11311 && source /opt/ros/noetic/setup.bash && roscore &"

sudo bash -c "export PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages:/home/rosmatch/.local/lib/python3.8/site-packages:/usr/local/lib/python3.8/dist-packages' && export ROS_MASTER_URI=http://localhost:11311 && source /opt/ros/noetic/setup.bash && python3 /home/rosmatch/catkin_ws/src/bms_manager/scripts/bms_manager_node.py"

#sudo bash -c "export ROS_MASTER_URI=$ROS_MASTER_URI &&  source /opt/ros/noetic/setup.bash && source /home/ubuntu/bms_manager_ws/devel/setup.bash && rosrun bms_manager bms_manager_node.py "
