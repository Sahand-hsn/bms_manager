# bms_manager



## to bring up the CAN interface at startup: 
the following file is created "/etc/systemd/network/80-can.network " 
that has the following content inside it.


	[Match]
	Name=can0
	[CAN]
	BitRate=250K
	RestartSec=100ms


## To run a lead strip test: 


## led_strip needs sudo to run: 
the script has to run on start up to start the node that query the bms data, updates the led strip and published the bms_status to ROS. 




bms_manager/scripts/bms_manager.sh



Using the robot_upstart does not work since we need sudo. 
