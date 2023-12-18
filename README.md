# bms_manager

## Installation
clone the package into your workspace. 

and to bring up the CAN interface at startup: 
the following file is created "/etc/systemd/network/80-can.network " 
that has the following content inside it.


	[Match]
	Name=can0
	[CAN]
	BitRate=250K
	RestartSec=100ms


## Testing: 
for testing the functionality of the node the script can be executed independantly: 

~> cd bms_manager
~> sudo bms_manager/scripts/bms_manager.sh

## start bms_manager on start up: 
we create a systemd service to start the node on start up. 

make the bms_manager.sh file exacutable: 

~> sudo chmod 744 scripts/bms_manager.sh

## to bring up the bms_manager at startup: 
we create a new file at /etc/systemd/system/bms_manager.service with the following content: 



[Unit]
Description=BMS-manager-node manages the led strip and published bms data in ROS  
Wants=network.target
After=syslog.target network-online.target

[Service]
Type=simple
ExecStart=/home/ubuntu/bms_manager_ws/src/bms_manager/scripts/bms_manager.sh

[Install]
WantedBy=multi-user.target


and then run: 

~> sudo systemctl daemon-reload 
~> sudo systemctl enable bms_manager.service
~> sudo systemctl start bms_manager.service

now the node should start on SOC topic and the led strip should light up.

## To 
the script has to run on start up to start the node that query the bms data, updates the led strip and published the bms_status to ROS. 


Using the robot_upstart does not work since we need sudo. 


