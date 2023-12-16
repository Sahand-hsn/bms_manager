# bms_manager



## changes for the CAN ifup on startup: 

created: 
' /etc/systemd/network/80-can.network '


	[Match]
	Name=can0
	[CAN]
	BitRate=250K
	RestartSec=100ms



## led_strip needs sudo to run: 

	 
