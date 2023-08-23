#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import bms_can
import bms_led

bms_status_SOC = 0

def init_node():
    pub = rospy.Publisher('bms_status', String, queue_size=10)
    rospy.init_node('bms_status')
    rate = rospy.Rate(0.2) # every 5 seconds 
    try:
        self.can = bms_can.BMS_CAN()
    except: 
        print("can init failed")
    while not rospy.is_shutdown():
        bms_update()
        SOC_msg = "SOC is {}%".format(self.can.data.SOC())
        rospy.loginfo(SOC_msg)
        pub.publish(SOC_msg)
        rate.sleep()

def bms_update():
    self.can.update() 
    #update_LED()


if __name__ == '__main__':
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass