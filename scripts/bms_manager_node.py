#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import bms_led

import asyncio
from typing import List
import can
from can.notifier import MessageRecipient


can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 250000
P = 0x18 
addres = 0x0340 
bms_SOC = 0

def message_callback(msg: can.Message) -> None:
    """Regular callback function. Can also be a coroutine."""
    data_ID = hex(msg.arbitration_id)[4:6] 
    global bms_SOC
    if data_ID == "90":
        Cumulative_total_voltage = int(msg.data[0:2].hex(),16)/10
        Gather_total_voltage = int(msg.data[2:4].hex(),16)/10
        Current = int(msg.data[4:6].hex(),16)/10 - 3000
        bms_SOC = int(msg.data[6:8].hex(),16)/10
        # print("Cumulative_total_voltage {}V".format(Cumulative_total_voltage)) 
        # print("Gather_total_voltage {}V".format(Gather_total_voltage))
        # print("Current {}A".format(Current))
        # print("SOC {}%".format(bms_SOC))


async def main(d_id):
    with can.Bus() as bus:
        reader = can.AsyncBufferedReader()
        logger = can.Logger("logfile.asc")

        listeners: List[MessageRecipient] = [
            message_callback,  # Callback function
            reader,  # AsyncBufferedReader() listener
            logger,  # Regular Listener object
        ]
        # Create Notifier with an explicit loop to use for scheduling of callbacks
        loop = asyncio.get_running_loop()
        notifier = can.Notifier(bus, listeners, loop=loop)
        # Start sending first message
        a1 = (P << 8) | (d_id)
        can_id = (a1 << 16 ) | addres
        msg_content = []
        try:
            bus.send(can.Message(arbitration_id=can_id, data=msg_content, is_extended_id=True)) 
        except can.CanError:
            print("Message NOT sent")
        await reader.get_message()
        notifier.stop()

def update_bms():
    asyncio.run(main(0x90))


def init_node():
    pub = rospy.Publisher('bms_status', String, queue_size=10)
    rospy.init_node('bms_status')
    rate = rospy.Rate(0.1) # every 10 seconds 
    #led_strip = bms_led.BMSLedStrip() 
    while not rospy.is_shutdown():
        update_bms()
        SOC_msg = "SOC level is {}%".format(bms_SOC)
        rospy.loginfo(SOC_msg)
        pub.publish(SOC_msg)
        #led_strip.update(int( bms_SOC / 100 * 9))
        rate.sleep()

if __name__ == '__main__':
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass
        