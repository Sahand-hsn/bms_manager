#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import String
#from bms_led import BMSLedStrip
#import importlib
#importlib.reload(BMSLedStrip)


import asyncio
from typing import List
import can
from can.notifier import MessageRecipient
import std_msgs
from math import floor

import time
import neopixel
import board

def update_strip(strip, strip_part_On):
    try:
        for i in range(12):
            if i <= strip_part_On:
                shade_g = int(255*strip_part_On/12)
                shade_r = int(-255*strip_part_On/12 + 255)
                strip[-i-1] = (0,shade_r,shade_g) #BRG
                #self.strip.setPixelColor(i, Color(255,0,0))
            else:
                strip[-i-1] = (0,0,0)
    except KeyboardInterrupt:
        strip.fill((0,0,0))


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


async def update_can(d_id):
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
            raise can.CanError("CAN Error, Message could NOT get sent. Interface can0 not available.")
        try:
            await asyncio.wait_for(reader.get_message(), 10)
        except asyncio.TimeoutError: 
            raise TimeoutError("CAN Timeout, BMS not reachable!")
        notifier.stop()

def update_bms():
    asyncio.run(update_can(0x90))


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('bms_status/SOC', std_msgs.msg.Float32, queue_size=10)
        rospy.init_node('bms_manager_node', log_level=rospy.DEBUG)
        rospy.loginfo("bms_node_node started")
        rate = rospy.Rate(0.1) # every 10 seconds
        led_strip = neopixel.NeoPixel(board.D21, 12)
        while not rospy.is_shutdown():
            try: 
                update_bms()
            except TimeoutError as e:
                rospy.logerr(e)
            except can.CanError as e:
                rospy.logerr(e)
            SOC_msg = "SOC is {}%".format(bms_SOC)
            rospy.logdebug(SOC_msg)
            pub.publish(round(bms_SOC,1))
            update_strip(led_strip, int( bms_SOC / 100 * 12))  
            rate.sleep()
        if rospy.is_shutdown(): 
            led_strip.fill((0,0,0))
    except: 
        rospy.logerr("bms_manager_node failed to start")

