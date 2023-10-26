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
from rpi_ws281x import ws, PixelStrip, Adafruit_NeoPixel, Color
class BMSLedStrip(): 
    def __init__(self):
        ### SETUP OF LED STRIP
        LED_2_COUNT = 54        # Number of LED pixels.
        LED_2_PIN = 18          # GPIO pin connected to the pixels (must support PWM! GPIO 13 or 18 on RPi 3).
        #LED_2_PIN = 10          # GPIO 10 uses SPI this can run without sudo 
        LED_2_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
        LED_2_DMA = 10          # DMA channel to use for generating signal (Between 1 and 14)
        LED_2_BRIGHTNESS = 30  # Set to 0 for darkest and 255 for brightest
        LED_2_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
        LED_2_CHANNEL = 0       # 0 or 1
        LED_2_STRIP = ws.WS2811_STRIP_GBR

        self.strip = PixelStrip(LED_2_COUNT, LED_2_PIN, LED_2_FREQ_HZ,
                                LED_2_DMA, LED_2_INVERT, LED_2_BRIGHTNESS,
                                LED_2_CHANNEL, LED_2_STRIP)
        self.strip.begin()
        self.update(9)
        time.sleep(2)
        self.colorWipe()
        self.config()

    def config(self):
        pass


    def colorWipe(self, color= Color(0, 0, 0), wait_ms=5):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def update(self, strip_part_On):
        try:
            for i in range(self.strip.numPixels()):
                if i <= strip_part_On:
                    shade_g = int(255*strip_part_On/9)
                    shade_r = int(-255*strip_part_On/9 + 255)
                    self.strip.setPixelColor(i, Color(shade_g,shade_r,0))
                    time.sleep(1 / 1000.0)
                else:
                    self.strip.setPixelColor(i, Color(0,0,0))
                    time.sleep(1 / 1000.0)
            self.strip.show()
        except KeyboardInterrupt:
            self.colorWipe()



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
            raise can.CanError("CAN Error, Message could NOT get sent. Interface can0 not available.")
        try:
            await asyncio.wait_for(reader.get_message(), 10)
        except asyncio.TimeoutError: 
            raise TimeoutError("CAN Timeout, BMS not reachable!")
        notifier.stop()

def update_bms():
    asyncio.run(main(0x90))


def init_node():
    global bms_SOC
    pub = rospy.Publisher('bms_status/SOC', std_msgs.msg.Float32, queue_size=10)
    rospy.init_node('bms_status')
    rate = rospy.Rate(0.1) # every 10 seconds
    led_strip = BMSLedStrip() 
    while not rospy.is_shutdown():
        try: 
            update_bms()
            SOC_msg = "SOC level is {}%".format(bms_SOC)
            rospy.loginfo(SOC_msg)
            pub.publish(round(bms_SOC,1))
        except TimeoutError as e:
            rospy.logerr(e)
        except can.CanError as e:
            rospy.logerr(e)
        led_strip.update(int( bms_SOC / 100 * 9)) #TODO
        rate.sleep()


if __name__ == '__main__':
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass

