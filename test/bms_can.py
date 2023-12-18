
import asyncio
from typing import List

import can
from can.notifier import MessageRecipient

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 250000

class BMS_CAN:
    def __init__(self): 
        self.P = 0x18 
        self.addres = 0x0340
        # try:
        #     self.bus = can.Bus()
        # except can.CanError:
        #     print("CAN initialisation NOT successful")
        #     self.bus.shutdown()
        self.SOC_data = 0

    def message_callback(self, msg: can.Message) -> None:
        """Regular callback function. Can also be a coroutine."""
        data_ID = hex(msg.arbitration_id)[4:6] 
        if data_ID == "91":
            Max_cell_voltage = int(msg.data[0:2].hex(),16)/1000
            No_cell_max_voltage = msg.data[2]
            Minimum_cell_voltage = int(msg.data[3:5].hex(),16)/1000
            No_cell_min_voltage = msg.data[5]
            print("Battery with: {} cells with Max_cell_voltage {}V".format(No_cell_max_voltage,Max_cell_voltage)) 
            print("Battery with: {} cells with Min_cell_voltage {}V".format(No_cell_min_voltage,Minimum_cell_voltage))
        if data_ID == "90":
            Cumulative_total_voltage = int(msg.data[0:2].hex(),16)/10
            Gather_total_voltage = int(msg.data[2:4].hex(),16)/10
            Current = int(msg.data[4:6].hex(),16)/10 - 3000
            self.SOC_data = int(msg.data[6:8].hex(),16)/10
            print("Cumulative_total_voltage {}V".format(Cumulative_total_voltage)) 
            print("Gather_total_voltage {}V".format(Gather_total_voltage))
            print("Current {}A".format(Current))
            print("SOC {}%".format(SOC))

    async def asy_update(self, d_id):
        print("asyn called!")
        with can.Bus() as bus:
            reader = can.AsyncBufferedReader()
            logger = can.Logger("logfile.asc")
            print("bus created!")
            listeners: List[MessageRecipient] = [
                self.message_callback,  # Callback function
                reader,  # AsyncBufferedReader() listener
                logger,  # Regular Listener object
            ]
            # Create Notifier with an explicit loop to use for scheduling of callbacks
            loop = asyncio.get_running_loop()
            notifier = can.Notifier(bus, listeners, loop=loop)
            # Start sending first message
            a1 = (self.P << 8) | (d_id)
            can_id = (a1 << 16 ) | self.addres
            msg_content = []
            try:
                bus.send(can.Message(arbitration_id=can_id, data=msg_content, is_extended_id=True)) 
            except can.CanError:
                print("Message NOT sent")
                #bus.shutdown()
            # Wait for next message from AsyncBufferedReader
            # msg = await reader.get_message()
            # Wait for last message to arrive
            await reader.get_message()
            # Clean-up
            notifier.stop()
    def update():
        asyncio.run(self.asy_update(0x90))



if __name__ == "__main__":
    try:
        can1 = BMS_CAN() 
        can1.update(0x90) 
    except: 
        print("can failed")
