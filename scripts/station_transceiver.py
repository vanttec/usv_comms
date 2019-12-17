#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice

#****************************************************************************************#
# Replace with the serial port where your local module is connected to.
PORT = "/dev/ttyUSB0" #La estacion
# Replace with the baud rate of your local module.
BAUD_RATE = 9600
# El nodo XBee con el que se quiere comunicar.
REMOTE_NODE_ID = "vtecboat"
# Frecuencia en Hz a la que se va a correr el nodo.
ROS_RATE = 100
#****************************************************************************************#

class XbeeStation:
    def __init__(self):
        self.boat_data_pub = rospy.Publisher('/usv_comms/station_transceiver/boat_data', String, queue_size=10)

def main():
    rospy.init_node('station_transceiver', anonymous=True)
    rate = rospy.Rate(ROS_RATE) # 100hz
    
    print(" +--------------------------------------+")
    print(" |                Station               |")
    print(" +--------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)
    xbee_node = XbeeStation()
    
    try:
        device.open()
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
        if remote_device is None:
            print("Could not find the remote device")
            exit(1)

        print("Waiting conversation...\n")
        
        #Variable to stop the conversation 
        commActive = True

        while not rospy.is_shutdown() and commActive:
            action = input("Command: ")
            device.send_data_async(remote_device, action)
            time.sleep(1)
            xbee_message = device.read_data()

            if xbee_message is not None:
                #Print the message and 
                message = xbee_message.data.decode()
                print("Received Message: " , message)
                xbee_node.boat_data_pub.publish(str(message))

            #print("Success")
            if action == "exit":
                commActive = False

            #rospy.spinOnce()
            rate.sleep()

    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass