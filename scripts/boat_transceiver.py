#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from digi.xbee.devices import XBeeDevice

#****************************************************************************************#
# Replace with the serial port where your local module is connected to.
PORT = "/dev/ttyUSB0"
# Replace with the baud rate of your local module.
BAUD_RATE = 9600
# El nodo XBee con el que se quiere comunicar.
REMOTE_NODE_ID = "vtecstation"
# Frecuencia en Hz a la que se va a correr el nodo.
ROS_RATE = 100
#****************************************************************************************#

class XbeeBoat:
    def __init__(self):
        
        self._mon_sub = rospy.Subscriber("data_input", String, self.data_callback)
        
        self.course_pub = rospy.Publisher("/usv_comms/boat_transceiver/course_config", String, queue_size=10)
        self.start_pub = rospy.Publisher("/usv_comms/boat_transceiver/start_mission", Empty, queue_size=10)
        self.stop_pub = rospy.Publisher("/usv_comms/boat_transceiver/stop_mission", Empty, queue_size=10)

        self.empty_msg = Empty()

    def data_callback(self, _data):
        self.boat_data = _data
        print('Sending data: ', self.boat_data)
        #rospy.logwarn(self.powerL)

def main():
    rospy.init_node('boat_transceiver', anonymous=True)
    rate = rospy.Rate(ROS_RATE) # 100hz
    
    print(" +-------------------------------------------------+")
    print(" |                       Boat                      |")
    print(" +-------------------------------------------------+\n")
    
    device = XBeeDevice(PORT, BAUD_RATE)
    xbee_node = XbeeBoat()

    try:
        device.open()
        device.flush_queues()
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
        if remote_device is None:
            print("Could not find the remote device")
            exit(1)
        
        print("Waiting conversation...\n")
        
        #Variable to stop the conversation 
        commActive = True

        while not rospy.is_shutdown() and commActive:

            #Read data and chek if something has been received 
            xbee_message = device.read_data()
            if xbee_message is not None:
                
                #Decode and print the message 
                message = xbee_message.data.decode()
                print("Received Message: " , message)

                if message[0] == 'c':
                    xbee_node.course_pub.publish(message[1])
                    device.send_data_async(remote_device, xbee_node.boat_data)
                elif message == 's':
                    xbee_node.start_mission.publish(xbee_node.empty_msg)
                    device.send_data_async(remote_device, 'Starting mission...')
                elif message == 'k':
                    xbee_node.stop_mission.publish(xbee_node.empty_msg)
                    device.send_data_async(remote_device, 'Stopping mission.')
                elif message == 'exit':
                    commActive = False

            # Send the monitoring data to station
            device.send_data_async(remote_device, xbee_node.boat_data)
            
            rospy.spinOnce()
            rate.sleep()

    #If the device is not closed, close it.
    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
