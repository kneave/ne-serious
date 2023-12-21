#!/usr/bin/env python3
from time import sleep

import rospy
from sensor_msgs.msg import BatteryState

import board
import redboard

redboard = redboard.RedBoard()

pub = rospy.Publisher('power', BatteryState, queue_size=10)
rospy.init_node('battery_monitor_node', anonymous=True)

if __name__ == '__main__':
    try:
        # Loop until disconnected
        while True:
            # Create message
            msg = BatteryState()
            msg.voltage = redboard.adc0
            msg.location = ""
            rospy.loginfo(msg)
            pub.publish(msg)

            sleep(1)
        
        print("Exiting, controller disconnected.")

    except rospy.ROSInterruptException:
        pass