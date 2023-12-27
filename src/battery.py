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
        rate = rospy.Rate(1)
        
        min_voltage = 6.8
        max_voltage = 8.4
        delta_voltage = max_voltage - min_voltage

        # Loop until disconnected    
        while not rospy.is_shutdown():
            # Create message
            msg = BatteryState()
            msg.voltage = redboard.adc0
            msg.percentage = ((msg.voltage - min_voltage) / delta_voltage)

            rospy.loginfo(msg)
            pub.publish(msg)

            rate.sleep()
        
        print("Exiting, controller disconnected.")

    except rospy.ROSInterruptException:
        pass