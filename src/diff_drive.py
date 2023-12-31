#!/usr/bin/env python3

import rospy
import redboard
import math
import time

from sensor_msgs.msg import Joy

# time in seconds in not recieving a message before stopping
msg_timeout = rospy.Duration(0.5)
last_msg_received = rospy.Time.from_sec(time.time())

rb = redboard.RedBoard()
tilt = 0.5
motor0 = 0
motor1 = 0

rb.m0_invert = False         # front left 
rb.m1_invert = False        # front right
rb.s7_config = 750, 1950

# from https://electronics.stackexchange.com/questions/19669/algorithm-for-mixing-2-axis-analog-input-to-control-a-differential-motor-drive
def steering(x, y):
    # convert to polar
    r = math.hypot(x, y)
    t = math.atan2(y, x)

    # rotate by 45 degrees
    t -= math.pi / 4

    # back to cartesian
    left = r * math.sin(t)
    right = r * math.cos(t)

    # rescale the new coords
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)

    # clamp to -1/+1
    left = max(-1, min(left, 1))
    right = max(-1, min(right, 1))

    return left, right

def scaleinput(input, invert, scale):
    if scale != 0:
        scaled = input / scale
    else:
        scaled = input

    if(invert):
        scaled = scaled * -1

    if scaled == -0.0:
        scaled = 0

    return scaled

def callback(data):
    global last_msg_received, tilt
    # rospy.loginfo(rospy.get_caller_id() + 'RCVD: %s', data)
    last_msg_received = rospy.Time.now()

    control_movement = data.buttons[2] == 1
    high_speed = data.buttons[4] == 0
    x, y = data.axes[0], data.axes[1]

    xbox = True

    # print(data.axes[0], data.axes[1])
    if xbox == True:
        right, left = steering(x, y)
    else:
        left, right = steering(x, y)

    # Buttons are on when down so this makes sense in the physical world
    if(high_speed == True):
        # Low speed, halve values
        left = left * 0.5
        right = right * 0.5
    else:
        left = left
        right = right

    setmotors(left, right)

    tilt_diff = scaleinput(data.axes[4], False, 40)  
    tilt += tilt_diff   

    if(tilt > 1):
        tilt = 1
    elif(tilt < -1):
        tilt = -1
    
    print(tilt)
    rb.s7 = tilt


def setmotors(m1, m2):
    rb.m0 = m1
    rb.m1 = m2

def timeout_check(event):
    timediff = rospy.Time.now() - last_msg_received
    if(timediff > msg_timeout):
        print("Timed out: " + str(rospy.Time.now()) + ", " + str(last_msg_received) + ", " + str(timediff))
        setmotors(0, 0)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # Set up the timer-based timeout check
    rospy.Timer(msg_timeout, timeout_check)

    rospy.spin()

if __name__ == '__main__':
    print("Motor node listening...")
    listener()