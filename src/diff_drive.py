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

tilt = 0.4
boom = -0.2
top_fork = 0.2
bottom_fork = 0.4

motor0 = 0
motor1 = 0

rb.m0_invert = False         # front left 
rb.m1_invert = False        # front right

rb.s7_config = 900, 2150
rb.s8_config = 900, 2150

cam_tilt_servo = rb.s7
boom_servo = rb.s8
top_fork_servo = rb.s9
bottom_fork_servo = rb.s10

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
    global last_msg_received, tilt, boom, top_fork, bottom_fork
    rospy.loginfo(rospy.get_caller_id() + 'RCVD: %s', data)
    last_msg_received = rospy.Time.now()

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

    tilt_diff = scaleinput(data.axes[3], False, 40)  
    tilt = limitinput(tilt + tilt_diff)   

    boom_diff = scaleinput(data.axes[7], False, 40)
    boom = limitinput(boom + boom_diff)

    top_fork_diff = scaleinput(data.axes[5], False, 40)
    top_fork = limitinput(top_fork + top_fork_diff)

    bottom_fork_diff = scaleinput(data.axes[9], False, 40)
    bottom_fork = limitinput(bottom_fork + bottom_fork_diff)

    rb.s7 = tilt
    rb.s8 = boom
    rb.s9 = top_fork
    rb.s10 = bottom_fork


def setmotors(m1, m2):
    rb.m0 = m1
    rb.m1 = m2

def limitinput(input):
    if(input > 1):
        input = 1
    elif(input < -1):
        input = -1
    
    return input

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

    rb.s7 = None
    rb.s8 = None
    rb.s9 = None
    rb.s10 = None

if __name__ == '__main__':
    print("Motor node listening...")
    listener()