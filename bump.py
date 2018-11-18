#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import nav_msgs.msg
import genpy
import std_msgs.msg
import visualization_msgs.msg
import kobuki_msgs.msg
import sensor_msgs.msg
from PIL import Image
import math
import numpy as np
import time
import roslib
import tf
import Queue
def cb1(T):
    print("bump:",T)
def cb2(T):
    print("cliff:",T)
def cb3(T):
    print("drop:",T)
def cb4(T):
    print("imu:",T)

if __name__ == '__main__':
    rospy.init_node('mobile', anonymous=True)
    rospy.Subscriber("mobile_base/events/bumper", kobuki_msgs.msg.BumperEvent, cb1)
    rospy.Subscriber("mobile_base/events/cliff", kobuki_msgs.msg.CliffEvent, cb2)
    rospy.Subscriber("mobile_base/events/wheel_drop", kobuki_msgs.msg.WheelDropEvent, cb3)
    rospy.Subscriber("mobile_base/sensors/imu_data", sensor_msgs.msg.Imu, cb4)
    while 1:
        i=0


