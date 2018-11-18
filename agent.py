#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import nav_msgs.msg
import genpy
import std_msgs.msg
import visualization_msgs.msg

from PIL import Image
import math
import numpy as np
import time
import roslib
import tf

def cb(T,buf):
    if len(buf) == 0:
        buf.append(T)
    elif len(buf) == 1:
        buf.pop()
        buf.append(T)
if __name__ == '__main__':
    rospy.init_node('mobile', anonymous=True)
    buf=[]
    rospy.Subscriber("mobile_base/commands/velocity",  geometry_msgs.msg.Twist, cb, buf)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10hz
    x=3
    y=3
    w=0
    t=0.01
    while not rospy.is_shutdown():
        br.sendTransform((x, y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, w),
                         rospy.Time.now(),
                         'base_footprint',
                         "map")
        if len(buf) == 1:
            vx=buf[0].linear.x
            vw=buf[0].angular.z
            if vw!=0:
                x=x+vx/vw*(math.sin(w+vw*t)-math.sin(w))
                y=y+vx/vw*(-math.cos(w+vw*t)+math.cos(w))
            else:
                x=x+vx*t*math.cos(w)
                y=y+vx*t*math.sin(w)
            w=w+vw*t
        time.sleep(t)

