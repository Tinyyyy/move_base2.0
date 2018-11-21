#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import nav_msgs.msg
import genpy
import std_msgs.msg
import math
from PIL import Image
import math
import numpy as np
import time
import roslib
import tf
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2


def postpoints(p):
    return [(x,y) for x in range(p[0]-1,p[0]+2) for y in range(p[1]-1,p[1]+2) if img[x][y]==254 and (x,y)!=p]
    # l=[(p[0] - 1, p[1]), (p[0] + 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1)]
    # return [a for a in l if img[a]==254]

s1=time.time()
im = Image.open("map2.pgm")

img=np.array(im)
img[img<205]=0
img[img>205]=254

pub_map=img.copy()
pub_map[pub_map==0]=100
pub_map[pub_map==205]=50
pub_map[pub_map==254]=0
pub_map=pub_map[::-1].reshape(pub_map.size).tolist()


point=geometry_msgs.msg.Point(-0,-0,0)
quat=geometry_msgs.msg.Quaternion(0,0,0,1)
origin=geometry_msgs.msg.Pose(point,quat)


map=nav_msgs.msg.OccupancyGrid()
map.data=[int(x) for x in pub_map]


def talker():
    pub = rospy.Publisher('map', nav_msgs.msg.OccupancyGrid, queue_size=10)
    pub1 = rospy.Publisher('obstacles_cloud', sensor_msgs.msg.PointCloud2, queue_size=10)
    pub2 = rospy.Publisher('ground_cloud', sensor_msgs.msg.PointCloud2, queue_size=10)
    br = tf.TransformBroadcaster()
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # br.sendTransform((3, 3, 0),
        #                  tf.transformations.quaternion_from_euler(0, 0, math.pi / 4),
        #                  rospy.Time.now(),
        #                  'base_footprint',
        #                  "map")
        map_info = nav_msgs.msg.MapMetaData(rospy.get_rostime(), 0.05, img.shape[1], img.shape[0], origin)
        map.info = map_info
        map.header.stamp=rospy.get_rostime()
        map.header.frame_id = 'map'
        rospy.loginfo("finished")
        pub.publish(map)
        print (map.info.resolution)
        p=[[8,y,0] for y in np.linspace(3,4,20)]
        pointcld=pc2.create_cloud_xyz32(header=std_msgs.msg.Header(frame_id='map'),points=p)
        # pub1.publish(pointcld)
        p = [[x, y, 0] for y in np.linspace(2, 4, 40) for x in np.linspace(3,5,40)]
        pointcld = pc2.create_cloud_xyz32(header=std_msgs.msg.Header(frame_id='map'), points=p)
        # pub2.publish(pointcld)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

