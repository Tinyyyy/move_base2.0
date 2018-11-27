#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import nav_msgs.msg
import genpy
import std_msgs.msg
import visualization_msgs.msg
import threading
from PIL import Image
import math
import numpy as np
import time
import roslib
import tf
import signal
import sys
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from global_path_heap import *
import copy
import Queue
import kobuki_msgs.msg


def quit(signum, frame):
    sys.exit()


class Nav:
    def __init__(self):
        self.main_proc()

    def cb_bump(self, b):
        self.event_bump.put((b, time.time()))

    def cb_cliff(self, c):
        if self.event_cliff.full():
            with self.event_cliff.mutex:
                self.event_cliff.queue[0] = c
            return
        self.event_cliff.put(c)

    def cb_drop(self, d):
        if self.event_drop.full():
            with self.event_drop.mutex:
                self.event_drop.queue[0] = d
            return
        self.event_drop.put(d)

    def cb_map(self, map):
        if self.map_buf.full():
            with self.map_buf.mutex:
                self.map_buf.queue[0] = map
            return
        self.map_buf.put(map)

    def cb_goal(self, g):
        rospy.loginfo(rospy.get_caller_id() + "goal heard ")
        if self.goals_buf.full():
            with self.goals_buf.mutex:
                self.goals_buf.queue[0] = g
            return
        self.goals_buf.put(g)

    def cb_pcl(self, pointcld):
        # rospy.loginfo(rospy.get_caller_id() + "pcl heard ")
        l = pc2.read_points(pointcld)
        p = [geometry_msgs.msg.Point32(*x) for x in l]
        pcl = sensor_msgs.msg.PointCloud(header=pointcld.header, points=p)
        if self.pcl.full():
            with self.pcl.mutex:
                self.pcl.queue[0] = pcl
            return
        self.pcl.put(pcl)
        # self.pcl.header.frame_id="base_footprint"
        # print(len(self.pcl))
        # print(self.pcl)

    def cb_pcl_ground(self, pointcld):
        # rospy.loginfo(rospy.get_caller_id() + "pcl heard ")
        l = pc2.read_points(pointcld)
        p = [geometry_msgs.msg.Point32(*x) for x in l]
        pcl = sensor_msgs.msg.PointCloud(header=pointcld.header, points=p)
        if self.pcl_ground.full():
            with self.pcl_ground.mutex:
                self.pcl_ground.queue[0] = pcl
            return
        self.pcl_ground.put(pcl)
        # self.pcl.header.frame_id="base_footprint"
        # print(len(self.pcl))
        # print(self.pcl)

    def stop(self):
        while 1:
            self.lock.acquire()
            if self.get_goal == True:
                print("goal reached!!")
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = 0
                cmd.angular.z = 0
                self.vel.publish(cmd)
                time.sleep(0.05)
            self.lock.release()

    def pub_path(self):
        pub = rospy.Publisher('Path', nav_msgs.msg.Path, queue_size=10)
        pub1 = rospy.Publisher('Pathpose', geometry_msgs.msg.PoseArray, queue_size=10)
        pub2 = rospy.Publisher('Marker', visualization_msgs.msg.Marker, queue_size=10)
        pub3 = rospy.Publisher('robot', visualization_msgs.msg.Marker, queue_size=10)
        # pub4 = rospy.Publisher('pcl_trans', sensor_msgs.msg.PointCloud, queue_size=10)
        while not rospy.is_shutdown():

            # global_ok = self.global_ok
            # pcl=self.pcl
            sim_targets = self.sim_targets

            diameter = 0.3
            M = visualization_msgs.msg.Marker()
            M.color.g = 1
            M.color.a = 1
            M.scale.x = diameter
            M.scale.y = diameter
            M.scale.z = 0.06
            M.pose = geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0, 0, M.scale.z / 2),
                                            orientation=geometry_msgs.msg.Quaternion(0, 0, 0, 1))
            M.header.frame_id = 'base_footprint'
            M.type = visualization_msgs.msg.Marker.CYLINDER
            M.action = visualization_msgs.msg.Marker.ADD
            # M.points = [geometry_msgs.msg.Point(x[0], x[1], 0) for x in self.sim_targets]
            pub3.publish(M)
            # if pcl:
            #     # pointcld = pc2.create_cloud_xyz32(header=std_msgs.msg.Header(frame_id='base_footprint'), points=self.pcl)
            #     pub4.publish(pcl)
            if self.path.full():
                with self.path.mutex:
                    path = self.path.queue[0]
                pub.publish(path)
                pa = geometry_msgs.msg.PoseArray()
                pa.header.frame_id = 'map'
                pa.poses = [i.pose for i in path.poses]
                pub1.publish(pa)
            if sim_targets != None:
                M = visualization_msgs.msg.Marker()
                M.color.r = 1
                M.color.a = 1
                M.scale.x = 0.01
                M.scale.y = 0.01
                M.header.frame_id = 'map'
                M.type = visualization_msgs.msg.Marker.POINTS
                M.action = visualization_msgs.msg.Marker.ADD
                M.points = [geometry_msgs.msg.Point(x[0], x[1], 0) for x in sim_targets]
                pub2.publish(M)

    def abnormal_proc(self):
        while 1:
            if not self.event_abnormal.isSet():
                continue
            if not self.traj.empty():
                with self.traj.mutex:
                    l = list(self.traj.queue)
                for i in l[::-1]:
                    cmd = geometry_msgs.msg.Twist()
                    cmd.linear.x = -i[0]
                    cmd.angular.z = -i[1]
                    self.vel.publish(cmd)
                    time.sleep(0.05)
            self.event_abnormal.clear()

    def dwa(self):
        get_ready = False
        while not rospy.is_shutdown():

            while 1:
                if self.path.full() and self.goals_buf.full():
                    break;
            with self.path.mutex:
                path = self.path.queue[0]
            (trans, qua) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            roat = tf.transformations.euler_from_quaternion(qua)
            # print(trans)

            p = np.array([[i.pose.position.x, i.pose.position.y] for i in path.poses
                          # if ((i.pose.position.x-trans[0])**2+(i.pose.position.y-trans[1])**2)**0.5>0.5
                          ])
            if p.size == 0:
                print("can not reach!")
                continue
            dis = np.sum((p - np.array([trans[0], trans[1]])) ** 2, axis=1)
            min_dis_ind = dis.argmin()
            local_goal = p[min(min_dis_ind + 4, len(p) - 1)]
            local_goal_ori = tf.transformations.euler_from_quaternion(
                    [path.poses[min_dis_ind].pose.orientation.x,
                     path.poses[min_dis_ind].pose.orientation.y,
                     path.poses[min_dis_ind].pose.orientation.z,
                     path.poses[min_dis_ind].pose.orientation.w]
            )[2]
            if ((p[-1][0] - trans[0]) ** 2 + (p[-1][1] - trans[1]) ** 2) ** 0.5 < 0.05:

                # self.get_goal=True
                get_ready = False
                self.goals_buf.get()
                print("goal reached!!")
                i = 0
                while i != 20:
                    i = i + 1
                    cmd = geometry_msgs.msg.Twist()
                    cmd.linear.x = 0
                    cmd.angular.z = 0
                    self.vel.publish(cmd)
                    time.sleep(0.05)
                # self.global_ok = False
                self.path.get()
                continue
            # local_goal = [-1,0]
            # #
            # local_goal_ori=math.atan2(local_goal[1]-trans[1],local_goal[0]-trans[0])

            if not get_ready:
                theta = math.atan2(local_goal[1] - trans[1], local_goal[0] - trans[0])
                # print(theta,roat[2])
                if abs(theta - roat[2]) < math.pi / 12:
                    get_ready = True
                else:
                    cmd = geometry_msgs.msg.Twist()
                    cmd.linear.x = 0
                    cmd.angular.z = 0.4
                    self.vel.publish(cmd)
                    time.sleep(0.05)
                    continue

            max_vel_x = 0.3
            max_vel_w = 0.6
            t = 2.5
            t1 = time.time()
            x_space = np.linspace(0.01, max_vel_x, 20)
            # x_space=[x for x in x_space if abs(x)>0.25]
            w_space = np.linspace(-max_vel_w, max_vel_w, 20)
            xw_space = [(x, w) for x in x_space for w in w_space]

            # targets=[
            #     [
            #         [
            #             trans[0]+x[0]/x[1]*(math.sin(roat[2]+x[1]*tt)-math.sin(roat[2])),
            #             trans[1]+x[0]/x[1]*(-math.cos(roat[2]+x[1]*tt)+math.cos(roat[2]))
            #         ]
            #         for tt in np.arange(0, t, 0.1) ]
            #     for x in xw_space if x[1]!=0]
            targets = [
                [
                    trans[0] + x[0] / x[1] * (math.sin(roat[2] + x[1] * t) - math.sin(roat[2])),
                    trans[1] + x[0] / x[1] * (-math.cos(roat[2] + x[1] * t) + math.cos(roat[2]))
                ]
                for x in xw_space if x[1] != 0]

            self.sim_targets = targets

            targets_w = [roat[2] + x[1] * t
                         for x in xw_space if x[1] != 0]

            targets = np.array([[i[0], i[1]] for i in targets])

            # dis_traj=[(np.sum((np.array(x)-local_goal)**2,axis=1)**0.5).min() for x in targets]

            dis_sim = np.sum((targets - local_goal) ** 2, axis=1) ** 0.5
            t2 = time.time()

            score = dis_sim
            ind = score.argmin()
            (x, w) = xw_space[ind]
            left = -1
            center = -1
            right = -1
            if not self.event_bump.empty():
                l = []
                t = []
                while not self.event_bump.empty():
                    l.append(self.event_bump.get())
                for i in l:
                    if i[0].state == 1 and i[0].bumper == 0 and i[1] - self.last_bump > 0.2:
                        left = 1
                        self.last_bump = i[1]
                    if i[0].state == 1 and i[0].bumper == 1 and i[1] - self.last_bump > 0.2:
                        center = 1
                        self.last_bump = i[1]
                    if i[0].state == 1 and i[0].bumper == 2 and i[1] - self.last_bump > 0.2:
                        right = 1
                        self.last_bump = i[1]
                if left == 1:
                    i = 0
                    while i != 10:
                        i = i + 1
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = -0.12
                        cmd.angular.z = 0
                        self.vel.publish(cmd)
                        time.sleep(0.05)
                    while i != 20:
                        i = i + 1
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = 0
                        cmd.angular.z = -0.8
                        self.vel.publish(cmd)
                        time.sleep(0.05)
                    while i != 30:
                        i = i + 1
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = 0.12
                        cmd.angular.z = -0.5
                        self.vel.publish(cmd)
                        time.sleep(0.05)
                elif right == 1:
                    i = 0
                    while i != 10:
                        i = i + 1
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = -0.12
                        cmd.angular.z = 0
                        self.vel.publish(cmd)
                        time.sleep(0.05)
                    while i != 20:
                        i = i + 1
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = 0
                        cmd.angular.z = 0.8
                        self.vel.publish(cmd)
                        time.sleep(0.05)
                    while i != 30:
                        i = i + 1
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = 0.12
                        cmd.angular.z = 0.5
                        self.vel.publish(cmd)
                        time.sleep(0.05)
                elif center == 1:
                    i = 0
                    while i != 15:
                        i = i + 1
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = -0.12
                        cmd.angular.z = 0
                        self.vel.publish(cmd)
                        time.sleep(0.05)
            if not (left == 1 or center == 1 or right == 1):
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = x
                cmd.angular.z = w
                self.vel.publish(cmd)
                if self.traj.full():
                    self.traj.get()
                self.traj.put((x, w))
                # print (x,w)
                time.sleep(0.05)

    def tf_from_map_to_grid(self, x, y, originx, originy, resolution, height):
        start1 = (round((x - originx) / resolution),
                  round((y - originy) / resolution))
        start = (int(height - start1[1]), int(start1[0]))
        return start

    def map_proc(self, occu_map):
        # from up to bottom
        m = np.array(occu_map.data).reshape(occu_map.info.height, occu_map.info.width)
        m = m[::-1]
        m[m == 50] = -1

        if self.pcl_ground.full():
            ##
            with self.pcl_ground.mutex:
                pcl_ground = self.pcl_ground.queue[0]
            ground_cld = self.listener.transformPointCloud('map', pcl_ground)
            ground_cld = [
                self.tf_from_map_to_grid(i.x, i.y, occu_map.info.origin.position.x, occu_map.info.origin.position.y,
                                         occu_map.info.resolution, occu_map.info.height) for i in ground_cld.points]
            for gr in ground_cld:
                if gr[0] >= m.shape[0] - 2 or gr[1] >= m.shape[1] - 2 or gr[0] < 3 or gr[1] < 3:
                    # print(gr,m.shape,"gr")
                    continue
                m[gr] = 0

        if self.pcl.empty():
            return m
        with self.pcl.mutex:
            pcl = self.pcl.queue[0]

        obstacle_cld = self.listener.transformPointCloud('map', pcl)
        # obstacle_cld_base = self.listener.transformPointCloud('base_footprint', pcl)
        # angle=np.array([math.atan2(i.y,i.x) for i in obstacle_cld_base.points]).sort()
        # angle_diff=angle[1:]-angle[:-1]
        # ind=np.where(angle_diff>math.pi/12.0)
        self.obstacle_list+=[(i.x,i.y) for i in obstacle_cld.points ]
        obstacle_cld = [
            self.tf_from_map_to_grid(i[0], i[1], occu_map.info.origin.position.x, occu_map.info.origin.position.y,
                                     occu_map.info.resolution, occu_map.info.height)
            for i in self.obstacle_list]

        for ob in obstacle_cld:
            if ob[0] >= m.shape[0] - 2 or ob[1] >= m.shape[1] - 2 or ob[0] < 3 or ob[1] < 3:
                # print(ob, m.shape,"ob")
                continue
            m[ob] = 100
        return m

    def main_proc(self):
        rospy.init_node('move_base2.0', anonymous=True)
        self.robot_diameter = 0.3515
        self.robot_height = 0.1248
        self.map_buf = Queue.Queue(1)
        self.goals_buf = Queue.Queue(1)
        self.path = Queue.Queue(1)
        self.pcl = Queue.Queue(1)
        self.pcl_ground = Queue.Queue(1)
        self.event_bump = Queue.Queue(100)
        self.bump_time = Queue.Queue(100)
        self.event_cliff = Queue.Queue(100)
        self.event_drop = Queue.Queue(100)
        self.traj = Queue.Queue(100)
        rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, self.cb_map)
        rospy.Subscriber("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.cb_goal)
        rospy.Subscriber("obstacles_cloud", sensor_msgs.msg.PointCloud2, self.cb_pcl)
        rospy.Subscriber("ground_cloud", sensor_msgs.msg.PointCloud2, self.cb_pcl_ground)
        rospy.Subscriber("mobile_base/events/bumper", kobuki_msgs.msg.BumperEvent, self.cb_bump)
        # rospy.Subscriber("mobile_base/events/cliff", kobuki_msgs.msg.CliffEvent, self.cb_cliff)
        # rospy.Subscriber("mobile_base/events/wheel_drop", kobuki_msgs.msg.WheelDropEvent, self.cb_drop)

        self.vel = rospy.Publisher('mobile_base/commands/velocity', geometry_msgs.msg.Twist, queue_size=1)
        self.listener = tf.TransformListener()
        # self.global_ok=0
        self.sim_targets = None
        self.last_bump = 0
        self.obstacle_list=[]
        t1 = threading.Thread(target=self.dwa, args=())
        t1.setDaemon(True)
        t1.start()

        t2 = threading.Thread(target=self.pub_path, args=())
        t2.setDaemon(True)
        t2.start()
        # self.event_abnormal=threading.Event()
        # t3 = threading.Thread(target=self.map_proc, args=())
        # t3.setDaemon(True)
        # t3.start()
        while not rospy.is_shutdown():
            t1 = time.time()
            # if self.event_bump.full():
            #     self.event_abnormal.set()
            if self.map_buf.full() and self.goals_buf.full():
                with self.map_buf.mutex:
                    occu_map = self.map_buf.queue[0]
                with self.goals_buf.mutex:
                    goals = self.goals_buf.queue[0]
                # nav_map = np.array(occu_map.data).reshape(occu_map.info.height, occu_map.info.width)[::-1]
                nav_map = self.map_proc(occu_map)
                (trans, qua) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                start = self.tf_from_map_to_grid(trans[0], trans[1], occu_map.info.origin.position.x,
                                                 occu_map.info.origin.position.y, occu_map.info.resolution,
                                                 occu_map.info.height)
                end = self.tf_from_map_to_grid(goals.pose.position.x, goals.pose.position.y,
                                               occu_map.info.origin.position.x,
                                               occu_map.info.origin.position.y, occu_map.info.resolution,
                                               occu_map.info.height)
                t2 = time.time()
                # print(t2 - t1,"t1")
                path = cal_global_path(nav_map, start, end)
                path1 = [(x[1] * occu_map.info.resolution + occu_map.info.origin.position.x,
                          (occu_map.info.height - x[0]) * occu_map.info.resolution + occu_map.info.origin.position.y)
                         for x in path[::-1]]  # from start to end
                # print(time.time()-t2,"t2")

                P = nav_msgs.msg.Path()
                P.header = std_msgs.msg.Header(frame_id="map")
                P.poses = [
                    geometry_msgs.msg.PoseStamped(
                            header=std_msgs.msg.Header(frame_id="map"),
                            pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(path1[i][0], path1[i][1], 0)
                                                        )
                    )
                    for i in range(len(path1))]
                if self.path.full():
                    with self.path.mutex:
                        self.path.queue[0] = P
                else:
                    self.path.put(P)
        print("ok")


if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)
    n = Nav()
