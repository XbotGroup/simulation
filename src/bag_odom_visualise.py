#!/usr/bin/env python
# coding=utf-8
"""
数据回放

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

class bag_odom_visual():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.OdomTopic, Odometry, self.OdomCB)
        rospy.spin()

    def OdomCB(self, data):
        self.visualiser(data.pose.pose)
        self.amcl_pose(data)

    def amcl_pose(self, data):
        pub = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size=1)
        robot_pose = PoseWithCovarianceStamped()
        robot_pose.header = data.header
        robot_pose.pose = data.pose.pose
        pub.publish(robot_pose)

    def visualiser(self, pose):
        self.marker.pose.position = pose.position
        self.marker.pose.orientation = pose.orientation
        self.marker.header.stamp = rospy.Time.now()
        self.seq += 1
        self.marker.header.seq = self.seq
        pub = rospy.Publisher('/bag_odom_visual', Marker, queue_size=1)
        pub.publish(self.marker)


    def define(self):
        self.OdomTopic = '/odom'+'_test'
        self.marker = Marker()
        self.marker.header.frame_id =  'map'
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1
        self.marker.color.g = 1
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.seq = 0

if __name__=='__main__':
    rospy.init_node('bag_odom_visual')
    try:
        rospy.loginfo( "initialization system")
        bag_odom_visual()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
