#!/usr/bin/env python
#coding=utf-8
"""

This programm is tested on kuboki base turtlebot.
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

"""
import rospy
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import tf
from threading import Lock

class ClearParams:
    def __init__(self):
        rospy.delete_param('~publish_frequency')
        rospy.delete_param('~odom_frame_id')
        rospy.delete_param('~map_frame_id')

class init_pose_test():
 def __init__(self):
     self.define()
     rospy.Subscriber(self.TFTopic, TFMessage, self.TFCB)
     rospy.Subscriber(self.PoseSettingTopic, PoseStamped , self.InitPoseCB, queue_size=1)
     rospy.spin()

 def InitPoseCB(self, data):
     rospy.loginfo('updating robot initial pose')
     self.tf_translation = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
     self.tf_rotation = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)

 def TFCB(self, data):
     with self.locker:
        time = rospy.Time.now() + self.transform_tolerance
        tf_child = self.tf_map
        tf_parent = self.tf_odom
        self.tf_map2odom.sendTransform(self.tf_translation, self.tf_rotation, time, tf_parent, tf_child)
        self.r.sleep()

 def define(self):
     if not rospy.has_param('~publish_frequency'):
         rospy.set_param('~publish_frequency', 10.0)

     if not rospy.has_param('~odom_frame_id'):
         rospy.set_param('~odom_frame_id', 'odom')

     if not rospy.has_param('~map_frame_id'):
         rospy.set_param('~map_frame_id', 'map')

     publish_frequency = rospy.get_param("~publish_frequency")


     self.tf_odom = rospy.get_param('~odom_frame_id') #default 'odom'
     self.tf_map = rospy.get_param('~map_frame_id') #default 'map'

     self.r = rospy.Rate(publish_frequency)
     self.tf_map2odom = tf.TransformBroadcaster()

     self.tf_translation = (0.0, 0.0, 0.0)
     self.tf_rotation = (0.0, 0.0, 0.0, 1.0)
     self.transform_tolerance = rospy.Duration(0.0)


     self.PoseSettingTopic = '/set_pose'
     self.TFTopic = '/tf'
     self.locker = Lock()

if __name__=='__main__':
    rospy.init_node('init_pose')
    try:
        rospy.loginfo ("initialization system")
        init_pose_test()
        ClearParams()
        rospy.loginfo ("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("robot twist node terminated.")
