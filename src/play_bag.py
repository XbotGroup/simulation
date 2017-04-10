#!/usr/bin/env python
# coding=utf-8
"""
数据回放

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
import rosbag
import getpass
import copy
from sensor_msgs.msg  import LaserScan
from nav_msgs.msg import Odometry
import signal

class PlayBag():
    def __init__(self):
        self.define()
        bag = rosbag.Bag(self.path)
        #debug
        #self.debug_ReadBag(bag)
        self.Pub_Fack_Topic(bag)

    def Pub_Fack_Topic(self, bag):
        laser_pub = rospy.Publisher(self.LaserTopic+'_test', LaserScan, queue_size=1)
        odom_pub = rospy.Publisher(self.OdomTopic+'_test', Odometry, queue_size=1)
        laser_data = LaserScan()
        laser_data.header.frame_id = self.laser_frame
        laser_data.header.seq = 0
        laser_data.header.stamp = rospy.Time.now()

        odom_data = Odometry()
        odom_data.header.frame_id = self.odom_frame
        odom_data.header.seq = 0
        odom_data.header.stamp = rospy.Time.now()

        for topic, msg, time in bag.read_messages(topics=['/scan', '/odom']):
            rospy.sleep(1.0 / self.frequency)
            if topic == self.LaserTopic:
                self.laser_seq +=1
                if msg.header.frame_id != self.laser_frame:
                    msg.header.frame_id = self.laser_frame
                laser_data = copy.deepcopy(msg)
            if topic == self.OdomTopic:
                self.odom_seq += 1
                #if msg.header.frame_id != self.odom_frame:
                    #msg.header.frame_id = self.odom_frame
                odom_data = copy.deepcopy(msg)

            self.pub(laser_data, self.laser_seq, laser_pub)
            self.pub(odom_data, self.odom_seq, odom_pub)
            #self.dbug()
            rospy.loginfo('publishing data...')
            signal.signal(signal.SIGINT, self.Break)

    def dbug(self):
        raw_input('press ENTER to continu')

    def pub(self, data, seq, pub):
        data.header.seq = seq
        data.header.stamp = rospy.Time.now()
        pub.publish(data)

    def Break(self, signal, frame):
        rospy.loginfo('KeyBoardInterruption')
        raise

    def debug_ReadBag(self, bag):
        topic_info = bag.get_type_and_topic_info()
        print 'topic_info', topic_info
        for topic, msg, time in bag.read_messages(topics=['/scan', '/odom']):
            #debug
            debug = raw_input('input ENTER to continue')
            print 'topic: ', topic
            print 'msg： ', msg.header
            #print 'time: ',time
            if debug.lower() == 'q':
                break

    def define(self):
        usr_name = getpass.getuser()
        WorkSpaces = 'Xbot'
        self.bag_name = 'amcl.bag'
        self.path = '/home/%s/%s/src/bags/'%(usr_name, WorkSpaces) + self.bag_name
        self.odom_seq = 0
        self.laser_seq = 0
        self.frequency = 90 #hz
        self.OdomTopic = '/odom'
        self.LaserTopic = '/scan'
        self.laser_frame = 'camera_depth_frame'
        self.odom_frame = 'odom'


if __name__=='__main__':
    rospy.init_node('play_bag')
    try:
        rospy.loginfo( "initialization system")
        PlayBag()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
