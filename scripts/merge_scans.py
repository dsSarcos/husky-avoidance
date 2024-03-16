#!/usr/bin/env python3
# Author: Diego Santamaria-sarcos
# Author: Marius Baden
# Source [4]: http://wiki.ros.org/message_filters

import rospy
import math
from sensor_msgs.msg import LaserScan
import message_filters
from std_msgs.msg import Float64
from rob_midterm_dsmb.msg import SmartScan

def pub_merge(front, distance):
    smart_scan = SmartScan()
    smart_scan.scan = front
    smart_scan.distance = distance.data
    pub.publish(smart_scan)
    rospy.loginfo("Sending distance: %0.5f", smart_scan.distance)

rospy.init_node("merge_scans")
rospy.loginfo("Started merging scans node")

pub = rospy.Publisher("scan", SmartScan, queue_size=1)

front_scan_sub = message_filters.Subscriber("front/scan", LaserScan)
destination_distance_sub = message_filters.Subscriber("obj_distance", Float64)

synchroizer = message_filters.ApproximateTimeSynchronizer([front_scan_sub, destination_distance_sub], 
    queue_size=20, slop=0.25, allow_headerless=True)

synchroizer.registerCallback(pub_merge)
rospy.loginfo("Waiting to receive first messages now scans node")
rospy.spin()