#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
import cv2
from copy import copy
from math import sqrt, pi, sin, cos, tan, asin, acos, atan2, tanh
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry



odom = Odometry()



def callback(data):

    global odom

    odom.header = copy(data.header)
    odom.header.frame_id = "base_footprint"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position = copy(data.pose.pose.position)
    odom.pose.pose.orientation = copy(data.pose.pose.orientation)



def change_msg():

    rospy.init_node("change_output_"+sys.argv[1], anonymous=True)
    rospy.Subscriber("/robot_pose_ekf_"+sys.argv[1]+"/pose_combined", PoseWithCovarianceStamped, callback)
    pub = rospy.Publisher("/robot_pose_ekf_"+sys.argv[1]+"/odom_combined", Odometry, queue_size=10)

    rate = rospy.Rate(30.0)

    print "Republishing following topics:"
    print "/robot_pose_ekf_"+sys.argv[1]+"/pose_combined as /robot_pose_ekf_"+sys.argv[1]+"/odom_combined"

    while not rospy.is_shutdown():
        pub.publish(odom)
        rate.sleep()



if __name__ == "__main__":
    try:
        change_msg()
    except rospy.ROSInterruptException:
        pass
