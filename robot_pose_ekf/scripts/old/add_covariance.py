#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
import cv2
from copy import copy
from math import sqrt, pi, sin, cos, tan, asin, acos, atan2, tanh
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry



odom_new = Odometry()
imu_new = Imu()
vo_new = Odometry()



def callback_odom(data):

    global odom_new

    odom_new = copy(data)
    odom_new.pose.covariance = list(1e-3*np.eye(6).flatten())
    odom_new.twist.covariance = list(1e-3*np.eye(6).flatten())



def callback_imu(data):

    global imu_new

    imu_new = copy(data)
    imu_new.orientation_covariance = list(1e-3*np.eye(3).flatten())
    imu_new.angular_velocity_covariance = list(1e-3*np.eye(3).flatten())
    imu_new.linear_acceleration_covariance = list(1e-3*np.eye(3).flatten())

    if sys.argv[4] == "_ouster":
        imu_new.orientation.z = -copy(data.orientation.z)
        imu_new.angular_velocity.z = -copy(data.angular_velocity.z)
        imu_new.linear_acceleration.z = -copy(data.linear_acceleration.z)



def callback_vo(data):

    global vo_new

    vo_new = copy(data)
    vo_new.child_frame_id = "base_link"
    vo_new.pose.covariance = list(1e-3*np.eye(6).flatten())
    vo_new.twist.covariance = list(1e-3*np.eye(6).flatten())



def main_function():

    rospy.init_node("add_covariance"+sys.argv[4], anonymous=True)

    rospy.Subscriber(sys.argv[1], Odometry, callback_odom)
    rospy.Subscriber(sys.argv[2], Imu, callback_imu)
    rospy.Subscriber(sys.argv[3], Odometry, callback_vo)

    pub_odom = rospy.Publisher("/robot_pose_ekf"+sys.argv[4]+"/odom", Odometry, queue_size=10)
    pub_imu = rospy.Publisher("/robot_pose_ekf"+sys.argv[4]+"/imu_data", Imu, queue_size=10)
    pub_vo = rospy.Publisher("/robot_pose_ekf"+sys.argv[4]+"/vo", Odometry, queue_size=10)

    rate = rospy.Rate(30.0)

    print "Republishing following topics:"
    print sys.argv[1], "as /robot_pose_ekf"+sys.argv[4]+"/odom"
    print sys.argv[2], "as /robot_pose_ekf"+sys.argv[4]+"/imu_data"
    print sys.argv[3], "as /robot_pose_ekf"+sys.argv[4]+"/vo"

    while not rospy.is_shutdown():
        pub_odom.publish(odom_new)
        pub_imu.publish(imu_new)
        pub_vo.publish(vo_new)
        rate.sleep()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
