#!/usr/bin/env python

import rospy
import tf
from rosgraph_msgs.msg import Clock


tfs = [[(+0.270, +0.000, +0.070), (+0.0, +0.0, +0.0, +1.0), "chassis_init", "rtab_init"],
       [(-0.270, +0.000, -0.070), (+0.0, +0.0, +0.0, +1.0), "rtab_odom",    "rtab_chassis"],
       [(+0.000, +0.000, +0.090), (+0.0, +0.0, +0.0, +1.0), "rtab_chassis", "rtab_imu"],
       [(+0.000, +0.000, +0.000), (+0.0, +0.0, +0.0, +1.0), "rtab_odom",    "rtab_d435i"],
       [(+0.000, +0.000, +0.000), (-0.5, +0.5, -0.5, +0.5), "rtab_d435i",   "rtab_rgb"],
       [(+0.000, +0.000, +0.000), (-0.5, +0.5, -0.5, +0.5), "rtab_d435i",   "rtab_depth"]]

clock_pub = False


def tf_broadcaster():
    
    rospy.init_node("rtab_broadcaster_node", anonymous=True)
    rospy.Subscriber("/clock", Clock, callback)
    
    t = rospy.Time.now().to_sec()
    while not clock_pub or (t-rospy.Time.now().to_sec()) < 2.0:
        rospy.sleep(0.01)
    
    if clock_pub:
        rospy.loginfo("Clock found! Publishing RTAB-Map TFs according to Clock time...")
    else:
        rospy.loginfo("Clock not found! Publishing RTAB-Map TFs according to ROS time...")
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()


def callback(data):
    
    global clock_pub
    clock_pub = True
    
    for k in range(len(tfs)):
        msg = tf.TransformBroadcaster()
        msg.sendTransform(tfs[k][0], tfs[k][1], data.clock, tfs[k][3], tfs[k][2])


if __name__ == "__main__":
    tf_broadcaster()