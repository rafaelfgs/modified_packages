#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Image


tfs = [[(+0.270, +0.000, +0.070), (+0.0, +0.0, +0.0, +1.0), "chassis_init", "rtab_init"],
       [(-0.270, +0.000, -0.070), (+0.0, +0.0, +0.0, +1.0), "rtab_pose",    "rtab_chassis"],
       [(+0.000, +0.000, +0.090), (+0.0, +0.0, +0.0, +1.0), "rtab_chassis", "rtab_imu"],
       [(+0.000, +0.000, +0.000), (+0.0, +0.0, +0.0, +1.0), "rtab_pose",    "rtab_d435i"],
       [(+0.000, +0.000, +0.000), (-0.5, +0.5, -0.5, +0.5), "rtab_d435i",   "rtab_rgb"],
       [(+0.000, +0.000, +0.000), (-0.5, +0.5, -0.5, +0.5), "rtab_d435i",   "rtab_depth"]]

freq = 10
t = 0.0


def callback(data):
    global t
    t = data.header.stamp.to_sec()


def tf_broadcaster():
    
    rospy.init_node("rtab_broadcaster_node", anonymous=True)
    rospy.Subscriber("/rtabmap/rgb/image_raw", Image, callback)
    
    rate = rospy.Rate(freq)
    while t == 0.0 and not rospy.is_shutdown():
        rate.sleep()
    
    rospy.loginfo("Color image subscribed! Publishing RTAB-Map TFs according to RGB Image stamp...")
    
    t_pub = (1.0/freq - 0.001)
    t_last = 0.0
    
    while not rospy.is_shutdown():
        
        if (t - t_last) > t_pub:
            
            for k in range(len(tfs)):
                msg = tf.TransformBroadcaster()
                msg.sendTransform(tfs[k][0], tfs[k][1], rospy.Time.from_sec(t), tfs[k][3], tfs[k][2])
            
            t_last = t
        
        rate.sleep()        


if __name__ == "__main__":
    tf_broadcaster()