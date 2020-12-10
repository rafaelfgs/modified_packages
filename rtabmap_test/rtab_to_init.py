#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry


class Odom2Init:
    
    
    def __init__(self):
        
        rospy.init_node("rtab_to_init_node", anonymous=True)
        
        self.topic = rospy.get_param("~topic", "/rtabmap/odom")
        tf_string = rospy.get_param("~tf", "0.27 0 0.07 0 0 0 1")
        self.tf = tuple([float(x) for x in tf_string.split(" ")])
        
        self.odom = Odometry()
        rospy.Subscriber(self.topic, Odometry, self.callback)
        self.pub = rospy.Publisher(self.topic+'_init', Odometry, queue_size=10)
        
        rospy.loginfo("Republishing %s to chassis_init", self.topic)
    
    
    def callback(self, data):
        
        p1 = (data.pose.pose.position.x,
              data.pose.pose.position.y,
              data.pose.pose.position.z)
        
        q1 = (data.pose.pose.orientation.x,
              data.pose.pose.orientation.y,
              data.pose.pose.orientation.z,
              data.pose.pose.orientation.w)
        
        p0 = self.tf[:3]
        q0 = self.tf[3:]
        
        q2 = qq_mult(q0, qq_mult(q1, q_conjugate(q0)))
        p3 = pp_sum(qp_mult(q0, p1), p0)
        p2 = pp_sum(qp_mult(q2, p_conjugate(p0)), p3)
        
        self.odom.header.seq = data.header.seq
        self.odom.header.stamp = data.header.stamp
        self.odom.header.frame_id = "chassis_init"
        self.odom.child_frame_id = data.child_frame_id
        
        self.odom.pose.pose.position.x = p2[0]
        self.odom.pose.pose.position.y = p2[1]
        self.odom.pose.pose.position.z = p2[2]
        
        self.odom.pose.pose.orientation.x = q2[0]
        self.odom.pose.pose.orientation.y = q2[1]
        self.odom.pose.pose.orientation.z = q2[2]
        self.odom.pose.pose.orientation.w = q2[3]
        
        self.odom.pose.covariance = data.pose.covariance
        
        self.pub.publish(self.odom)
    
    
def pp_sum(p1, p2):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x = x1 + x2
    y = y1 + y2
    z = z1 + z2
    return x, y, z

def p_conjugate(p):
    x, y, z = p
    return -x, -y, -z

def qq_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w

def q_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w

def qp_mult(q1, p1):
    q2 = p1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]


if __name__ == "__main__":
    
    Odom2Init()
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
