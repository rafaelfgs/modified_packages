#!/usr/bin/python

import rospy
import cv2
import time
import sys
import numpy as np
from copy import copy
from math import tan, pi
from threading import Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import std_msgs.msg as std

image_size = 400
freq_rate = 0.2
mode = "none" #"stack""overlay"

R = np.array([[ 0.9999756813050,  0.004391118884090,  0.005417240317910],
              [-0.0043891328387,  0.999990284443000, -0.000378685304895],
              [-0.0054188510403,  0.000354899209924,  0.999985337257000]])
T = np.array( [-0.0641773045063,  0.000311704527121, -4.76178320241e-06])

bridge = CvBridge()
frame_mutex = Lock()
msg_info = CameraInfo()
msg_header = std.Header()

def callback_img1(data):
    global img1_data
    img1_data = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

def callback_img2(data):
    global img2_data
    img2_data = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

def callback_info1(data):
    global K_left, D_left, time_diff
    if data.header.stamp.to_sec() < 1581534822:
        time_diff = rospy.Time.now().to_sec() - data.header.stamp.to_sec()
    else:
        time_diff = 0.0
    K_left = np.reshape(data.K,(3,3))
    D_left = np.array(data.D[0:4])

def callback_info2(data):
    global K_right, D_right
    K_right = np.reshape(data.K,(3,3))
    D_right = np.array(data.D[0:4])

def realsense_depth():
    
    global mode, frame_data, center_undistorted, disparity, disp_corrected, disp_vis, disp_color, color_image
    
    rospy.init_node("realsense_depth", anonymous=True)
    
    rospy.Subscriber("/realsense/fisheye1/image_raw", Image, callback_img1)
    rospy.Subscriber("/realsense/fisheye2/image_raw", Image, callback_img2)
    rospy.Subscriber("/realsense/fisheye1/camera_info", CameraInfo, callback_info1)
    rospy.Subscriber("/realsense/fisheye2/camera_info", CameraInfo, callback_info2)
    
    pub_depth_info  = rospy.Publisher("/realsense/stereo/depth/camera_info", CameraInfo, queue_size=1)
    pub_depth_image = rospy.Publisher("/realsense/stereo/depth/image_raw", Image, queue_size=1)
    pub_color_info  = rospy.Publisher("/realsense/stereo/color/camera_info", CameraInfo, queue_size=1)
    pub_color_image = rospy.Publisher("/realsense/stereo/color/image_raw", Image, queue_size=1)
    pub_rgb_info    = rospy.Publisher("/realsense/stereo/rgb/camera_info", CameraInfo, queue_size=1)
    pub_rgb_image   = rospy.Publisher("/realsense/stereo/rgb/image_raw", Image, queue_size=1)
    
    sys.stdout.write('\nWaiting for "/realsense" publications... ')
    sys.stdout.flush()
    while not 'time_diff' in globals():
        time.sleep(0.1)
    sys.stdout.write('Ok!\n\n')
    sys.stdout.flush()
    
    window_size = 5
    min_disp = 0
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                   numDisparities = num_disp,
                                   blockSize = 16,
                                   P1 = 8*3*window_size**2,
                                   P2 = 32*3*window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32)
    
    stereo_fov_rad = pi/2
    stereo_height_px = image_size
    stereo_width_px = stereo_height_px + max_disp
    stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2
    
    R_left = np.eye(3)
    R_right = R
    
    P_left = np.array([[stereo_focal_px,               0, stereo_cx, 0],
                       [              0, stereo_focal_px, stereo_cy, 0],
                       [              0,               0,         1, 0]])
    P_right = copy(P_left)
    P_right[0][3] = T[0]*stereo_focal_px
    
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left" : (lm1, lm2), "right" : (rm1, rm2)}
    
    msg_info.height = image_size
    msg_info.width = image_size
    msg_info.distortion_model = "plumb_bob"
    msg_info.K = [0.8*image_size, 0, 0.5*image_size, 0, 0.8*image_size, 0.5*image_size, 0, 0, 1]#K_left.flatten()
    msg_info.D = [0, 0, 0, 0, 0]#D_left.flatten()
    msg_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]#R_left.flatten()
    msg_info.P = [0.8*image_size, 0, 0.5*image_size, 0, 0, 0.8*image_size, 0.5*image_size,0 , 0, 0, 1, 0]#P_left.flatten()
    
    if not mode == "none":
        cv2.namedWindow("realsense", cv2.WINDOW_NORMAL)

    rate = rospy.Rate(freq_rate)
    count = 0
    
    while not rospy.is_shutdown() and time_diff > 0:
        
        #t=time.time()
        count += 1
        
        frame_mutex.acquire()
        frame_data = {"left"  : copy(img1_data),
                      "right" : copy(img2_data),
                      "seq"   : copy(count),
                      "stamp" : rospy.Time.from_sec(rospy.Time.to_sec(rospy.Time.now()) - time_diff),
                      "frame" : "realsense_depth_optical_frame"}
        frame_mutex.release()
        
        center_undistorted = {"left" : cv2.remap(src = frame_data["left"],
                                      map1 = undistort_rectify["left"][0],
                                      map2 = undistort_rectify["left"][1],
                                      interpolation = cv2.INTER_LINEAR),
                              "right" : cv2.remap(src = frame_data["right"],
                                      map1 = undistort_rectify["right"][0],
                                      map2 = undistort_rectify["right"][1],
                                      interpolation = cv2.INTER_LINEAR)}
        
        disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32)
        disparity[disparity<0.0] = 0.0
        disparity_cropped = disparity[:,max_disp:] / 16.0
        
#        disp_corrected = 5000.0/(disparity[:,max_disp:]**1.5+500.0)+0.1
        disp_corrected = 850/(disparity[:,max_disp:]+80.0)-0.4
        disp_corrected[disp_corrected>6.0] = 0.0
        
        disp_vis = 255*(disparity_cropped - min_disp) / num_disp
        disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
        color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
        
        msg_depth = bridge.cv2_to_imgmsg(disp_corrected, encoding="passthrough")
        msg_color = bridge.cv2_to_imgmsg(disp_color, encoding="passthrough")
        msg_rgb = bridge.cv2_to_imgmsg(color_image, encoding="rgb8")
        
        msg_header.seq = frame_data["seq"]
        msg_header.stamp = frame_data["stamp"]
        msg_header.frame_id = frame_data["frame"]
        
        msg_depth.header = msg_header
        msg_color.header = msg_header
        msg_rgb.header = msg_header
        msg_info.header = msg_header
        
        pub_depth_image.publish(msg_depth)
        pub_color_image.publish(msg_color)
        pub_rgb_image.publish(msg_rgb)
        pub_depth_info.publish(msg_info)
        pub_color_info.publish(msg_info)
        pub_rgb_info.publish(msg_info)
        
        if not mode == "none":
            
            if mode == "stack":
                img_window = np.hstack((color_image, disp_color))
                cv2.imshow("realsense", img_window)
                
            if mode == "overlay":
                img_window = disp_color
                img_window[disparity_cropped<min_disp] = 0.0
                cv2.imshow("realsense", img_window)
                
            key = cv2.waitKey(1)
            
            if key == ord("s"): mode = "stack"
            if key == ord("o"): mode = "overlay"
            if cv2.getWindowProperty("realsense", cv2.WND_PROP_VISIBLE) < 1: mode = "none"
        
        rate.sleep()
        #print(time.time()-t)
    
if __name__ == "__main__":
    try:
        realsense_depth()
    except rospy.ROSInterruptException:
        pass
