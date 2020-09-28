# espeleo_pose_ekf

Package with an Extended Kalman Filter used to estimate the 6D pose of EspeleoRobô.

## Original package

This is a modified version of robot_pose_ekf (1.14.5 2019-04-04), available in:

https://github.com/ros-planning/navigation/tree/kinetic-devel/robot_pose_ekf.

## Modifications

The main modifications of this package correspond to the computation of the z-axis.

The file *odom_estimation_node.cpp* was modified to perform this computation.

Changes in the other files were related to parameters and variable declarations.

The file *espeleo_pose_ekf.launch* shows the parameters and topics of this package.
