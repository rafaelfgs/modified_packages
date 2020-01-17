# axis_camera_rafael

Package for accessing and publishing Image topics from Axis camera.


## Creating workspace for the package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/ros-drivers/axis_camera.git
git clone https://github.com/ros-perception/camera_info_manager_py.git
cd ..
catkin_make
source devel/setup.bash
```


## Viewing and setting IP adress:

```bash
sudo systemctl start dnsmasq
journalctl -fu dnsmasq
```

- IP address is usually: 192.168.1.133


## Running full launch
```bash
roslaunch axis_camera_rafael axis_camera_rafael.launch
```

#### Input parameters
- *camera_name*: Camera name used on topics (*default*=*axis*)
- *hostname*: Camera IP address (*default*=192.168.1.133)
- *width*: Width of captured image (*default*=1280)
- *height* Height of captured image (*default*=720)
- *frame_id*: Topic *frame_id* for publishing (*default*=*axis_camera*)
- *enable_ptz*: *boolean* variable for enable PTZ (*default*=*false*)
- *enable_republish*: *boolean* variable for image converting (*default*=*false*)
- *enable_proc*: *boolean* variable for rectifying (*default*=*false*)


## Running each node

#### Main node for publishing *sensor_msgs/CompressedImage*:
```bash
rosrun axis_camera_rafael axis.py _hostname:=192.168.1.133 _password:=root
```

#### Node for converting to *sensor_msgs/Image*:
```bash
rosrun image_transport republish compressed in:=image_raw raw out:=image_raw
```

#### Node for rectifying (*sensor_msgs/Image* and calibration required):
```bash
rosrun image_proc image_proc
```

#### Node for calibrating (*sensor_msgs/Image* required):
```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 \ image:=/axis/image_raw camera:=/axis
```

## For more details

- http://wiki.ros.org/axis_camera
- http://wiki.ros.org/camera_info_manager_py

