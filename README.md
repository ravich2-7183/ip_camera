# Description
ROS node to read frames from an ip camera stream and publish to a ROS topic. 

## To run:
directly:
python nodes/ip_camera.py -u http://192.168.100.2:8080/video

using roslaunch:
roslaunch ip_camera.launch

Of course, for this to work you should add the ip_camera dir to ROS_PACKAGE_PATH, so 'rospack find' can find ip_camera. 

Make sure to replace the video stream ip address with the correct one in ip_camera.launch. 

## To test frame rate:
rostopic hz --window=1000 /ip_camera
