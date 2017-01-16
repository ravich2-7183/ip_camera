# Description
ROS node to read frames from an ip camera stream and publish to a ROS topic. 

## To run:
python nodes/ip_camera.py -u http://192.168.100.2:8080/video

replace ip address with your camera's. 

## To test frame rate:
rostopic hz --window=1000 /ip_camera
