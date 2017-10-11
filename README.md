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

## Utils for playing back images:
Also included are the following 2 utilities which help with playing back video streams stored as either bag files or standard video files (.mp4, .avi, etc.): 

- bag_play_on_msg.py
- video_play_on_msg.py

Once either of these nodes is started, they publish images on the /camera/image_raw topic on receipt of Bool messages from the /img_processed topic. Typically these messages are generated from the node that consumes the images to signal that it is done processing a frame and is ready for the next. 

For testing purposes the Bool messages can be generated using rostopic pub: 

for a single message:

$ rostopic pub /img_processed std_msgs/Bool true

to produce a stream of images at a given frequency (here 2 per sec):

$ rostopic pub /img_processed std_msgs/Bool true -r 0.5
