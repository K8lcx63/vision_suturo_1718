### Important setup note:
add the following to your bashrc (or zshrc):
export KINECT1=true  
export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/vision_suturo_1718/vision/models  
export GAZEBO_RESOURCE_PATH=${HOME}/catkin_ws/vision_suturo_1718/vision/worlds  

Use "catkin build object_detection" to build the messages-package beforehand.



### Services

#### Get Object Information
Get Information about:
- normal_features (VHFSignature308 as vector)
- color_features (Color Histogram as vector)
- object_amount (Number of objects in scene)
- object_poses  (PoseStamped of objects in scene)
- object_information (additional information)
- object_errors (errors while extracting)

> rosservice call /suturo_vision/objects_information

### Kinect
#### setup
sudo apt install ros-indigo-freenect-launch freenect libfreenect-bin

#### startup
roslaunch freenect_launch freenect-registered-xyzrgb.launch
oder
roslaunch vision vision_kinect.launch

#### save files
rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth_registered/points


### With real pr2
#### Topic to listen on
Topic: /kinect_head/depth_registered/points

#### Rosbags
**Recording:** rosbag record /kinect_head/depth_registered/points /tf
**Playing:** rosbag play -l filename.bag
