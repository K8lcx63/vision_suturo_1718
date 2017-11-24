### Important setup note:
add the following to your bashrc (or zshrc):
export KINECT1=true  
export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/vision_suturo_1718/vision/models  
export GAZEBO_RESOURCE_PATH=${HOME}/catkin_ws/vision_suturo_1718/vision/worlds  

Use "catkin build object_detection" to build the messages-package beforehand.

##### Kinect 
###### setup
sudo apt install ros-indigo-freenect-launch freenect libfreenect-bin

###### startup
roslaunch freenect_launch roslaunch freenect_launch freenect-registered-xyzrgb.launch
oder
roslaunch vision vision_kinect.launch

##### save files
rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth_registered/points

