cmake_minimum_required(VERSION 2.8.3)
project(vision_base)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs object_detection)

catkin_package(CATKIN_DEPENDS object_detection)

include_directories()

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp gazebo_ros)
