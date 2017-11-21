cmake_minimum_required(VERSION 2.8.3)
project(vision)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  sensor_msgs
  pcl_conversions
  vision_msgs
  message_generation
)

catkin_package(CATKIN_DEPENDS
  vision_msgs
)

find_package(PCL 1.3 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(
  listener
  src/main.cpp
)

target_link_libraries(
  listener
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(listener beginner_tutorials_generate_messages_cpp gazebo_ros)
