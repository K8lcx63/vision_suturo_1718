cmake_minimum_required(VERSION 2.8.3)
project(vision_base)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  pcl_ros
  sensor_msgs
  pcl_conversions
  object_detection
)

catkin_package(CATKIN_DEPENDS
  object_detection
)

find_package(PCL 1.3 REQUIRED COMPONENTS
  common
  io
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(
  listener
  src/listener.cpp
)

target_link_libraries(
  listener
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

add_dependencies(listener beginner_tutorials_generate_messages_cpp gazebo_ros)
