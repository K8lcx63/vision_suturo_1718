cmake_minimum_required(VERSION 2.8.3)
project(vision)

set (CMAKE_CXX_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        pcl_ros
        sensor_msgs
        pcl_conversions
        object_detection
        message_generation
	visualization_msgs
	tf_conversions
        )

catkin_package(CATKIN_DEPENDS
        object_detection
        vision_msgs
        )

find_package(PCL 1.6 REQUIRED)

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(
        vision_main
        src/main.cpp
)

target_link_libraries(
        vision_main
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}

)

add_dependencies(vision_main beginner_tutorials_generate_messages_cpp gazebo_ros)
