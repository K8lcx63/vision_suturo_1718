cmake_minimum_required(VERSION 2.8.3)
project(suturo_vision)

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
        suturo_vision_msgs
)

find_package(PCL 1.7 REQUIRED)

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(
        vision_node
		src/main.cpp
		src/perception/perception.cpp
		src/saving/saving.cpp
		src/viewer/viewer.cpp
		src/perception/short_types.h
		src/perception/transformer/CloudTransformer.cpp
		src/node/vision_node.cpp

)

target_link_libraries(
        vision_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}

)

add_dependencies(vision_node beginner_tutorials_generate_messages_cpp gazebo_ros)
