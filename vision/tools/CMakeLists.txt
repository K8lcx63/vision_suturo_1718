cmake_minimum_required(VERSION 2.8.3)
project(pcd_to_featureCSV)

set (CMAKE_CXX_STANDARD 11)


find_package(PCL 1.7 REQUIRED)

include_directories(
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(batch_processor batch_processor.cpp)

target_link_libraries(
        batch_processor     
        ${PCL_LIBRARIES}
)

