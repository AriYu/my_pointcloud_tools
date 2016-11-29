cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(Boost 1.4 COMPONENTS program_options REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer src/cloud_viewer.cpp src/utils.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable (multi_cloud_viewer src/multi_cloud_viewer.cpp src/utils.cpp)
target_link_libraries (multi_cloud_viewer ${PCL_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)
