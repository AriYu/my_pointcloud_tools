cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_transformer)

find_package(Boost 1.4 COMPONENTS program_options REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_transformer src/cloud_transformer.cpp)
target_link_libraries (cloud_transformer ${PCL_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)
