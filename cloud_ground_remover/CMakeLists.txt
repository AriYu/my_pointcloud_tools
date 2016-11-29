cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(remove_surface)

find_package(Boost 1.4 COMPONENTS program_options REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (remove_surface src/remove_surface.cpp)
target_link_libraries (remove_surface ${PCL_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(ground_remover src/main.cpp src/ransac_remove_ground.cpp src/remove_ground.cpp src/region_growing_segmentation_remover.cpp)
target_link_libraries (ground_remover ${PCL_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(ground_remover_multi src/ground_remover_multi.cpp src/ransac_remove_ground.cpp src/remove_ground.cpp src/region_growing_segmentation_remover.cpp)
target_link_libraries (ground_remover_multi ${PCL_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)
