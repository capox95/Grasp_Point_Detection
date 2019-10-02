cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Entropy_Filter)

find_package(Boost REQUIRED)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable (GraspPointDetection src/main.cpp src/entropy.cpp  src/binsegmentation.cpp src/pointpose.cpp)
target_link_libraries (GraspPointDetection ${PCL_LIBRARIES})
target_link_libraries(GraspPointDetection ${Boost_LIBRARIES})
