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

add_executable (OnlyBin src/only_bin.cpp src/binsegmentation.cpp)
target_link_libraries (OnlyBin ${PCL_LIBRARIES})
target_link_libraries(OnlyBin ${Boost_LIBRARIES})

add_executable (EntropyOnly src/entropy_only.cpp src/entropy.cpp src/pointpose.cpp)
target_link_libraries (EntropyOnly ${PCL_LIBRARIES})
target_link_libraries(EntropyOnly ${Boost_LIBRARIES})

add_executable (Pico src/pico.cpp src/entropy.cpp src/pointpose.cpp)
target_link_libraries (Pico ${PCL_LIBRARIES})
target_link_libraries(Pico ${Boost_LIBRARIES})
