cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(sacpreicp)
#=====pcl
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

#=====cuda
find_package(CUDA REQUIRED)
INCLUDE(FindCUDA)
include_directories(${CUDA_INCLUDE_DIRS})
message("CUDA INCLUDE DIRS: ${CUDA_INCLUDE_DIRS}")

#======for sac prerejective algorithm
add_executable(sacpreicp "sacpreicp.cpp")
target_link_libraries(sacpreicp ${PCL_LIBRARIES})

