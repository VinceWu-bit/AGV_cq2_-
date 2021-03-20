cmake_minimum_required(VERSION 2.8.3)
project(velocity_smoother)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  geometry_msgs 
  nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Build driver node
add_executable(velocity_smoother src/velocity_smoother_node.cpp src/velocity_smoother.cpp)
target_link_libraries(velocity_smoother
  ${catkin_LIBRARIES}
)

