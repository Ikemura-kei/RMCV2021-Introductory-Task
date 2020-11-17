cmake_minimum_required(VERSION 3.0.2)
project(example_pkg)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES example_pkg
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Example.cpp
)

add_executable(example_node ./src/MyExample_Node.cpp)

target_link_libraries(example_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME} # link to my own lib
)
