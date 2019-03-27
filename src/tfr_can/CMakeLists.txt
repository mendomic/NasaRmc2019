cmake_minimum_required(VERSION 2.8.3)
project(tfr_can)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kacanopen
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

catkin_package(
)

add_executable(create_ros_topics_for_can_nodes
  src/create_ros_topics_for_can_nodes.cpp
)
add_dependencies(create_ros_topics_for_can_nodes tfr_msgs_gencpp)
target_link_libraries(create_ros_topics_for_can_nodes
  ${catkin_LIBRARIES}
)



