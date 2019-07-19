cmake_minimum_required(VERSION 2.8.3)
project(ras_talker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

add_compile_options(-std=c++11)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(talker_node src/talker.cpp)
target_link_libraries(talker_node ${catkin_LIBRARIES})





