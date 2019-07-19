cmake_minimum_required(VERSION 2.8.3)
project(ras_motor_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  phidgets
  geometry_msgs
)


catkin_package(
#INCLUDE_DIRS
#  LIBRARIES
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})



