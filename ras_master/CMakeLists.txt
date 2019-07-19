cmake_minimum_required(VERSION 2.8.3)
project(ras_master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  arduino_servo_control
  ras_msgs
)


catkin_package(
#INCLUDE_DIRS
#  LIBRARIES
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(master src/master.cpp)
target_link_libraries(master ${catkin_LIBRARIES})



