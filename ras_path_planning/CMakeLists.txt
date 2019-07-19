cmake_minimum_required(VERSION 2.8.3)
project(ras_path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  phidgets
)

add_compile_options(-std=c++11)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(path_plan src/a_star.cpp)
target_link_libraries(path_plan ${catkin_LIBRARIES})

add_executable(mismatch src/mismatch_collision.cpp)
target_link_libraries(mismatch ${catkin_LIBRARIES})

add_executable(closest_point_to_object src/closest_point_to_object.cpp)
target_link_libraries(closest_point_to_object ${catkin_LIBRARIES})

