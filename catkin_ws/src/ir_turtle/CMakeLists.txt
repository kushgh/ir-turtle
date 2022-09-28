cmake_minimum_required(VERSION 3.0.2)
project(ir_turtle)

find_package(catkin REQUIRED COMPONENTS  geometry_msgs  roscpp  rospy  turtlesim)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(pub_turtle_sub_arduino src/pubsub_to_turtlesim_and_arduino.cpp)
target_link_libraries(pub_turtle_sub_arduino ${catkin_LIBRARIES})
