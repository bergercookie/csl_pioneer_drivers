cmake_minimum_required(VERSION 2.8.3)
project(joy_to_arduino)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arduino_feedback
  joy
  roscpp
  rospy
  sensor_msgs
  )

catkin_package(CATKIN_DEPENDS)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp executable
add_executable(teleop_MR_2at src/teleop_MR_2at.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(teleop_MR_2at arduino_feedback_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(joy_to_arduino_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(teleop_MR_2at ${catkin_LIBRARIES})




