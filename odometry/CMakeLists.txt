cmake_minimum_required(VERSION 2.8.3)
project(odometry)

find_package(catkin REQUIRED COMPONENTS
  arduino_feedback
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

add_message_files(
   FILES
   odo_msg.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_node_3dx src/odometry_node3.cpp)
target_link_libraries(odometry_node_3dx ${catkin_LIBRARIES})

add_dependencies (odometry_node_3dx odometry_generate_messages_cpp)
