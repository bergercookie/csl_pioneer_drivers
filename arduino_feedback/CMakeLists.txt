cmake_minimum_required(VERSION 2.8.3)
project(arduino_feedback)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosserial_arduino
    std_msgs
    message_generation
    )

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    feedback_int.msg
    arduino_input.msg
    PID_gains.msg
    PID_motors_gains.msg
    )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    )

catkin_package(

    CATKIN_DEPENDS message_runtime
    )
