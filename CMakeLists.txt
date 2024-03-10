cmake_minimum_required(VERSION 3.0.2)
project(custom_ackermann_steering_controller_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

catkin_package()

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS
    scripts/ackermann_steering_controller.py
    scripts/simple_control.py
    scripts/keyboard_teleop.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

