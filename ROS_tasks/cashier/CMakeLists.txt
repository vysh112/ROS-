cmake_minimum_required(VERSION 3.0.2)
project(cashier)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bill.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UpdateParameters.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Specify package build info export
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## Install the Python scripts
catkin_install_python(PROGRAMS
  src/publisher_node.py
  src/subscriber_node.py
  src/printer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
