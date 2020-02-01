cmake_minimum_required(VERSION 2.8.3)
project(ontology_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Activity.msg
  PropertyChanged.msg
  ObjUpdate.msg
  HandUpdate.msg
  OntoProperty.msg
  OntoPropertyList.msg
  DisplayLabel.msg
  GraphNode.msg
  Graph.msg
  GraphUpdate.msg
  SimulationStateChange.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  ontology_msgs
)

