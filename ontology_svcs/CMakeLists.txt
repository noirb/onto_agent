cmake_minimum_required(VERSION 2.8.3)
project(ontology_svcs)

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

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetAllClassInstances.srv
  GetAllClassInstancesWithPoses.srv
  DisplayLabel.srv
  InstructBehavior.srv
  GetActivity.srv
  GetActivities.srv
  GetActivityGraph.srv
  GetTaskGraph.srv
  RelabelActivity.srv
  RelabelObject.srv
  AssertClass.srv
  PerformTask.srv
  SaveLog.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  ontology_msgs
)


