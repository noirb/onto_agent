cmake_minimum_required(VERSION 2.8.3)
project(onto_agent)

add_compile_options(-std=c++11 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  cmake_modules
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  message_generation
  json_prolog
)

find_package(Eigen REQUIRED)

find_package(Curses REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES knowrob_test01
  CATKIN_DEPENDS roscpp rospy rosbag std_msgs geometry_msgs ontology_msgs ontology_svcs message_runtime
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  src/
  src/test/lest/
)

## Declare a C++ library
# add_library(knowrob_test01
#   src/${PROJECT_NAME}/knowrob_test01.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(knowrob_test01 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(${SWIPL_LIBRARY_DIRS})

## Declare a C++ executable
add_executable(onto_agent
  src/main.cpp
  src/Ontology.cpp
  src/Agent.cpp
  )
target_link_libraries(onto_agent ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${CURSES_LIBRARIES})

add_executable(knowrob_graph_test_node
  src/test/graph_test.cpp
  src/Ontology.cpp
  )
target_link_libraries(knowrob_graph_test_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${CURSES_LIBRARIES})

add_executable(knowrob_graph_test_updates
  src/test/graph_update_test.cpp
  )
target_link_libraries(knowrob_graph_test_updates ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(knowrob_graph_test_suite
  src/test/tests.cpp
  )
target_link_libraries(knowrob_graph_test_suite ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(knowrob_test01_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(knowrob_test01_node
#   ${catkin_LIBRARIES}
# )

