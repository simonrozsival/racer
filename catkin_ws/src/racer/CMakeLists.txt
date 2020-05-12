cmake_minimum_required(VERSION 2.8.3)
project(racer)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17 -O3 -ffast-math)
add_compile_options(-std=c++17 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  racer_msgs
  visualization_msgs
  dynamic_reconfigure
  costmap_2d
  tf
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(trajectory_following_node src/trajectory_following_node.cpp)
add_executable(planning_node src/planning_node.cpp)
add_executable(circuit_node src/circuit_node.cpp)
add_executable(current_state_node src/current_state_node.cpp)
add_executable(visualize_action src/visualize_action.cpp)
add_executable(visualize_plan src/visualize_plan.cpp)
add_executable(visualize_carrot src/visualize_carrot.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(trajectory_following_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROMECT_NAME}_gencfg)
add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(circuit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(current_state_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visualize_action ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visualize_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visualize_carrot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_following_node
  ${catkin_LIBRARIES}
)

target_link_libraries(planning_node
  ${catkin_LIBRARIES}
)

target_link_libraries(circuit_node
  ${catkin_LIBRARIES}
)

target_link_libraries(current_state_node
  ${catkin_LIBRARIES}
)

target_link_libraries(visualize_action
  ${catkin_LIBRARIES}
)

target_link_libraries(visualize_plan
  ${catkin_LIBRARIES}
)

target_link_libraries(visualize_carrot
  ${catkin_LIBRARIES}
)