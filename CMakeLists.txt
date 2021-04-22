cmake_minimum_required(VERSION 3.0.2)
project(rviz_markers_xml)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  INCLUDE_DIRS include
#  LIBRARIES rviz_markers_xml
#  CATKIN_DEPENDS geometry_msgs roscpp tinyxml2 visualization_msgs
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

add_executable(markers_xml_parser_test src/markers_xml_parser_test.cpp src/markers_xml_parser.cpp)
add_dependencies(markers_xml_parser_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(markers_xml_parser_test ${catkin_LIBRARIES})

add_executable(markers_xml_parser_node src/markers_xml_parser_node.cpp src/markers_xml_parser.cpp)
add_dependencies(markers_xml_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(markers_xml_parser_node ${catkin_LIBRARIES})

add_executable(rviz_markers_xml_node src/rviz_markers_xml_node.cpp src/markers_xml_parser.cpp)
add_dependencies(rviz_markers_xml_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_markers_xml_node ${catkin_LIBRARIES})
