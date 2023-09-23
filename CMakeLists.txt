cmake_minimum_required(VERSION 3.0.2)
project(rviz_markers_xml)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  visualization_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(markers_xml_parser_node src/markers_xml_parser_node.cpp src/markers_xml_parser.cpp)
add_dependencies(markers_xml_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(markers_xml_parser_node ${catkin_LIBRARIES})
