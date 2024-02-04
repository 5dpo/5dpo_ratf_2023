cmake_minimum_required(VERSION 3.0.2)
project(sdpo_ratf_ros_path_planning)


file(GLOB_RECURSE PROGRAM_LIBS RELATIVE ${PROJECT_SOURCE_DIR}
  "include/*.cpp"
  "include/*.h"
)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  message_generation
  std_srvs
)


add_message_files(
  FILES
  boxes_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs
  tf
  std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(path_planning 
  src/path_planning.cpp
  ${PROGRAM_LIBS}
)

# add_executable(path_planning
#   src/main.cpp
# )

add_dependencies(path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_planning
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
)

