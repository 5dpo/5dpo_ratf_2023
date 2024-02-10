cmake_minimum_required(VERSION 3.0.2)
project(sdpo_ratf_ros_gui)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  pluginlib
  # Other dependecies
  std_msgs
  sdpo_ratf_ros_path_planning
  # message_generation
)

add_message_files(
  FILES
  boxes_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sdpo_ratf_ros_path_planning
)

catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS include
  #sdpo_ratf_ros_path_planning
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR} # To include UI header file
)


## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/ratfPanel.cpp
  src/main.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/ratfPanel.hpp
)

# Define ui file
set(${PROJECT_NAME}_UIS
  ui/ratf_panel.ui
)

# Create header from ui file (uic)
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
endif()

# add_executable(${PROJECT_NAME}_node
#   ${PROGRAM_SOURCES}
#   ${PROGRAM_HEADERS}
#   src/main.cpp
# )

add_executable(markers_node 
  src/main.cpp
  include/markers.h
  src/markers.cpp
)

add_dependencies(markers_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(markers_node
 ${catkin_LIBRARIES}
)

## Add library is needed in order to generate the header file from ui file.
add_library(ratf_panel
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS} 
)

target_link_libraries(ratf_panel
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

