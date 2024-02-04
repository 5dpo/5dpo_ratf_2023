cmake_minimum_required(VERSION 2.8.3)
project(MPU9255)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  calib_matrix
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(MPU9255_node src/MPU9255_node.cpp)
target_link_libraries(MPU9255_node ${catkin_LIBRARIES} Eigen3::Eigen)
target_link_libraries (MPU9255_node wiringPi)
