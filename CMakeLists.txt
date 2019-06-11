cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ekf_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  std_msgs
  message_generation
)



find_package(OpenCV REQUIRED)
find_package(Eigen3  REQUIRED)

add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
  Chassis_cmd.msg
  Chassis_data.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/include/json11
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
src/EKFLocalizer.cpp
src/EKFLocalizationWrapper.cpp
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/json11/)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)



add_executable(ekf_localization_node
src/EKFLocalizationNode.cpp)
target_link_libraries(ekf_localization_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  libjson11.a
)
add_dependencies(ekf_localization_node apriltag_ekf_localization_generate_messages_cpp)
