################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hls_lfcd_lds_driver)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(SDK_PATH "./sdk/")

FILE(GLOB Laser_SRC
  src/dev/*.cpp
  src/ydlidar-sdk/*.cpp
  src/ydlidar-sdk/*.c
  src/ydlidar-sdk/impl/unix/*.cpp
  src/*.cpp
        )
################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  message_generation
  sensor_msgs
  cmake_modules
  tf
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################
add_service_files(FILES
  SetLidar.srv
)

add_message_files(FILES
  scan_ctrl.msg
)

Generate_messages(DEPENDENCIES std_msgs sensor_msgs)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
  std_msgs 
  sensor_msgs 
  std_srvs
  DEPENDS Boost
)

################################################################################
# Build
################################################################################
include_directories(
  include
  include/ydlidar-sdk
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/sdk/include
  ${PROJECT_SOURCE_DIR}/sdk/src
)

add_executable(hlds_laser_publisher	${Laser_SRC})
add_dependencies(hlds_laser_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hlds_laser_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(hlds_laser_segment_publisher	src/hlds_laser_segment_publisher.cpp)
#add_dependencies(hlds_laser_segment_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(hlds_laser_segment_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS hlds_laser_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
