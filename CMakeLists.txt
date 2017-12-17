cmake_minimum_required(VERSION 2.8.3)
project(laserscan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  laser_geometry
  tf
  sensor_msgs
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
)

find_package(PCL REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)
include_directories(${csm_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Laser.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laserscan
#  CATKIN_DEPENDS roscpp rospy std_msg
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
##add_executable(scan_to_cloud 
##  src/scan_to_cloud.cpp
##)

add_executable(scan_projector 
  src/scan_projector.cpp
)
target_link_libraries(scan_projector 
  ${catkin_LIBRARIES}
)

add_executable(line_extraction 
  src/line_extraction.cpp
)
target_link_libraries(line_extraction 
  ${catkin_LIBRARIES}
)

#add_executable(laser_odom 
#  src/laser_odom.cpp
#)
#target_link_libraries(laser_odom 
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

add_executable(laser_odom 
  src/laser_scan_matcher.cpp
)
target_link_libraries(laser_odom 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${csm_LIBRARIES}
)

add_executable(laser_odom_opt 
  src/csm_with_imu.cpp
)
target_link_libraries(laser_odom_opt 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${csm_LIBRARIES}
)


