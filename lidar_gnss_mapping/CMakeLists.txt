cmake_minimum_required(VERSION 2.8.3)
project(lidar_gnss_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  message_generation
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  gps_ins_msg.msg
  gpgga_msg.msg
  odom_and_status.msg
  gps_status.msg
  odom_and_ctrl.msg
  headinga_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_integration
#  CATKIN_DEPENDS pcl_ros roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
${GTSAM_INCLUDE_DIR}
)
link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_gps_ins_node src/gps_ins_node.cpp)
add_executable(${PROJECT_NAME}_rtk_registration src/rtk_registration.cpp)
add_executable(${PROJECT_NAME}_rtk_odom src/rtk_odom.cpp)
add_executable(${PROJECT_NAME}_rtk_mapping src/rtk_mapping.cpp)
add_executable(${PROJECT_NAME}_register_lidar_gps_new src/register_lidar_gps_new.cpp)

add_dependencies(${PROJECT_NAME}_gps_ins_node ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}_rtk_registration ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}_rtk_odom ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}_rtk_mapping ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}_register_lidar_gps_new ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME}_gps_ins_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_rtk_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_rtk_odom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_rtk_mapping ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_register_lidar_gps_new ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} gtsam)


