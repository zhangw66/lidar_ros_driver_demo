cmake_minimum_required(VERSION 2.8.3)
project(lh_lidar_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
	include
)

add_executable(lh_lidar_node src/lh_laser.cpp src/lh_laser_publisher.cpp)
 target_link_libraries(lh_lidar_node
   ${catkin_LIBRARIES}
 )

