cmake_minimum_required(VERSION 3.0.2)

include_directories(include)

add_subdirectory(visual)
add_subdirectory(control)

project(pulsefire)
catkin_package(
	INCLUDE_DIRS include
  	CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
)