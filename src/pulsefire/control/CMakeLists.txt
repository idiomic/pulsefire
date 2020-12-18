project(control)

find_package(catkin REQUIRED COMPONENTS
	gazebo_plugins
	gazebo_ros
	roscpp
	laser_geometry
)

find_package(gazebo REQUIRED)
find_package(octomap REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
                     ${OCTOMAP_INCLUDE_DIRS}
)

add_library(control control.cpp)
target_link_libraries(control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OCTOMAP_LIBRARIES})