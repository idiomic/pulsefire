project(visual)

find_package(catkin REQUIRED COMPONENTS
	gazebo_plugins
	gazebo_ros
	roscpp
)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
)

add_library(visual stereo_visual_depth.cpp)

target_link_libraries(visual ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})