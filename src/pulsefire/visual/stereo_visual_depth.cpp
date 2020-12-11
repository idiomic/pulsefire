#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "stereo_visual_depth.hpp"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

namespace gazebo {
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(StereoVisualDepth)

  StereoVisualDepth::StereoVisualDepth() :
    node("light_sensor_plugin"),
    publisher(node.advertise<sensor_msgs::PointCloud>("depth", 1)) {
    ROS_INFO("Loaded StereoVisualDepth");
  }

  StereoVisualDepth::~StereoVisualDepth() {
    ROS_INFO("Unloaded StereoVisualDepth");
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void StereoVisualDepth::OnNewFrameLeft(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format) {
    OnNewFrame(_image, this->utils[0]);
  }

  void StereoVisualDepth::OnNewFrameRight(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format) {
    OnNewFrame(_image, this->utils[1]);
  }
}