//#include <gazebo/common/Plugin.hh>
#include "control.hpp"

//#include "gazebo_plugins/gazebo_ros_camera.h"
//#include <gazebo/sensors/Sensor.hh>
//#include <gazebo/sensors/SensorTypes.hh>
//#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

namespace gazebo {
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(ControlPlugin)

  ControlPlugin::ControlPlugin() :
    node("control_sensor_plugin"),
    publisher(node.advertise<sensor_msgs::PointCloud>("depth", 1)) {
    ROS_INFO("Loaded control_sensor_plugin");
  }

  ControlPlugin::~ControlPlugin() {
    ROS_INFO("Unloaded control_sensor_plugin");
  }

  void ControlPlugin::OnNewLaserFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format) {
    ROS_INFO("OnNewLaserFrame: %u, %u, %u\n", _width, _height, _depth);
  }
}