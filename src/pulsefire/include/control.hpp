#pragma once

#include <string>

#include <ros/ros.h>
#include <gazebo/GpuRayPlugin.hh>

namespace gazebo {
  class ControlPlugin : public GpuRayPlugin {
    public:
      ControlPlugin();
      ~ControlPlugin();

    protected:
      void OnNewLaserFrame(const float *_image,
        unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string &_format) override;

      ros::NodeHandle node;
      ros::Publisher publisher;
  };

}