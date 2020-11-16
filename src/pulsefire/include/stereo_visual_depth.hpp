#pragma once

#include <string>
#include <gazebo_plugins/gazebo_ros_multicamera.h>

namespace gazebo {
  class StereoVisualDepth : public GazeboRosMultiCamera {
    public:
      StereoVisualDepth();
      ~StereoVisualDepth();

    protected:
      void OnNewFrameLeft(const unsigned char *_image,
        unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string &_format) override;
      void OnNewFrameRight(const unsigned char *_image,
        unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string &_format) override;

      ros::NodeHandle node;
      ros::Publisher publisher;
  };

}