#include "control.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <boost/function.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#define DEBUG

const std::string map_name("/map");

using std::ofstream;

template<typename T>
static inline constexpr T pow2(const T& v) {
  return v * v;
}

template<typename T>
static inline constexpr T dist2(const T& x, const T& y, const T& z) {
  return pow2(x) + pow2(y) + pow2(z);
}

namespace gazebo {

  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

  ControlPlugin::ControlPlugin() :
    node("controller"),
    sub(node.subscribe(map_name, 10, &ControlPlugin::SetMap, this)),
    server(node.advertiseService("/map", &ControlPlugin::GetMap, this))
  {}

  ControlPlugin::~ControlPlugin()
  {
    ROS_INFO("Unloaded Control Model Plugin");
  }

  void ControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    ROS_INFO("Loaded Control Model Plugin");
    this->model = parent;
    this->sensor = parent->GetLink("sensors");
    this->con = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ControlPlugin::OnUpdate, this));
    this->left_axel = parent->GetJoint("left_wheel_joint");
    this->right_axel = parent->GetJoint("right_wheel_joint");
  }

  void ControlPlugin::OnUpdate()
  {
    auto now = ros::Time::now();
    if (now == last)
      return;

    last = now;
    publish("sensors", "map", this->sensor->WorldPose());
    publish("bot", "map", this->model->WorldPose());
    seq++;
  }

  void ControlPlugin::SetMap(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    this->map = map;
  }

  bool ControlPlugin::GetMap(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {
    res.map = *map;
    return true;
  }
}