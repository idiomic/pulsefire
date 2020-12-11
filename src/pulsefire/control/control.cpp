#include "control.hpp"
#include <iostream>

namespace gazebo {

  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

  ControlPlugin::ControlPlugin() :
    node("controller"),
    sub(node.subscribe("/map", 10, &ControlPlugin::SetMap, this)),
    control(node.subscribe("/cmd_vel", 10, &ControlPlugin::Move, this)),
    server(node.advertiseService("/map", &ControlPlugin::GetMap, this))
  {
    M << r/2, r/2, -r/l, r/l;
    M_inv = M.inverse();
  }

  ControlPlugin::~ControlPlugin()
  {}

  void ControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    ROS_INFO("Loaded Control Model Plugin");
    this->model = parent;
    this->sensor = parent->GetLink("sensors");
    this->con = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ControlPlugin::OnUpdate, this));
    this->left_axel = parent->GetJoint("left_wheel_joint");
    this->right_axel = parent->GetJoint("right_wheel_joint");
    this->left_axel->SetVelocity(0, -24*M_PI);
    this->right_axel->SetVelocity(0, 0);
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

  void ControlPlugin::SetMap(const nav_msgs::OccupancyGrid::ConstPtr map) {
    this->map = map;
  }

  bool ControlPlugin::GetMap(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {
    printf("Get Map\n");
    res.map = *map;
    return true;
  }

  void ControlPlugin::Move(const geometry_msgs::Twist& twist) {
    Vector2f d;
    d << twist.linear.x, twist.angular.z;
    Vector2f v = M_inv * d;
    if (twist.linear.x > 0) {
      this->left_axel->SetVelocity(0, -v(1));
      this->right_axel->SetVelocity(0, -v(0));
    } else {
      this->left_axel->SetVelocity(0, v(1));
      this->right_axel->SetVelocity(0, v(0));
    }
  }
}