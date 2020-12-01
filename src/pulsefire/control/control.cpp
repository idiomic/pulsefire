#include "control.hpp"
#include <iostream>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

#define NODE_NAME "controller"
#define LASER_TOPIC "point_cloud"

using std::ofstream;

namespace gazebo {
  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

  ControlPlugin::ControlPlugin() :
    map(0.1),
    node(NODE_NAME),
    sub(node.subscribe(LASER_TOPIC, 64, &ControlPlugin::OnLaserScan, this)) {
  }

  ControlPlugin::~ControlPlugin() {
    ROS_INFO("Unloaded Control Model Plugin");
  }

  void ControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    ROS_INFO("Loaded Control Model Plugin");
    this->model = parent;
    this->con = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ControlPlugin::OnUpdate, this));
    this->sensor = parent->GetLink("sensors");
    this->left_axel = parent->GetJoint("left_wheel_joint");
    this->right_axel = parent->GetJoint("right_wheel_joint");
  }

  void ControlPlugin::OnUpdate() {
    octomap::pose6d pose(GetPose());

    // Attempt to find floor
    const octomath::Vector3 down(0, 0, -1);
    octomap::point3d floor_pos;
    bool is_hit = map.castRay(pose.trans(), down, floor_pos);

    // Move forward until we are on known ground
    if (!is_hit)
      return MoveForward();

    UpdateFringe(floor_pos);

    if (!is_target_goal) {
      if (IsTargetStillFringe())
        UpdateTarget();
      else if (found_goal)
        GoToGoal();
    }

    Navigate();
  }

  void ControlPlugin::OnLaserScan(const sensor_msgs::PointCloud& cloud) {
    octomap::pose6d pose = GetPose();

    ROS_INFO("Got Point Cloud");
    octomap::Pointcloud octomap_cloud;
    for (const geometry_msgs::Point32& point : cloud.points) {
      octomap::point3d q(point.x, point.y, point.z);
        octomap_cloud.push_back(q);
    }
    map.insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0), pose, 9);

#if defined DEBUG
    ofstream save("/home/dev/map.bt", ofstream::out | ofstream::binary | ofstream::trunc);
    bool success = map.writeBinary(save);
    if (!success)
      std::cerr << "Error: " << strerror(errno) << "\n";
    save.close();
#endif
  }

  octomap::pose6d ControlPlugin::GetPose() {
    auto& wpose = this->sensor->WorldPose();
    auto& wpos = wpose.Pos();
    octomath::Vector3 pos(wpos.X(), wpos.Y(), wpos.Z());
    auto& wrot = wpose.Rot();
    octomath::Quaternion rot(wrot.X(), wrot.Y(), wrot.Z(), wrot.W());
    return octomap::pose6d(pos, rot);
  }

  void ControlPlugin::MoveForward() {
    this->left_axel->SetParam("fmax", 0, 100.0);
    this->left_axel->SetParam("vel", 0, -2 * M_PI);

    this->right_axel->SetParam("fmax", 0, 100.0);
    this->right_axel->SetParam("vel", 0, -2 * M_PI);
  }

  void ControlPlugin::Navigate() {
  }

  void ControlPlugin::UpdateFringe(octomap::point3d) {}
  bool ControlPlugin::IsTargetStillFringe() {}
  void ControlPlugin::UpdateTarget() {}
  void ControlPlugin::GoToGoal() {}
}