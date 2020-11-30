#include "control.hpp"
#include <iostream>
#include <fstream>

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
  }

  void ControlPlugin::OnUpdate() {
  }

  void ControlPlugin::OnLaserScan(const sensor_msgs::PointCloud& cloud) {
    auto& wpose = this->sensor->WorldPose();
    auto& pos = wpose.Pos();
    auto& rot = wpose.Rot();
    octomath::Vector3 trans(pos.X(), pos.Y(), pos.Z());
    octomap::pose6d pose(trans,
      octomath::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W())
    );

    ROS_INFO("Got Point Cloud");
    octomap::Pointcloud octomap_cloud;
    for (const geometry_msgs::Point32& point : cloud.points) {
      octomap::point3d q(point.x, point.y, point.z);
        octomap_cloud.push_back(q);
    }
    map.insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0), pose, 9);

    ofstream save("/home/dev/map.bt", ofstream::out | ofstream::binary | ofstream::trunc);
    bool success = map.writeBinary(save);
    if (!success)
      std::cerr << "Error: " << strerror(errno) << "\n";
    save.close();
  }
}