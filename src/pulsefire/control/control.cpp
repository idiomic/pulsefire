#include "control.hpp"
#include <iostream>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

#define NODE_NAME "controller"
#define LASER_TOPIC "point_cloud"

#define DEBUG

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
    map(0.1),
    tflistener(tfbuff),
    node(NODE_NAME),
    sub(node.subscribe(LASER_TOPIC, 64, &ControlPlugin::OnLaserScan, this)),
    floor_state(map)
  {
  }

  ControlPlugin::~ControlPlugin()
  {
    ROS_INFO("Unloaded Control Model Plugin");
  }

  void ControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    ROS_INFO("Loaded Control Model Plugin");
    this->model = parent;
    this->con = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ControlPlugin::OnUpdate, this));
    this->sensor = parent->GetLink("sensors");
    this->left_axel = parent->GetJoint("left_wheel_joint");
    this->right_axel = parent->GetJoint("right_wheel_joint");
    this->init_pose = GetPose();
  }

  void ControlPlugin::OnUpdate()
  {
    static unsigned seq = 0;
    static ros::Time last;
    auto pose = GetPose();

    auto now = ros::Time::now();
    if (now != last) {
      last = now;

      auto p = pose.trans();
      auto r = pose.rot();

      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.seq = seq++;
      tfMsg.header.stamp = now;
      tfMsg.header.frame_id = "world";
      tfMsg.child_frame_id = "sensors";
      tfMsg.transform.translation.x = p.x();
      tfMsg.transform.translation.y = p.y();
      tfMsg.transform.translation.z = p.z();
      tfMsg.transform.rotation.x = r.x();
      tfMsg.transform.rotation.y = r.y();
      tfMsg.transform.rotation.z = r.z();
      tfMsg.transform.rotation.w = r.u();
      tfbroadcaster.sendTransform(tfMsg);
    }

    // Attempt to find floor
    const octomath::Vector3 down(0, 0, -1);
    octomap::point3d floor_pos;
    bool is_hit = map.castRay(pose.trans(), down, floor_pos);

    // Move forward until we are on known ground
    if (!is_hit)
      return MoveForward();

    UpdateFringe(floor_pos);

    if (!is_target_goal)
    {
      if (!this->target || !this->target->is_fringe)
        UpdateTarget();
      else if (found_goal)
        GoToGoal();
    }

    Navigate();
  }

  void ControlPlugin::OnLaserScan(const sensor_msgs::PointCloud& cloud)
  {
    auto tfMsg = tfbuff.lookupTransform("world", "sensors", cloud.header.stamp, ros::Duration(1));
    auto trans = tfMsg.transform.translation;
    auto rot = tfMsg.transform.rotation;
    auto p = octomap::point3d(trans.x, trans.y, trans.z);
    auto r = octomath::Quaternion(rot.x, rot.y, rot.z, rot.w);
    auto pose = octomap::pose6d(p, r);

    ROS_INFO("Got Point Cloud at  %f in %s", cloud.header.stamp.toSec(), cloud.header.frame_id.c_str());
    ROS_INFO("Cur Pos: <%.3f, %.3f, %.3f>", pose.x(), pose.y(), pose.z());
    ROS_INFO("Got Transform");

    octomap::Pointcloud octomap_cloud;
    for (const geometry_msgs::Point32& point : cloud.points) {
      octomap::point3d q(point.x, point.y, point.z);
      if (pow2(q.x()) + pow2(q.y()) > pow2(0.3))
      octomap_cloud.push_back(q);
    }
    map.insertPointCloud(octomap_cloud, octomap::point3d(0,0,0), pose, 9);

    map.prune();

#if defined DEBUG
    ofstream save("/home/dev/map.bt", ofstream::out | ofstream::binary | ofstream::trunc);
    bool success = map.writeBinary(save);
    if (!success)
      std::cerr << "Error: " << strerror(errno) << "\n";
    save.close();
#endif
  }

  octomap::pose6d ControlPlugin::GetPose()
  {
    auto& wpose = this->sensor->WorldPose();
    auto& wpos = wpose.Pos();
    octomath::Vector3 pos(wpos.X(), wpos.Y(), wpos.Z());
    auto& wrot = wpose.Rot();
    octomath::Quaternion rot(wrot.X(), wrot.Y(), wrot.Z(), wrot.W());
    return octomap::pose6d(pos, rot);
  }

  void ControlPlugin::MoveForward()
  {
    this->left_axel->SetParam("fmax", 0, 100.0);
    this->left_axel->SetParam("vel", 0, 6 * M_PI);

    this->right_axel->SetParam("fmax", 0, 100.0);
    this->right_axel->SetParam("vel", 0, 6 * M_PI);
  }

  void ControlPlugin::Navigate()
  {
  }

  void ControlPlugin::UpdateFringe(octomap::point3d p)
  {
    expand_floor(this->floor_state, this->map.coordToKey(p));
  }

  void ControlPlugin::UpdateTarget()

  {}
  void ControlPlugin::GoToGoal()
  {}
}