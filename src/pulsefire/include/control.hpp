#pragma once

#include <string>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/GetMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <octomap/octomap.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>
#include "minheap.hpp"
#include <vector>

using namespace Eigen;

namespace gazebo {

  class ControlPlugin : public ModelPlugin {
    public:
      ControlPlugin();
      ~ControlPlugin();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
      void OnUpdate();
      void SetMap(const nav_msgs::OccupancyGrid::ConstPtr map);
      void SetGoal(geometry_msgs::PoseStamped goal);

    protected:
      float d2(const Node* n);

      tf2_ros::TransformBroadcaster tfbroadcaster;
      physics::ModelPtr model;
      physics::LinkPtr sensor;
      physics::JointPtr left_axel;
      physics::JointPtr right_axel;
      event::ConnectionPtr con;
      ros::NodeHandle node;
      ros::Subscriber sub;
      ros::Subscriber update_goal;
      ros::ServiceServer server;

      nav_msgs::OccupancyGrid::ConstPtr map;

      float r = 0.033;
      float l = 0.26;
      Matrix<float, 2, 2> M;
      Matrix<float, 2, 2> M_inv;

      Node goal;
      std::vector<Vector2f> points;
      int cur_p = 0;

      unsigned seq = 0;
      ros::Time last;

      template<typename T>
      void publish(const char* from, const char* to, const T& pose) {
        auto p = pose.Pos();
        auto r = pose.Rot();
        geometry_msgs::TransformStamped tfMsg;
        tfMsg.header.seq = seq;
        tfMsg.header.stamp = last;
        tfMsg.header.frame_id = to;
        tfMsg.child_frame_id = from;
        tfMsg.transform.translation.x = p.X();
        tfMsg.transform.translation.y = p.Y();
        tfMsg.transform.translation.z = p.Z();
        tfMsg.transform.rotation.x = r.X();
        tfMsg.transform.rotation.y = r.Y();
        tfMsg.transform.rotation.z = r.Z();
        tfMsg.transform.rotation.w = r.W();
        tfbroadcaster.sendTransform(tfMsg);
      }
  };
}