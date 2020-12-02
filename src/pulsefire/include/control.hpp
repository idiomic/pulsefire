#pragma once

#include <string>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <octomap/octomap.h>
#include "path.hpp"

namespace gazebo {
  class ControlPlugin : public ModelPlugin {
    public:
      ControlPlugin();
      ~ControlPlugin();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
      void OnUpdate();
      void OnLaserScan(const sensor_msgs::PointCloud& cloud);

    protected:
      tf2_ros::Buffer tfbuff;
      tf2_ros::TransformListener tflistener;
      tf2_ros::TransformBroadcaster tfbroadcaster;

      octomap::OcTree map;
      physics::ModelPtr model;
      physics::LinkPtr sensor;
      physics::JointPtr left_axel;
      physics::JointPtr right_axel;
      event::ConnectionPtr con;
      ros::NodeHandle node;
      ros::Subscriber sub;
      PathState floor_state;
      octomap::pose6d init_pose;

      PathNode* target = nullptr;
      bool is_target_goal = false;
      bool found_goal = false;

      octomap::pose6d GetPose();

      void MoveForward();
      void Navigate();
      void UpdateFringe(octomap::point3d);
      void UpdateTarget();
      void GoToGoal();
  };
}