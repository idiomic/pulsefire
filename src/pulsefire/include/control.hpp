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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <octomap/octomap.h>

namespace gazebo {
  class ControlPlugin : public ModelPlugin {
    public:
      ControlPlugin();
      ~ControlPlugin();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
      void OnUpdate();
      void OnLaserScan(const sensor_msgs::PointCloud& cloud);

    protected:
      octomap::OcTree map;
      physics::ModelPtr model;
      physics::LinkPtr sensor;
      physics::JointPtr left_axel;
      physics::JointPtr right_axel;
      event::ConnectionPtr con;
      ros::NodeHandle node;
      ros::Subscriber sub;

      octomap::point3d target;
      bool is_target_goal = false;
      bool found_goal = false;

      octomap::pose6d GetPose();

      void MoveForward();
      void Navigate();
      void UpdateFringe(octomap::point3d);
      bool IsTargetStillFringe();
      void UpdateTarget();
      void GoToGoal();
  };
}