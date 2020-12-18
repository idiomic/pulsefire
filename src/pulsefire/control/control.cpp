#include "control.hpp"
#include <iostream>
#include <queue>

namespace gazebo
{

  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

  ControlPlugin::ControlPlugin() :
    node("controller"),
    sub(node.subscribe("/map", 10, &ControlPlugin::SetMap, this)),
    update_goal(node.subscribe("/clicked_point", 10, &ControlPlugin::SetGoal, this))
  {
    M << r/2, r/2, -r/l, r/l;
    M_inv = M.inverse();
    goal.x = 0;
    goal.y = 0;
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
  }

  void ControlPlugin::OnUpdate()
  {
    if (cur_p < points.size()) {
      auto p = this->model->WorldPose().Pos();
      auto r = this->model->WorldPose().Rot();
      Vector2f q;
      q << p.X(), p.Y();

      Vector2f a = points[cur_p - 1];
      Vector2f b = points[cur_p];
      Vector2f v = b - a;
      while ((q - a).dot(v) > v.norm()) {
        cur_p++;
        if (cur_p == points.size())
          break;
        a = b;
        b = points[cur_p];
        v = b - a;
      }

      if (cur_p < points.size()) {
        v = b - q;
        a << -std::sin(r.Z()), std::cos(r.Z());
        if (v.dot(a) < 0) {
          this->left_axel->SetVelocity(0, 100.0);
          this->right_axel->SetVelocity(0, 50.0);
        } else {
          this->left_axel->SetVelocity(0, 50.0);
          this->right_axel->SetVelocity(0, 100.0);
        }
      }

    } else {
      this->left_axel->SetVelocity(0, 0.0);
      this->right_axel->SetVelocity(0, 0.0);
    }

    auto now = ros::Time::now();
    if (now == last)
      return;

    last = now;
    publish("sensors", "map", this->sensor->WorldPose());
    publish("bot", "map", this->model->WorldPose());
    seq++;
  }

  void ControlPlugin::SetGoal(geometry_msgs::PoseStamped goal)
  {
    this->goal.x = goal.pose.position.x;
    this->goal.y = goal.pose.position.y;
  }

  float ControlPlugin::d2(const Node* n)
  {
    float dx = goal.x - n->x;
    float dy = goal.y - n->y;
    return dx * dx + dy * dy;
  }

  void ControlPlugin::SetMap(const nav_msgs::OccupancyGrid::ConstPtr map)
  {
    this->map = map;
    auto p = this->model->WorldPose().Pos();
    auto mp = map->info.origin.position;

    Node** nodes = new Node*[map->info.height];
    for (size_t y = 0; y < map->info.height; y++) {
      Node* row = new Node[map->info.width];
      nodes[y] = row;
      for (size_t x = 0; x < map->info.width; x++) {
        int data = map->data[y * map->info.width + x];
        row[x].x = x;
        row[x].y = y;
        row[x].goal_cost = d2(row + x);
        if (data == -1)
          row[x].is_unknown = true;
        else if (data > 50)
          row[x].is_open = false;
      }
    }

    printf("Number of nodes: %d\n", map->info.width * map->info.height);

    size_t x = std::clamp(
      (size_t)((p.X() - mp.x)/map->info.resolution), (size_t)0, (size_t)map->info.width);
    size_t y = std::clamp(
      (size_t)((p.Y() - mp.y)/map->info.resolution), (size_t)0, (size_t)map->info.height);
    printf("Start at: %llu %llu\n", x, y);
    Node* start = nodes[y] + x;
    start->goal_cost = d2(start);
    start->cost = start->goal_cost;
    
    MinHeap fringe;
    fringe.push(start);

    Node* best_unknown = nullptr;
    Node* best = nullptr;

    int num = 0;
    while (fringe.size() > 0) {
      Node* cur = fringe.pop();
      if (cur->goal_cost < 1) {
        printf("Found goal\n\n");
        best = cur;
        goto cleanup;
      }
      cur->is_visited = true;

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0)
            continue;
          x = cur->x + dx;
          y = cur->y + dy;
          if (x < 0 or x >= map->info.width or y < 0 or y >= map->info.height)
            continue;

          Node* child = nodes[y] + x;

          if (child->is_visited or not child->is_open)
            continue;

          float path_cost = cur->path_cost + dx*dx + dy*dy;
          if (child->is_unknown) {
            child->is_visited = true;
            child->path_cost = path_cost;
            child->parent = cur;
            if (best_unknown == nullptr or path_cost < best_unknown->path_cost)
              best_unknown = child;
          } else if (child->is_inserted) {
            if (path_cost < child->path_cost) {
              child->path_cost = path_cost;
              child->cost = path_cost + child->goal_cost;
              child->parent = cur;
              fringe.decreaseKey(child->i);
            }
          } else {
            child->path_cost = path_cost;
            child->cost = path_cost + child->goal_cost;
            child->parent = cur;
            child->is_inserted = true;
            fringe.push(child);
          }
        }
      }
    }

    if (best_unknown){
      best = best_unknown;
      printf("Found nearest unknown\n\n");
    } else
      printf("Could not find goal\n\n");

cleanup:
    points.clear();
    while (best) {
      Vector2f p;
      p << best->x * map->info.resolution, best->y * map->info.resolution;
      points.push_back(p);
      best = best->parent;
    }
    cur_p = 1;

    for (size_t y = 0; y < map->info.height; y++)
      delete[] nodes[y];
    delete[] nodes;
  }
}