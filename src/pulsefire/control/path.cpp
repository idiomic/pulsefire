#include "path.hpp"
#include <ros/ros.h>

void expand_floor(PathState& state, const PathKey& floor_key)
{
	auto floor_iter = state.nodes.find(floor_key);
	PathNode *floor;
	if (floor_iter == state.nodes.end())
		floor = new PathNode(floor_key);
	else
		floor = floor_iter->second;

	octomap::OcTreeNode *node = state.map.search(floor_key);
	if (node == nullptr)
		ROS_INFO("Floor is empty");
	else if (node->getOccupancy() >= 0.5)
		ROS_INFO("Floor is occupied at %f", node->getOccupancy());
	else
		ROS_INFO("Floor is unoccupied at %f", node->getOccupancy());
}