#pragma once
#include <unordered_map> 
#include <octomap/octomap.h>
#include <functional>

typedef octomap::OcTreeKey PathKey;

struct PathNode
{
	PathNode *north = nullptr;
	PathNode *east = nullptr;
	PathNode *south = nullptr;
	PathNode *west = nullptr;

	PathKey key;
	PathNode(const PathKey& key) : key(key) {}

	bool is_expanded = false;
	bool is_fringe = false;
};

struct PathState
{
	std::unordered_map<
		PathKey,
		PathNode*,
		PathKey::KeyHash
	> nodes;
	std::vector<PathNode*> fringe;
	octomap::OcTree& map;

	PathState(octomap::OcTree& map) : map(map) {}
};

void expand_floor(PathState& state, const PathKey& floor_key);