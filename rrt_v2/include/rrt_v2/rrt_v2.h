/**
 * @file rrt_v2.hpp
 * @author shahwazk
 * @brief Contains the RRT class
 * @cite: https://github.com/vss2sn/path_planning/blob/master/include/path_planning/rrt.hpp
 */

#ifndef RRT_H
#define RRT_H

#include <bits/stdc++.h>

#include "utils/utils.hpp"
#include "planner.h"

/// @brief Class for objects that plan using RRT algo
class RRT : public Planner
{
public:
  /// @brief Constructor
  /// @param grid the grid on which the planner is to plan
  explicit RRT(std::vector<std::vector<int>> grid) : Planner(std::move(grid)) {}

  /// @brief Setting RRT parameters
  /// @param threshold
  /// @param max_iterations_x_factor max iterations allowed in x direction
  void SetParams(const int threshold = 2, const int max_iterations_x_factor = 20);

  /// @brief RRT plan algorithm implementation
  /// @param start start node
  /// @param goal goal node
  /// @return tuple containing boolean of path found and vector of the found path
  std::tuple<bool, std::vector<Node>> Plan(const Node &start, const Node &goal) override;

private:
  /// @brief Find the nearest Node that has been seen by the algo. This does not consider cost to reach the node
  /// @param new_node Node to which the nearest node must be found
  /// @return nearest node
  std::tuple<bool, Node> FindNearestPoint(Node& new_node);

  /// @brief Check if there is any obstacle between the two nodes (obstacles are in grid so sqaure shape)
  /// @param n1 Node 1
  /// @param n2 Node 2
  /// @return bool value of whether obstacle exists between nodes
  bool IsAnyObstacleInPath(const Node &n1, const Node &n2) const;

  /// @brief Generate a random node
  /// @return Random generate node
  Node GenerateRandomNode() const;

  /// @brief Check if the goal is reachable from current node
  /// @param new_node Current node
  /// @return bool value of whetther goal is reachable from current node
  bool CheckGoalVisible(const Node &new_node);

  /// @brief Create obstacles list from input grid
  void CreateObstacleList();

  /// @brief Generate path from start node to goal node
  /// @return vector of Nodes connecting the path
  std::vector<Node> CreatePath();

private:
  Node start_, goal_;

  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> point_list_;
  std::unordered_map<Node, std::vector<Node>> near_nodes_;

  std::vector<Node> obstacles_list_;
  double threshold_ = 1.5;    // Read from json file
  int max_iter_x_factor_ = 500; // Read from json file
};

#endif //RRT_H
