#include "gnu_rrt_plotting/gnuplot-iostream.h"
#include <rclcpp/rclcpp.hpp>

class Node
{

public:
  /// @brief Node x coordianate
  float x_;
  /// @brief Node y coordinate
  float y_;

  /// @brief path found in x direction
  std::vector<float> path_x_;
  /// @brief path found in y direction
  std::vector<float> path_y_;

  /// @brief initial parent id
  int parent = -1;

  Node(float x, float y) : x_(x), y_(y) {}
  Node() {}
};

/// @brief Generate circle obstacle
using CircleObstacle = std::tuple<float, float, float>;

class RRT
{

public:
  RRT(float min_rand, float max_rand, std::vector<CircleObstacle> &obs,
      float expand_dis = 3.0f, float path_res = 0.25, int goal_sample_rate = 5,
      size_t max_iter = 500);

  std::pair<std::vector<float>, std::vector<float>>
  plan(float sx, float sy, float gx, float gy, Gnuplot &gp);

private:
  Node generateRandomNode();

  size_t nearestNodeIndex(std::vector<Node> &nodes, Node &query);

  Node steer(Node &from_node, Node &to_node, float extend_length);

  bool noCollision(Node &n);

  std::pair<std::vector<float>, std::vector<float>>
  generateFinalCourse(std::vector<Node> &node_list);

  std::pair<float, float> calculateDistanceAndAngle(Node &s, Node &g);

  float calculateDistanceToGoal(Node &n);

  float min_rand_;
  float max_rand_;
  std::vector<CircleObstacle> obs_;
  float expand_dis_;
  float path_res_;
  float goal_sample_rate_;
  size_t max_iter_;

  Node goal_node_;
  Node start_node_;
};
