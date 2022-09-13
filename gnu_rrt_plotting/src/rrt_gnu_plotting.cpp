

#include <bits/stdc++.h>

#include <gnu_rrt_plotting/rrt_gnu_plotting.h>

RRT::RRT(float min_rand, float max_rand, std::vector<CircleObstacle> &obs,
         float expand_dis, float path_res , int goal_sample_rate,
         size_t max_iter)
    : min_rand_(min_rand),
      max_rand_(max_rand),
      obs_(obs),
      expand_dis_(expand_dis),
      path_res_(path_res),
      goal_sample_rate_(goal_sample_rate),
      max_iter_(max_iter) {}

std::pair<std::vector<float>, std::vector<float>> RRT::plan(float sx, float sy,
                                                            float gx, float gy,
                                                            Gnuplot &gp)
{
  start_node_ = Node(sx, sy);
  goal_node_ = Node(gx, gy);

  std::vector<Node> node_list;
  node_list.emplace_back(sx, sy);

  for (size_t i = 0; i < max_iter_; i++)
  {
    // sample new node and find nearest neighbor
    Node random_node = generateRandomNode();

    size_t nearest_ind = nearestNodeIndex(node_list, random_node);

    Node nearest_node = node_list[nearest_ind];

    // steer to new node and check for collision, append if we can

    Node new_node = steer(nearest_node, random_node, expand_dis_);
    new_node.parent = nearest_ind;

    if (noCollision(new_node))
      node_list.push_back(new_node);

    // do plotting
    gp << "plot '-' with circles title 'obstacles' fill solid, '-'  with "
          "vectors nohead title 'tree', '-' title 'next' pointtype 26\n";

    gp.send1d(obs_);

    // display nodes (x, y);

    std::vector<std::tuple<float, float, float, float>> disp_nodes;
    for (auto &node : node_list)
    {
      if (node.parent != -1)
      {
        int p_id = node.parent;
        float dx = node_list[p_id].x_ - node.x_;
        float dy = node_list[p_id].y_ - node.y_;

        disp_nodes.emplace_back(node.x_, node.y_, dx, dy);
      }
    }

    gp.send1d(disp_nodes);

    // display sampled point
    std::vector<std::pair<float, float>> rand_plot = {
        {random_node.x_, random_node.y_}};

    // gp.send1d(rand_plot);

    sleep(1.0);

    if (calculateDistanceToGoal(new_node) <= expand_dis_)
    {
      Node final_node = steer(new_node, goal_node_, expand_dis_);
      if (noCollision(final_node))
      {
        auto [rx, ry] = generateFinalCourse(node_list);
        gp << "plot '-' with circles title 'obstacles' fill solid, '-'  with "
              "vectors nohead title 'tree', '-' with linespoints lw 5 title "
              "'path'\random_node";
        gp.send1d(obs_);
        gp.send1d(disp_nodes);
        gp.send1d(boost::make_tuple(rx, ry));

        return {rx, ry};
      }
    }
  }
  return {{}, {}};
}

std::pair<std::vector<float>, std::vector<float>> RRT::generateFinalCourse(
    std::vector<Node> &node_list)

{
  std::vector<float> rx, ry;
  size_t goal_id = node_list.size() - 1;

  Node node = node_list[goal_id];
  while (node.parent != -1)
  {
    rx.push_back(node.x_);
    ry.push_back(node.y_);

    node = node_list[node.parent];
  }
  return {rx, ry};
}

bool RRT::noCollision(Node &random_node)
{
  for (auto [ox, oy, r] : obs_)
  {
    for (size_t i = 0; i < random_node.path_x_.size(); ++i)
    {
      float dx = ox - random_node.path_x_[i];
      float dy = oy - random_node.path_y_[i];

      float d = std::hypot(dx, dy);

      // if point inside obs
      if (d <= r)
        return false;
    }
  }
  return true;
}

Node RRT::steer(Node &from_node, Node &to_node, float extend_length)
{
  Node new_node(from_node.x_, from_node.y_);

  new_node.path_x_.push_back(new_node.x_);
  new_node.path_y_.push_back(new_node.y_);

  auto [d, theta] = calculateDistanceAndAngle(new_node, to_node);

  // cap the extension length
  if (extend_length > d)
    extend_length = d;

  // generate trajectory to to_node
  int n_expand = std::floor(extend_length / path_res_);

  for (int i = 0; i < n_expand; i++)
  {
    new_node.x_ += path_res_ * std::cos(theta);
    new_node.y_ += path_res_ * std::sin(theta);

    new_node.path_x_.push_back(new_node.x_);
    new_node.path_y_.push_back(new_node.y_);
  }

  //  add last node if necessary
  auto [final_d, final_theta] = calculateDistanceAndAngle(new_node, to_node);

  if (final_d <= path_res_)
  {
    new_node.path_x_.push_back(to_node.x_);
    new_node.path_y_.push_back(to_node.y_);
  }
  return new_node;
}

Node RRT::generateRandomNode()
{
  std::random_device rd;

  std::mt19937 gen(rd());
  std::uniform_int_distribution<> goal_dis(0, 100);
  std::uniform_int_distribution<> point_dis(min_rand_, max_rand_);

  Node random_node;

  if (goal_dis(gen) > goal_sample_rate_)
  {
    random_node.x_ = point_dis(gen);
    random_node.y_ = point_dis(gen);
  }
  else
  {
    random_node.x_ = goal_node_.x_;
    random_node.y_ = goal_node_.y_;
  }

  RCLCPP_INFO(rclcpp::get_logger("rrt-gnu"),"Random number node generate successfully");
  return random_node;
}

size_t RRT::nearestNodeIndex(std::vector<Node> &nodes, Node &query)
{
  auto it = std::min_element(nodes.begin(), nodes.end(), [&](Node &n1, Node &n2) {
    float dx1 = n1.x_ - query.x_;
    float dy1 = n1.y_ - query.y_;

    float dx2 = n2.x_ - query.x_;
    float dy2 = n2.y_ - query.y_;

    return std::hypot(dx1, dy1) < std::hypot(dx2, dy2);
  });

  return std::distance(nodes.begin(), it);
}

std::pair<float, float> RRT::calculateDistanceAndAngle(Node &s, Node &g)
{
  float dx = g.x_ - s.x_;
  float dy = g.y_ - s.y_;

  float d = std::hypot(dx, dy);
  float theta = std::atan2(dy, dx);

  return {d, theta};
}

float RRT::calculateDistanceToGoal(Node &n)
{
  float dx = n.x_ - goal_node_.x_;
  float dy = n.y_ - goal_node_.y_;

  return std::hypot(dx, dy);
}
