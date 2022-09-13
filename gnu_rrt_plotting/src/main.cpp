
#include <gnu_rrt_plotting/rrt_gnu_plotting.h>

int main(int argc, char *argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("rrt-gnu"), "Started planning");
  Gnuplot gp;
  std::vector<CircleObstacle> obs = {
      {5, 5, 1},
      {3, 6, 2},
      {3, 8, 2},
      {3, 10, 2},
      {7, 5, 2},
      {9, 5, 2},
      {8, 10, 1}};

  gp << "set size ratio 1.0\n";
  gp << "set xrange [-2:15]\nset yrange [-2:18]\n";
  gp << "set term gif animate\n";
  gp << "set output '/home/bot/path_planner_ws/src/path_planning_algos/gnu_rrt_plotting/animations/rrt.gif'\n";

  float sx = 3.0;
  float sy = 3.0;
  float gx = 6.0;
  float gy = 10.0;

  float min_rand = -2.0;
  float max_rand = 15.0;

  RRT rrt(min_rand, max_rand, obs);
  auto [rx, ry] = rrt.plan(sx, sy, gx, gy, gp);


  gp << "set output\n";

  RCLCPP_INFO(rclcpp::get_logger("rrt-gnu"), "Planning finished!");

  return 0;
}
