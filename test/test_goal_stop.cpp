// Check the real controller together with the native stopped goal checker.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>

#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_footprint"),
    rclcpp::Parameter("width", 6), rclcpp::Parameter("height", 6),
    rclcpp::Parameter("origin_x", -3.0), rclcpp::Parameter("origin_y", -3.0),
    rclcpp::Parameter("resolution", 0.05),
    rclcpp::Parameter("robot_radius", 0.2),
    rclcpp::Parameter("plugins", std::vector<std::string>{})});
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  costmap->configure();
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_stop_check");
  parent->declare_parameter("controller_frequency", 20.0);
  parent->declare_parameter("FollowPath.translation_kp", 3.0);
  parent->declare_parameter("FollowPath.translation_ki", 0.0);
  parent->declare_parameter("FollowPath.translation_kd", 0.0);
  parent->declare_parameter("FollowPath.enable_rotation", false);
  parent->declare_parameter("FollowPath.v_linear_max", 2.5);
  parent->declare_parameter("FollowPath.use_velocity_scaled_lookahead_dist", false);
  parent->declare_parameter("FollowPath.lookahead_dist", 0.6);
  parent->declare_parameter("FollowPath.max_velocity_scaling_factor_rate", 0.9);
  parent->declare_parameter("FollowPath.min_approach_linear_velocity", 0.15);
  parent->declare_parameter("FollowPath.approach_velocity_scaling_dist", 2.4);
  parent->declare_parameter("goal.xy_goal_tolerance", 0.1);
  parent->declare_parameter("goal.yaw_goal_tolerance", 6.28);
  parent->declare_parameter("goal.stateful", false);
  parent->declare_parameter("goal.trans_stopped_velocity", 0.1);
  parent->declare_parameter("goal.rot_stopped_velocity", 10.0);
  auto buffer = costmap->getTfBuffer();
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = parent->now();
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_footprint";
  tf.transform.rotation.w = 1.0;
  buffer->setTransform(tf, "test", true);
  tf.header.frame_id = "map";
  tf.child_frame_id = "odom";
  tf.transform.translation.x = 2.0;
  buffer->setTransform(tf, "test", true);

  nav2_controller::StoppedGoalChecker checker;
  checker.initialize(parent, "goal", costmap);
  pb_omni_pid_pursuit_controller::OmniPidPursuitController controller;
  controller.configure(parent, "FollowPath", buffer, costmap);
  controller.activate();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.header.stamp = parent->now();
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  auto plan = [&](double x, double y, bool clipped = false) {
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      auto point = pose;
      point.header = path.header;
      point.pose.position.x = 2.0;
      path.poses.push_back(point);
      if (clipped) {
        point.pose.position.x = 2.05;
        path.poses.push_back(point);
      }
      point.pose.position.x = 2.0 + x;
      point.pose.position.y = y;
      path.poses.push_back(point);
      controller.setPlan(path);
    };
  plan(0.2, 0.0);
  auto command = controller.computeVelocityCommands(pose, moving, &checker);
  assert(std::hypot(command.twist.linear.x, command.twist.linear.y) >= 0.149);
  plan(0.08, 0.0);
  command = controller.computeVelocityCommands(pose, moving, &checker);
  assert(command.twist.linear.x == 0.0 && command.twist.linear.y == 0.0);
  auto goal = pose.pose;
  goal.position.x = 0.08;
  assert(!checker.isGoalReached(pose.pose, goal, moving));
  assert(checker.isGoalReached(pose.pose, goal, geometry_msgs::msg::Twist()));
  plan(0.08, 0.08);
  command = controller.computeVelocityCommands(pose, moving, &checker);
  assert(std::hypot(command.twist.linear.x, command.twist.linear.y) > 0.0);
  plan(4.0, 0.0, true);
  command = controller.computeVelocityCommands(pose, moving, &checker);
  assert(std::hypot(command.twist.linear.x, command.twist.linear.y) > 0.0);
  // A curve followed by a straight replan recovers speed gradually.
  auto test_path = [&](const std::vector<std::pair<double, double>> & coordinates) {
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      for (const auto & xy : coordinates) {
        auto point = pose;
        point.header = path.header;
        point.pose.position.x = 2.0 + xy.first;
        point.pose.position.y = xy.second;
        path.poses.push_back(point);
      }
      controller.setPlan(path);
    };
  test_path({{0.0, 0.0}, {0.2, 0.02}, {0.4, 0.1}, {0.6, 0.3},
             {0.8, 0.6}, {1.0, 1.0}, {1.5, 1.5}, {2.0, 2.0}});
  for (int i = 0; i < 10; ++i) {
    command = controller.computeVelocityCommands(pose, moving, &checker);
  }
  const double curved_speed = std::hypot(command.twist.linear.x, command.twist.linear.y);
  assert(curved_speed > 0.8 && curved_speed < 1.2);
  std::vector<std::pair<double, double>> straight;
  for (int i = 0; i <= 40; ++i) straight.emplace_back(0.1 * i, 0.0);
  test_path(straight);
  command = controller.computeVelocityCommands(pose, moving, &checker);
  double previous_speed = std::hypot(command.twist.linear.x, command.twist.linear.y);
  assert(previous_speed < 1.2);
  for (int i = 0; i < 25; ++i) {
    test_path(straight);  // Frequent replans preserve recovery progress.
    command = controller.computeVelocityCommands(pose, moving, &checker);
    const double speed = std::hypot(command.twist.linear.x, command.twist.linear.y);
    assert(speed >= previous_speed - 1e-8 && speed - previous_speed <= 0.082);
    previous_speed = speed;
  }
  assert(previous_speed > 1.79 && previous_speed < 1.81);

  // A moving obstacle on the path still invokes the existing collision stop.
  auto grid = costmap->getCostmap();
  unsigned int cell_x, cell_y;
  assert(grid->worldToMap(0.6, 0.0, cell_x, cell_y));
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      grid->setCost(cell_x + dx, cell_y + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  bool stopped_for_obstacle = false;
  try {
    controller.computeVelocityCommands(pose, moving, &checker);
  } catch (const nav2_core::PlannerException &) {
    stopped_for_obstacle = true;
  }
  assert(stopped_for_obstacle);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      grid->setCost(cell_x + dx, cell_y + dy, nav2_costmap_2d::FREE_SPACE);
    }
  }
  command = controller.computeVelocityCommands(pose, moving, &checker);
  assert(command.twist.linear.x > 0.0);
  // Enable distance braking after verifying the original collision mode above.
  assert(parent->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.obstacle_slowdown_distance", 2.4),
    rclcpp::Parameter("FollowPath.obstacle_stop_margin", 0.1),
    rclcpp::Parameter("FollowPath.obstacle_deceleration", 2.0),
    rclcpp::Parameter("FollowPath.obstacle_reaction_time", 0.3)}).successful);
  assert(!parent->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.obstacle_deceleration", 0.0)}).successful);
  assert(parent->get_parameter("FollowPath.obstacle_deceleration").as_double() == 2.0);
  auto with_obstacle = [&](double x, double y, unsigned char cost) {
      unsigned int mx, my;
      assert(grid->worldToMap(x, y, mx, my));
      grid->setCost(mx, my, cost);
      auto result = controller.computeVelocityCommands(pose, moving, &checker);
      grid->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
      return result;
    };
  test_path(straight);
  assert(parent->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.lookahead_dist", 1.0),
    rclcpp::Parameter("FollowPath.obstacle_slowdown_distance", 2.9)}).successful);
  assert(std::abs(controller.computeVelocityCommands(pose, moving, &checker).twist.linear.x - 2.5) < 1e-8);
  assert(parent->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.lookahead_dist", 0.6),
    rclcpp::Parameter("FollowPath.obstacle_slowdown_distance", 2.4)}).successful);
  const double clear_speed = controller.computeVelocityCommands(pose, moving, &checker).twist.linear.x;
  double last = clear_speed + 0.01;
  for (const double x : {2.01, 1.11, 0.61, 0.376, 0.28}) {
    const auto result = with_obstacle(x, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
    const double speed = std::hypot(result.twist.linear.x, result.twist.linear.y);
    assert(speed < last && speed >= 0.0);
    // Compare stopping travel with physical footprint clearance, allowing one grid cell.
    const double radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
    assert(speed * 0.3 + speed * speed / 4.0 <= std::max(0.0, x - radius - 0.1 + 0.05));
    if (x < 0.4) assert(speed < 0.15);  // Obstacle stop overrides the goal crawl floor.
    std::cout << "obstacle_x=" << x << " speed=" << speed << "\n";
    last = speed;
  }
  assert(last == 0.0);
  // A cleared obstacle allows the existing smoother to restore normal travel.
  assert(controller.computeVelocityCommands(pose, moving, &checker).twist.linear.x == clear_speed);
  assert(with_obstacle(-0.61, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE).twist.linear.x == clear_speed);
  assert(with_obstacle(0.61, 0.8, nav2_costmap_2d::LETHAL_OBSTACLE).twist.linear.x == clear_speed);
  assert(with_obstacle(0.61, 0.18, nav2_costmap_2d::LETHAL_OBSTACLE).twist.linear.x < clear_speed);
  assert(with_obstacle(0.61, 0.0, nav2_costmap_2d::NO_INFORMATION).twist.linear.x < clear_speed);

  // Dense footprint checks also find obstacles between sparse path waypoints.
  test_path({{0.0, 0.0}, {2.0, 0.0}});
  assert(with_obstacle(1.11, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE).twist.linear.x < 1.4);
  std::vector<std::pair<double, double>> reverse;
  for (int i = 0; i <= 40; ++i) reverse.emplace_back(-0.1 * i, 0.0);
  test_path(reverse);
  const double reverse_speed = with_obstacle(-0.61, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE).twist.linear.x;
  assert(reverse_speed < 0.0 && reverse_speed > -clear_speed);

  // Current footprint overlap retains the hard collision stop.
  test_path(straight);
  assert(grid->worldToMap(0.05, 0.0, cell_x, cell_y));
  grid->setCost(cell_x, cell_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  stopped_for_obstacle = false;
  try {
    controller.computeVelocityCommands(pose, moving, &checker);
  } catch (const nav2_core::PlannerException &) {
    stopped_for_obstacle = true;
  }
  assert(stopped_for_obstacle);
  grid->setCost(cell_x, cell_y, nav2_costmap_2d::FREE_SPACE);
  auto edge = pose;
  edge.pose.position.x = 2.85;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_footprint";
  tf.transform.translation.x = 2.85;
  buffer->setTransform(tf, "test", true);
  test_path({{2.85, 0.0}, {2.95, 0.0}});
  stopped_for_obstacle = false;
  try {
    controller.computeVelocityCommands(edge, moving, &checker);
  } catch (const nav2_core::PlannerException &) {
    stopped_for_obstacle = true;
  }
  assert(stopped_for_obstacle);
  tf.transform.translation.x = 0.0;
  buffer->setTransform(tf, "test", true);
  plan(0.08, 0.0);
  command = controller.computeVelocityCommands(pose, moving, &checker);
  assert(command.twist.linear.x == 0.0 && command.twist.linear.y == 0.0);
  // Brake before an intermediate bend, while the final goal is still far away.
  assert(parent->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.lookahead_dist", 1.0),
    rclcpp::Parameter("FollowPath.obstacle_slowdown_distance", 2.9),
    rclcpp::Parameter("FollowPath.max_lateral_accel", 1.5)}).successful);
  assert(!parent->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.max_lateral_accel", -1.0)}).successful);
  test_path(straight);
  for (int i = 0; i < 40; ++i) controller.computeVelocityCommands(pose, moving, &checker);
  assert(std::abs(controller.computeVelocityCommands(pose, moving, &checker).twist.linear.x - 2.5) < 1e-8);
  double bend_previous = 2.5;
  for (const double corner : {2.0, 1.0, 0.5}) {
    std::vector<std::pair<double, double>> bend;
    for (int i = 0; i <= 20; ++i) bend.emplace_back(corner * i / 20.0, 0.0);
    for (int i = 1; i <= 20; ++i) bend.emplace_back(corner, i / 10.0);
    test_path(bend);
    command = controller.computeVelocityCommands(pose, moving, &checker);
    const double speed = std::hypot(command.twist.linear.x, command.twist.linear.y);
    std::cout << "bend_distance=" << corner << " speed=" << speed << "\n";
    assert(speed < bend_previous && speed > 0.0);
    if (corner == 2.0) assert(speed < 2.4);
    bend_previous = speed;
  }
  // Passing an intermediate waypoint must preserve gradual speed recovery.
  test_path(straight);
  command = controller.computeVelocityCommands(pose, moving, &checker);
  assert(command.twist.linear.x <= bend_previous + 2.5 * 0.9 * 0.05 + 1e-8);
  for (int i = 0; i < 40; ++i) controller.computeVelocityCommands(pose, moving, &checker);
  assert(std::abs(controller.computeVelocityCommands(pose, moving, &checker).twist.linear.x - 2.5) < 1e-8);
  // Recorded mid-route shape: the path bends away from a nearby obstacle,
  // while the direct 0.4 m pursuit segment cuts across its footprint margin.
  grid->resetMap(0, 0, grid->getSizeInCellsX(), grid->getSizeInCellsY());
  assert(parent->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.lookahead_dist", 0.4)}).successful);
  assert(grid->worldToMap(.275, -.075, cell_x, cell_y));
  grid->setCost(cell_x, cell_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  const std::vector<std::pair<double, double>> corner_path{
    {0., 0.}, {.038, .026}, {.058, .085}, {.101, .141}, {.142, .170},
    {.183, .198}, {.224, .226}, {.285, .268}, {.326, .296}, {.408, .353},
    {.510, .423}, {.604, .515}};
  test_path(corner_path);
  for (int i = 0; i < 5; ++i)
    command = controller.computeVelocityCommands(pose, moving, &checker);
  std::cout << "corner_speed=" << std::hypot(command.twist.linear.x, command.twist.linear.y)
            << " x=" << command.twist.linear.x << " y=" << command.twist.linear.y << std::endl;
  assert(std::hypot(command.twist.linear.x, command.twist.linear.y) >= .14);
  assert(command.twist.linear.y > command.twist.linear.x);
  // An obstacle on the bend itself still stops the controller.
  assert(grid->worldToMap(.125, .275, cell_x, cell_y));
  grid->setCost(cell_x, cell_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  bool bend_stopped = false;
  try {
    command = controller.computeVelocityCommands(pose, moving, &checker);
    bend_stopped = std::hypot(command.twist.linear.x, command.twist.linear.y) < 1e-6;
  } catch (const nav2_core::PlannerException &) {
    bend_stopped = true;
  }
  assert(bend_stopped);
  controller.deactivate();
  controller.cleanup();
  // Run the requested high-speed settings through the existing controller.
  grid->resizeMap(360, 360, 0.05, -9.0, -9.0);
  parent->declare_parameter("Fast.translation_kp", 3.0);
  parent->declare_parameter("Fast.translation_ki", 0.0);
  parent->declare_parameter("Fast.translation_kd", 0.0);
  parent->declare_parameter("Fast.enable_rotation", false);
  parent->declare_parameter("Fast.v_linear_max", 5.0);
  parent->declare_parameter("Fast.use_velocity_scaled_lookahead_dist", true);
  parent->declare_parameter("Fast.lookahead_time", 1.2);
  parent->declare_parameter("Fast.max_lookahead_dist", 2.0);
  parent->declare_parameter("Fast.min_lookahead_dist", 0.4);
  parent->declare_parameter("Fast.approach_velocity_scaling_dist", 6.6);
  parent->declare_parameter("Fast.min_approach_linear_velocity", 0.15);
  parent->declare_parameter("Fast.max_velocity_scaling_factor_rate", 0.9);
  parent->declare_parameter("Fast.obstacle_slowdown_distance", 6.9);
  parent->declare_parameter("Fast.obstacle_deceleration", 2.5);
  parent->declare_parameter("Fast.obstacle_reaction_time", 0.3);
  parent->declare_parameter("Fast.obstacle_stop_margin", 0.1);
  parent->declare_parameter("Fast.max_lateral_accel", 2.5);
  pb_omni_pid_pursuit_controller::OmniPidPursuitController fast;
  fast.configure(parent, "Fast", buffer, costmap);
  fast.activate();
  moving.linear.x = 5.0;
  auto fast_plan = [&](const std::vector<std::pair<double, double>> & coordinates) {
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      for (const auto & xy : coordinates) {
        auto point = pose;
        point.header = path.header;
        point.pose.position.x = 2.0 + xy.first;
        point.pose.position.y = xy.second;
        path.poses.push_back(point);
      }
      fast.setPlan(path);
    };
  auto fast_speed = [&]() {
      const auto result = fast.computeVelocityCommands(pose, moving, &checker);
      return std::hypot(result.twist.linear.x, result.twist.linear.y);
    };
  // Short goals start from rest with usable speed even with a 5 m/s ceiling.
  moving.linear.x = 0.0;
  fast_plan({{0., 0.}, {2., 0.}});
  const double short_start = fast_speed();
  std::cout << "short_goal_2m_start_speed=" << short_start << '\n';
  assert(short_start > 1.0);
  assert(short_start * .3 + short_start * short_start / 5.0 <= 1.9);
  fast_plan({{0., 0.}, {.8, 0.}});
  assert(fast_speed() > .8);
  std::vector<std::pair<double, double>> short_route;
  for (int i = 0; i <= 20; ++i) short_route.emplace_back(i * .1, 0.0);
  fast_plan(short_route);
  double travelled = 0.0, actual_speed = 0.0, peak_speed = 0.0;
  int steps = 0;
  geometry_msgs::msg::TransformStamped robot_tf;
  robot_tf.header.frame_id = "odom";
  robot_tf.child_frame_id = "base_footprint";
  robot_tf.transform.rotation.w = 1.0;
  for (; steps < 200; ++steps) {
    auto current_pose = pose;
    current_pose.header.stamp = parent->now();
    current_pose.pose.position.x = travelled;
    robot_tf.header.stamp = current_pose.header.stamp;
    robot_tf.transform.translation.x = travelled;
    buffer->setTransform(robot_tf, "test", true);
    moving.linear.x = actual_speed;
    const double requested = fast.computeVelocityCommands(current_pose, moving, &checker).twist.linear.x;
    const double next = std::clamp(requested, actual_speed - 2.5 * .05, actual_speed + 2.5 * .05);
    travelled += (actual_speed + next) * .5 * .05;
    actual_speed = next;
    peak_speed = std::max(peak_speed, actual_speed);
    if (std::abs(2.0 - travelled) <= .1 && std::abs(actual_speed) <= .1) break;
  }
  std::cout << "short_route_time=" << steps * .05 << " peak=" << peak_speed
            << " final_x=" << travelled << std::endl;
  assert(steps * .05 < 6.0 && peak_speed > .8);
  assert(std::abs(travelled - 2.0) <= .1 && std::abs(actual_speed) <= .1);
  robot_tf.transform.translation.x = 0.0;
  buffer->setTransform(robot_tf, "test", true);
  moving.linear.x = 5.0;
  std::vector<std::pair<double, double>> long_straight;
  for (int i = 0; i <= 100; ++i) long_straight.emplace_back(i * .1, 0.0);
  fast_plan(long_straight);
  assert(std::abs(fast_speed() - 5.0) < 1e-6);
  double approach_previous = 5.0;
  for (double distance : {6.0, 3.0, 1.0, .2, .08}) {
    fast_plan({{0., 0.}, {distance, 0.}});
    const double speed = fast_speed();
    assert(speed < approach_previous && speed >= 0.0);
    assert(speed * .3 + speed * speed / 5.0 <= distance + .05);
    approach_previous = speed;
    std::cout << "5m approach_distance=" << distance << " speed=" << speed << '\n';
  }
  assert(approach_previous == 0.0);
  fast_plan(long_straight);
  for (int i = 0; i < 40; ++i) fast_speed();
  double obstacle_previous = 5.0;
  for (double distance : {6.5, 5.0, 3.0, 1.0}) {
    unsigned int mx, my;
    assert(grid->worldToMap(distance, 0., mx, my));
    grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    const double speed = fast_speed();
    grid->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    assert(speed < obstacle_previous && speed > 0.0);
    assert(speed * .3 + speed * speed / 5.0 <= distance - .2 - .1 + .05);
    obstacle_previous = speed;
    std::cout << "5m obstacle_distance=" << distance << " speed=" << speed << '\n';
  }
  double corner_previous = 5.0;
  for (double corner : {6.0, 4.0, 2.0}) {
    std::vector<std::pair<double, double>> curve;
    for (int i = 0; i <= 70; ++i) curve.emplace_back(corner * i / 70.0, 0.0);
    for (int i = 1; i <= 80; ++i) curve.emplace_back(corner, i * .1);
    fast_plan(curve);
    const double speed = fast_speed();
    assert(speed < corner_previous && speed > 0.0);
    corner_previous = speed;
    std::cout << "5m bend_distance=" << corner << " speed=" << speed << '\n';
  }
  fast_plan(long_straight);
  double recovering = fast_speed();
  assert(recovering <= corner_previous + 5.0 * .9 * .05 + 1e-6);
  for (int i = 0; i < 40; ++i) {
    const double next = fast_speed();
    assert(next >= recovering - 1e-6 && next - recovering <= 5.0 * .9 * .05 + 1e-6);
    recovering = next;
  }
  assert(std::abs(recovering - 5.0) < 1e-6);
  geometry_msgs::msg::Twist spinning;
  spinning.angular.z = 7.0;
  assert(checker.isGoalReached(pose.pose, goal, spinning));
  fast.deactivate();
  fast.cleanup();
  // Native 4 m/s settings: 10 Hz pose feedback, 150--200 ms measurement age,
  // 20 Hz commands and 2.5 m/s2 planar acceleration on a right-angle route.
  for (const auto & setting : std::vector<std::pair<std::string, double>>{
      {"translation_kp", 4.0}, {"translation_ki", 0.1}, {"translation_kd", 0.0},
      {"v_linear_max", 4.0}, {"lookahead_time", 0.6}, {"min_lookahead_dist", 0.4},
      {"max_lookahead_dist", 1.0}, {"obstacle_slowdown_distance", 4.9},
      {"obstacle_deceleration", 2.5}, {"obstacle_reaction_time", 0.3},
      {"obstacle_stop_margin", 0.1}, {"max_lateral_accel", 2.5},
      {"min_approach_linear_velocity", 0.15}, {"approach_velocity_scaling_dist", 4.5},
      {"max_velocity_scaling_factor_rate", 0.9}})
  {
    parent->declare_parameter("Motion." + setting.first, setting.second);
  }
  parent->declare_parameter("Motion.enable_rotation", false);
  parent->declare_parameter("Motion.use_velocity_scaled_lookahead_dist", true);
  pb_omni_pid_pursuit_controller::OmniPidPursuitController motion;
  motion.configure(parent, "Motion", buffer, costmap);
  motion.activate();
  nav_msgs::msg::Path right_angle;
  right_angle.header.frame_id = "odom";
  for (int i = 0; i <= 240; ++i) {
    geometry_msgs::msg::PoseStamped point;
    point.header = right_angle.header;
    point.pose.orientation.w = 1.0;
    point.pose.position.x = std::min(i, 120) * .05;
    point.pose.position.y = std::max(0, i - 120) * .05;
    right_angle.poses.push_back(point);
  }
  struct Sample {double x, y, vx, vy;};
  Sample actual{0., 0., 0., 0.}, measured = actual;
  std::deque<Sample> history(4, actual);
  double closest_corner = 100., maximum_error = 0., beyond_corner = 0.;
  int bend_zeroes = 0, motion_steps = 0;
  for (; motion_steps < 400; ++motion_steps) {
    if (motion_steps % 2 == 0) measured = history.front();
    auto current_pose = pose;
    current_pose.header.stamp = parent->now();
    current_pose.pose.position.x = measured.x;
    current_pose.pose.position.y = measured.y;
    robot_tf.header.stamp = current_pose.header.stamp;
    robot_tf.transform.translation.x = measured.x;
    robot_tf.transform.translation.y = measured.y;
    buffer->setTransform(robot_tf, "test", true);
    moving.linear.x = measured.vx;
    moving.linear.y = measured.vy;
    if (motion_steps % 8 == 0) motion.setPlan(right_angle);
    const auto requested = motion.computeVelocityCommands(current_pose, moving, &checker).twist.linear;
    if (actual.x > 4. && actual.y < 2. && std::hypot(requested.x, requested.y) < 1e-6)
      ++bend_zeroes;
    const double dx = requested.x - actual.vx, dy = requested.y - actual.vy;
    const double scale = std::min(1.0, .125 / std::max(1e-9, std::hypot(dx, dy)));
    const double vx = actual.vx + dx * scale, vy = actual.vy + dy * scale;
    actual.x += (actual.vx + vx) * .025;
    actual.y += (actual.vy + vy) * .025;
    actual.vx = vx;
    actual.vy = vy;
    history.push_back(actual);
    history.pop_front();
    closest_corner = std::min(closest_corner, std::hypot(actual.x - 6., actual.y));
    maximum_error = std::max(maximum_error, std::min(std::abs(actual.y), std::abs(actual.x - 6.)));
    beyond_corner = std::max(beyond_corner, actual.x - 6.);
    if (std::hypot(actual.x - 6., actual.y - 6.) < .1 && std::hypot(vx, vy) < .1) break;
  }
  std::cout << "motion_corner_distance=" << closest_corner << " path_error=" << maximum_error
            << " beyond_corner=" << beyond_corner << " bend_zeroes=" << bend_zeroes << std::endl;
  assert(motion_steps < 400 && closest_corner < .35 && maximum_error < .25);
  assert(beyond_corner < .15 && bend_zeroes == 0);
  motion.deactivate();
  motion.cleanup();
  costmap->cleanup();
  rclcpp::shutdown();
  std::cout << "PASS: approach, position stop, measured stop, clipped path, curve recovery, obstacle braking, sparse paths, reverse, footprint stops and braking before intermediate bends\n";
}
