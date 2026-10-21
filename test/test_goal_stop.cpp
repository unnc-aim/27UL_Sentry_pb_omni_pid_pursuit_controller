// Check the real controller together with the native stopped goal checker.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
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
  controller.deactivate();
  controller.cleanup();
  costmap->cleanup();
  rclcpp::shutdown();
  std::cout << "PASS: approach, position stop, measured stop, clipped path, curve recovery, obstacle braking, sparse paths, reverse, footprint stops and braking before intermediate bends\n";
}
