#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Spin in a separate thread so MoveGroupInterface callbacks are serviced.
  auto node = rclcpp::Node::make_shared(
    "moveit_smoke_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  auto spin_thread = std::thread([&executor]() { executor->spin(); });

  const auto logger = node->get_logger();

  // ── Arm planning group ────────────────────────────────────────────────────
  MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(5);

  // ── Step (a): move to "ready" named pose ─────────────────────────────────
  RCLCPP_INFO(logger, "Step (a): moving to 'ready' pose ...");
  arm.setNamedTarget("ready");
  auto result_a = arm.move();
  if (result_a) {
    RCLCPP_INFO(logger, "Step (a): SUCCESS — arm at 'ready'.");
  } else {
    RCLCPP_ERROR(logger, "Step (a): FAILED (error code %d).", result_a.val);
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  // ── Step (b): Cartesian target (small offset from current pose) ───────────
  RCLCPP_INFO(logger, "Step (b): Cartesian target ...");

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.35;
  target_pose.position.y = 0.10;
  target_pose.position.z = 0.55;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  target_pose.orientation = tf2::toMsg(q);

  arm.setPoseTarget(target_pose);
  auto result_b = arm.move();
  if (result_b) {
    RCLCPP_INFO(logger, "Step (b): SUCCESS — arm reached Cartesian target.");
  } else {
    RCLCPP_WARN(logger, "Step (b): FAILED (error code %d) — IK may be unreachable; continuing.", result_b.val);
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  // ── Step (c): return to "home" ────────────────────────────────────────────
  RCLCPP_INFO(logger, "Step (c): returning to 'home' ...");
  arm.setNamedTarget("home");
  auto result_c = arm.move();
  if (result_c) {
    RCLCPP_INFO(logger, "Step (c): SUCCESS — arm at 'home'.");
  } else {
    RCLCPP_ERROR(logger, "Step (c): FAILED (error code %d).", result_c.val);
  }

  // ── Summary ───────────────────────────────────────────────────────────────
  RCLCPP_INFO(logger,
    "Smoke test complete: (a) %s  (b) %s  (c) %s",
    result_a ? "OK" : "FAIL",
    result_b ? "OK" : "FAIL",
    result_c ? "OK" : "FAIL");

  executor->cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return (result_a && result_c) ? 0 : 1;
}
