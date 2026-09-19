/**
 * experiment_runner.cpp — Phase 6 Part 2: runs a chosen scenario for a
 * chosen system ("llm" or "baseline"), N times, with optional mid-task
 * disturbance (cube teleported away after a fixed delay), logging results
 * to CSV for the paper.
 *
 * Usage:
 *   ros2 run mobile_manipulator_core experiment_runner <scenario_id> <system> <num_trials>
 *   e.g. ros2 run mobile_manipulator_core experiment_runner S1 baseline 3
 *
 * "baseline" reuses the exact same fixed 5-step sequence as
 * baseline_controller.cpp (duplicated here rather than shared via a header
 * -- it's ~5 lines of orchestration per step, not worth abstracting). It
 * ignores the scenario's natural-language command entirely, same as the
 * standalone baseline_controller -- a hardcoded system has no way to act on
 * "bring the red cube" vs "move all cubes" differently. Running baseline on
 * S2/S3/S5 is expected to produce a degenerate result; that limitation IS
 * the comparison point, not a bug.
 *
 * "llm" calls mm_llm_planner's /execute_task action with the scenario's
 * command and reads back its self-reported success/num_llm_calls/
 * num_replans -- the same metrics already validated in Phase 5.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "mm_interfaces/srv/navigate_to.hpp"
#include "mm_interfaces/srv/drive_distance.hpp"
#include "mm_interfaces/srv/pick_at_pose.hpp"
#include "mm_interfaces/srv/place.hpp"
#include "mm_interfaces/srv/open_gripper.hpp"
#include "mm_interfaces/srv/move_arm_to_named.hpp"
#include "mm_interfaces/action/execute_task.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "mobile_manipulator_core/gz_teleport.hpp"
#include "mobile_manipulator_core/gz_pose.hpp"
#include "mobile_manipulator_core/baseline_params.hpp"

using namespace std::chrono_literals;
using ExecuteTask = mm_interfaces::action::ExecuteTask;

namespace baseline_task {
  // Mirrors baseline_controller.cpp's baseline_params exactly -- same
  // hardcoded values, same reasoning (see that file's comments). Kept as a
  // short, separate constant block rather than a shared header: this is
  // ~6 lines, not worth the indirection.
  // Aliases onto baseline_params.hpp -- the same constants baseline_controller
  // uses. This sequence is a copy of that controller's, so it must not carry
  // its own values: the nudge here was left at 0.15 while the controller was
  // calibrated to 0.30, which made the two incomparable.
  constexpr const char* PICK_LOCATION    = baseline_params::PICK_LOCATION;
  constexpr const char* DELIVER_LOCATION = baseline_params::DELIVER_LOCATION;
  constexpr const char* OBJECT_LABEL     = baseline_params::RED_LABEL;
  constexpr double HARDCODED_CUBE_X = baseline_params::RED_X;
  constexpr double HARDCODED_CUBE_Y = baseline_params::RED_Y;
  constexpr double HARDCODED_CUBE_Z = baseline_params::RED_Z;
  constexpr double APPROACH_NUDGE_M = baseline_params::APPROACH_NUDGE_M;
}

namespace world_state {
  constexpr const char* WORLD_NAME = "warehouse";
  struct EntityPose { const char* name; double x, y, z; };
  // Ground-truth spawn poses from warehouse.sdf -- used both to restore
  // cubes between trials and as the basis for disturbance target sanity.
  constexpr EntityPose CUBE_SPAWNS[] = {
    {"red_cube",   2.0,  0.12, 0.40},
    {"green_cube", 2.0,  0.00, 0.40},
    {"blue_cube",  2.0, -0.12, 0.40},
  };

  // Dispatch zone geometry, read off the floor patch in warehouse.sdf
  // (<model name="dispatch_zone"> at -2.0 0, a 0.8 x 0.8 box). Used to
  // check delivery against the world instead of trusting the system under
  // test to report its own success.
  constexpr double DISPATCH_X      = -2.0;
  constexpr double DISPATCH_Y      =  0.0;
  constexpr double DISPATCH_HALF   =  0.4;
  // A delivered cube rests on the floor. The bound rejects the case where
  // the robot is parked over the zone still holding the cube at arm height
  // (table pick height is 0.40 m, so 0.20 m clears it with margin).
  constexpr double DISPATCH_MAX_Z  =  0.20;
}

struct ScenarioConfig {
  std::string id;
  std::string command;
  bool disturbance_enabled = false;
  std::string disturbance_target;
  double disturbance_delay_sec = 0.0;
  double disturbance_x = 0.0, disturbance_y = 0.0, disturbance_z = 0.0;
};

static ScenarioConfig load_scenario(const std::string& path, const std::string& scenario_id)
{
  YAML::Node root = YAML::LoadFile(path);
  YAML::Node node = root["scenarios"][scenario_id];
  if (!node) {
    throw std::runtime_error("Unknown scenario id: " + scenario_id);
  }
  ScenarioConfig cfg;
  cfg.id = scenario_id;
  cfg.command = node["command"].as<std::string>();
  YAML::Node dist = node["disturbance"];
  if (dist && dist["enabled"] && dist["enabled"].as<bool>()) {
    cfg.disturbance_enabled = true;
    cfg.disturbance_target = dist["target_object"].as<std::string>();
    cfg.disturbance_delay_sec = dist["delay_sec"].as<double>();
    cfg.disturbance_x = dist["new_position"]["x"].as<double>();
    cfg.disturbance_y = dist["new_position"]["y"].as<double>();
    cfg.disturbance_z = dist["new_position"]["z"].as<double>();
  }
  return cfg;
}

static std::string json_escape(const std::string& s)
{
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

struct TrialResult {
  bool success = false;
  double total_time_sec = 0.0;
  double path_length_m = 0.0;
  int num_llm_calls = 0;
  int num_replans = 0;
  double time_to_recover_sec = -1.0;  // -1 = not applicable (no disturbance, or failed)
  // Independent, simulator-side check of the delivery, recorded alongside
  // -- never instead of -- the system's self-reported `success`, so the two
  // can be compared and earlier runs stay interpretable.
  //  1 = cube verified inside the dispatch zone
  //  0 = cube verified outside it
  // -1 = could not read the cube pose (verification unavailable)
  int delivered_verified = -1;
  double delivered_x = 0.0, delivered_y = 0.0, delivered_z = 0.0;
  std::string message;
};

class ExperimentRunner : public rclcpp::Node
{
public:
  ExperimentRunner() : Node("experiment_runner")
  {
    nav_cli_   = create_client<mm_interfaces::srv::NavigateTo>("tool_server/navigate_to");
    drive_cli_ = create_client<mm_interfaces::srv::DriveDistance>("tool_server/drive_distance");
    pick_cli_  = create_client<mm_interfaces::srv::PickAtPose>("tool_server/pick_at_pose");
    place_cli_ = create_client<mm_interfaces::srv::Place>("tool_server/place");
    open_cli_  = create_client<mm_interfaces::srv::OpenGripper>("tool_server/open_gripper");
    move_arm_cli_ = create_client<mm_interfaces::srv::MoveArmToNamed>("tool_server/move_arm_to_named");
    execute_task_cli_ = rclcpp_action::create_client<ExecuteTask>(this, "/execute_task");

    for (const auto& cube : world_state::CUBE_SPAWNS) {
      release_pubs_[cube.name] = create_publisher<std_msgs::msg::Empty>(
        std::string("/release/") + cube.name, 1);
    }

    declare_parameter("odom_topic", "/diff_drive_controller/odom");
    std::string odom_topic = get_parameter("odom_topic").as_string();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 50,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });
  }

  template<typename Cli, typename Req>
  auto call_blocking(Cli& cli, std::shared_ptr<Req> req)
  {
    auto future = cli->async_send_request(req);
    while (rclcpp::ok()) {
      if (future.wait_for(100ms) == std::future_status::ready) break;
      rclcpp::spin_some(shared_from_this());
    }
    return future.get();
  }

  void wait_for_services()
  {
    RCLCPP_INFO(get_logger(), "Waiting for tool_server services...");
    nav_cli_->wait_for_service();
    drive_cli_->wait_for_service();
    pick_cli_->wait_for_service();
    place_cli_->wait_for_service();
    open_cli_->wait_for_service();
    move_arm_cli_->wait_for_service();
    RCLCPP_INFO(get_logger(), "Tool services ready.");
  }

  // ── World reset between trials ──────────────────────────────────────────
  // Cubes are simple rigid bodies -- teleport is clean. The robot is a
  // complex articulated entity (arm joints, odometry, AMCL pose) where a
  // raw entity teleport would NOT reset joint positions or localization --
  // so instead of teleporting the robot, send it home via the
  // already-proven navigate_to/move_arm_to_named services.
  void reset_world()
  {
    RCLCPP_INFO(get_logger(), "[reset] Releasing any held object, opening gripper...");
    for (auto& [label, pub] : release_pubs_) {
      pub->publish(std_msgs::msg::Empty());
    }
    auto open_req = std::make_shared<mm_interfaces::srv::OpenGripper::Request>();
    call_blocking(open_cli_, open_req);

    // Force the arm back to a known-clean joint configuration. Without
    // this, the arm is left wherever the previous trial's last motion put
    // it -- usually fine, but repeated IK solves across trials can drift it
    // into a contorted pose that no longer has a valid path to "ready",
    // which then fails every subsequent trial identically until the whole
    // stack is restarted. Resetting joints explicitly here, not just the
    // base pose, removes that cross-trial coupling.
    RCLCPP_INFO(get_logger(), "[reset] Moving arm to home configuration...");
    auto arm_req = std::make_shared<mm_interfaces::srv::MoveArmToNamed::Request>();
    arm_req->pose_name = "home";
    call_blocking(move_arm_cli_, arm_req);

    RCLCPP_INFO(get_logger(), "[reset] Restoring cube spawn positions...");
    for (const auto& cube : world_state::CUBE_SPAWNS) {
      bool ok = gz_teleport::teleport(world_state::WORLD_NAME, cube.name, cube.x, cube.y, cube.z);
      if (!ok) {
        RCLCPP_WARN(get_logger(), "[reset] Failed to reset %s position", cube.name);
      }
    }
    std::this_thread::sleep_for(500ms);  // let physics settle

    RCLCPP_INFO(get_logger(), "[reset] Sending robot home...");
    auto nav_req = std::make_shared<mm_interfaces::srv::NavigateTo::Request>();
    nav_req->location = "home";
    call_blocking(nav_cli_, nav_req);
  }

  // ── One trial ────────────────────────────────────────────────────────────
  TrialResult run_trial(const ScenarioConfig& cfg, const std::string& system)
  {
    TrialResult result;
    path_length_m_ = 0.0;
    have_last_odom_ = false;
    auto t0 = std::chrono::steady_clock::now();

    // Fire-and-forget disturbance thread: sleeps a fixed delay (see
    // scenarios.yaml comment -- a fixed delay is the explicitly-allowed
    // simpler alternative to live odometry-threshold monitoring), then
    // teleports the target cube. Started right before the blocking
    // run, joined after -- runs concurrently with it either way.
    std::thread disturbance_thread;
    std::chrono::steady_clock::time_point disturbance_fired_at;
    bool disturbance_fired = false;
    if (cfg.disturbance_enabled) {
      disturbance_thread = std::thread([&]() {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<long>(cfg.disturbance_delay_sec * 1000)));
        bool ok = gz_teleport::teleport(world_state::WORLD_NAME, cfg.disturbance_target,
                                         cfg.disturbance_x, cfg.disturbance_y, cfg.disturbance_z);
        disturbance_fired_at = std::chrono::steady_clock::now();
        disturbance_fired = true;
        RCLCPP_INFO(get_logger(), "[disturbance] Teleported %s -> (%.2f, %.2f, %.2f): %s",
                    cfg.disturbance_target.c_str(), cfg.disturbance_x, cfg.disturbance_y,
                    cfg.disturbance_z, ok ? "OK" : "FAILED");
      });
    }

    if (system == "baseline") {
      run_baseline_sequence(result);
    } else if (system == "llm") {
      run_llm_sequence(cfg, result);
    } else {
      result.success = false;
      result.message = "Unknown system: " + system;
    }

    if (disturbance_thread.joinable()) disturbance_thread.join();

    result.total_time_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count() / 1000.0;
    result.path_length_m = path_length_m_;

    verify_delivery(result);

    if (cfg.disturbance_enabled && disturbance_fired && result.success) {
      result.time_to_recover_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - disturbance_fired_at).count() / 1000.0;
    }

    return result;
  }

private:
  // Reads the target cube's true world pose and records whether it is
  // inside the dispatch zone. Does not modify `success`: self-reported and
  // verified outcomes are logged side by side so a disagreement between
  // them is visible in the data rather than silently resolved here.
  void verify_delivery(TrialResult& result)
  {
    gz_pose::Pose p;
    const bool inside = gz_pose::within_zone(
      world_state::WORLD_NAME, baseline_task::OBJECT_LABEL,
      world_state::DISPATCH_X, world_state::DISPATCH_Y,
      world_state::DISPATCH_HALF, world_state::DISPATCH_MAX_Z, p);

    if (!inside && (p.x == 0.0 && p.y == 0.0 && p.z == 0.0)) {
      RCLCPP_WARN(get_logger(),
                  "[verify] Could not read %s pose -- delivery unverified",
                  baseline_task::OBJECT_LABEL);
      result.delivered_verified = -1;
      return;
    }

    result.delivered_verified = inside ? 1 : 0;
    result.delivered_x = p.x;
    result.delivered_y = p.y;
    result.delivered_z = p.z;

    RCLCPP_INFO(get_logger(),
                "[verify] %s at (%.2f, %.2f, %.2f) -> %s dispatch zone "
                "(self-reported success=%s)",
                baseline_task::OBJECT_LABEL, p.x, p.y, p.z,
                inside ? "INSIDE" : "OUTSIDE", result.success ? "true" : "false");

    if (inside != result.success) {
      RCLCPP_WARN(get_logger(),
                  "[verify] MISMATCH: self-reported success=%s but cube is %s "
                  "the dispatch zone",
                  result.success ? "true" : "false", inside ? "inside" : "outside");
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    if (have_last_odom_) {
      double dx = x - last_x_, dy = y - last_y_;
      path_length_m_ += std::sqrt(dx * dx + dy * dy);
    }
    last_x_ = x;
    last_y_ = y;
    have_last_odom_ = true;
  }

  void run_baseline_sequence(TrialResult& result)
  {
    using namespace baseline_task;

    auto nav_req = std::make_shared<mm_interfaces::srv::NavigateTo::Request>();
    nav_req->location = PICK_LOCATION;
    auto nav_res = call_blocking(nav_cli_, nav_req);
    if (!nav_res->success) {
      result.success = false; result.message = "navigate_to(table): " + nav_res->message; return;
    }

    auto drive_req = std::make_shared<mm_interfaces::srv::DriveDistance::Request>();
    drive_req->distance_m = APPROACH_NUDGE_M;
    auto drive_res = call_blocking(drive_cli_, drive_req);
    if (!drive_res->success) {
      result.success = false; result.message = "drive_distance: " + drive_res->message; return;
    }

    auto pick_req = std::make_shared<mm_interfaces::srv::PickAtPose::Request>();
    pick_req->object_label = OBJECT_LABEL;
    pick_req->x = HARDCODED_CUBE_X;
    pick_req->y = HARDCODED_CUBE_Y;
    pick_req->z = HARDCODED_CUBE_Z;
    auto pick_res = call_blocking(pick_cli_, pick_req);
    if (!pick_res->success) {
      result.success = false; result.message = "pick_at_pose: " + pick_res->message; return;
    }

    auto nav2_req = std::make_shared<mm_interfaces::srv::NavigateTo::Request>();
    nav2_req->location = DELIVER_LOCATION;
    auto nav2_res = call_blocking(nav_cli_, nav2_req);
    if (!nav2_res->success) {
      result.success = false; result.message = "navigate_to(dispatch): " + nav2_res->message; return;
    }

    auto place_req = std::make_shared<mm_interfaces::srv::Place::Request>();
    place_req->location_name = DELIVER_LOCATION;
    auto place_res = call_blocking(place_cli_, place_req);
    result.success = place_res->success;
    result.message = place_res->message;
  }

  void run_llm_sequence(const ScenarioConfig& cfg, TrialResult& result)
  {
    if (!execute_task_cli_->wait_for_action_server(5s)) {
      result.success = false;
      result.message = "execute_task action server not available";
      return;
    }

    auto goal = ExecuteTask::Goal();
    goal.command = cfg.command;

    auto goal_handle_future = execute_task_cli_->async_send_goal(goal);
    while (rclcpp::ok() && goal_handle_future.wait_for(100ms) != std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      result.success = false;
      result.message = "execute_task goal rejected";
      return;
    }

    auto result_future = execute_task_cli_->async_get_result(goal_handle);
    while (rclcpp::ok() && result_future.wait_for(100ms) != std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
    }
    auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      result.success = false;
      result.message = "execute_task did not complete normally";
      return;
    }

    result.success = wrapped.result->success;
    result.message = wrapped.result->summary;
    result.num_llm_calls = wrapped.result->num_llm_calls;
    result.num_replans = wrapped.result->num_replans;
  }

  rclcpp::Client<mm_interfaces::srv::NavigateTo>::SharedPtr    nav_cli_;
  rclcpp::Client<mm_interfaces::srv::DriveDistance>::SharedPtr drive_cli_;
  rclcpp::Client<mm_interfaces::srv::PickAtPose>::SharedPtr    pick_cli_;
  rclcpp::Client<mm_interfaces::srv::Place>::SharedPtr         place_cli_;
  rclcpp::Client<mm_interfaces::srv::OpenGripper>::SharedPtr   open_cli_;
  rclcpp::Client<mm_interfaces::srv::MoveArmToNamed>::SharedPtr move_arm_cli_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr                execute_task_cli_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> release_pubs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double path_length_m_ = 0.0;
  double last_x_ = 0.0, last_y_ = 0.0;
  bool have_last_odom_ = false;
};

static void log_csv(const std::string& path, const std::string& scenario,
                     const std::string& system, int trial, const TrialResult& r)
{
  bool need_header = !std::filesystem::exists(path);
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  std::ofstream f(path, std::ios::app);
  if (need_header) {
    f << "timestamp,scenario,system,trial,success,total_time_sec,path_length_m,"
         "num_llm_calls,num_replans,time_to_recover_sec,"
         "delivered_verified,delivered_x,delivered_y,delivered_z,message\n";
  }
  f << std::fixed << std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0
    << "," << scenario
    << "," << system
    << "," << trial
    << "," << (r.success ? "1" : "0")
    << "," << r.total_time_sec
    << "," << r.path_length_m
    << "," << r.num_llm_calls
    << "," << r.num_replans
    << "," << r.time_to_recover_sec
    << "," << r.delivered_verified
    << "," << r.delivered_x
    << "," << r.delivered_y
    << "," << r.delivered_z
    << ",\"" << json_escape(r.message) << "\"\n";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 4) {
    std::cerr << "Usage: experiment_runner <scenario_id> <llm|baseline> <num_trials>\n";
    rclcpp::shutdown();
    return 1;
  }
  std::string scenario_id = argv[1];
  std::string system = argv[2];
  int num_trials = std::atoi(argv[3]);

  auto node = std::make_shared<ExperimentRunner>();

  std::string scenarios_path =
    "/home/f/aset_ws/install/mobile_manipulator_core/share/mobile_manipulator_core/config/scenarios.yaml";
  ScenarioConfig cfg;
  try {
    cfg = load_scenario(scenarios_path, scenario_id);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load scenario: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "=== EXPERIMENT RUNNER: %s system=%s trials=%d ===",
              scenario_id.c_str(), system.c_str(), num_trials);
  RCLCPP_INFO(node->get_logger(), "Command: '%s'  disturbance=%s",
              cfg.command.c_str(), cfg.disturbance_enabled ? "ON" : "off");

  node->wait_for_services();

  std::string csv_path = std::string(std::getenv("HOME")) + "/aset_ws/experiment_logs/experiment_results.csv";

  for (int trial = 1; trial <= num_trials; ++trial) {
    RCLCPP_INFO(node->get_logger(), "--- Trial %d/%d: resetting world ---", trial, num_trials);
    node->reset_world();

    RCLCPP_INFO(node->get_logger(), "--- Trial %d/%d: running ---", trial, num_trials);
    auto result = node->run_trial(cfg, system);

    RCLCPP_INFO(node->get_logger(), "--- Trial %d/%d: %s (%.1fs, path=%.2fm) %s ---",
                trial, num_trials, result.success ? "SUCCESS" : "FAIL",
                result.total_time_sec, result.path_length_m, result.message.c_str());

    log_csv(csv_path, scenario_id, system, trial, result);
  }

  RCLCPP_INFO(node->get_logger(), "=== EXPERIMENT COMPLETE — logged to %s ===", csv_path.c_str());

  rclcpp::shutdown();
  return 0;
}
