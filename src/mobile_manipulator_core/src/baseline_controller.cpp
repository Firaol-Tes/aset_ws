/**
 * baseline_controller.cpp — Traditional/hardcoded pick-and-deliver baseline.
 *
 * Performs the SAME task as Phase 5 Scenario 1 (pick the red cube from the
 * table, deliver it to the dispatch zone) using the SAME underlying tool
 * services as the LLM planner — so the comparison in the paper is about
 * decision-making, not capability. The difference is entirely in how the
 * next action is chosen:
 *
 *   - Fixed, pre-programmed sequence (no LLM, no branching on scene state).
 *   - Picks at a HARDCODED position (see HARDCODED_CUBE_* below), via
 *     pick_at_pose — never calls detect_objects. A real traditional robot
 *     is programmed once against a measured workspace, not re-perceived
 *     each run.
 *   - No replanning: any step failure stops the run immediately. A
 *     hardcoded sequence has no fallback action to try instead.
 *
 * This is also the foundation for Phase 6's disturbance experiment: since
 * the cube position is a compile-time constant rather than read from
 * perception, moving the real cube mid-task will NOT be reflected here,
 * and the pick will fail at the stale coordinates — that mismatch is the
 * intended comparison point against the LLM planner's perception-driven
 * recovery.
 *
 * Usage (after phase4.launch.py is running):
 *   ros2 run mobile_manipulator_core baseline_controller
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "mm_interfaces/srv/navigate_to.hpp"
#include "mm_interfaces/srv/drive_distance.hpp"
#include "mm_interfaces/srv/pick_at_pose.hpp"
#include "mm_interfaces/srv/place.hpp"

using namespace std::chrono_literals;

// ── Hardcoded task parameters — edit here for the disturbance experiment ──
// Matches red_cube's spawn pose in warehouse.sdf: <pose>2.0 0.12 0.40 0 0 0</pose>
// (model centre); pick targets the top face, +half the 4cm cube height. This
// is the TRUE position — not fudged — see APPROACH_NUDGE_M below for why a
// fixed approach step is needed before it's reachable.
namespace baseline_params {
  constexpr const char* PICK_LOCATION    = "table";
  constexpr const char* DELIVER_LOCATION = "dispatch_zone";
  constexpr const char* OBJECT_LABEL     = "red_cube";
  constexpr double HARDCODED_CUBE_X = 2.0;
  constexpr double HARDCODED_CUBE_Y = 0.12;
  constexpr double HARDCODED_CUBE_Z = 0.42;

  // red_cube sits right at the edge of arm reach from navigate_to(table)'s
  // parking pose (confirmed: the LLM planner's first pick attempt at this
  // same position also failed with "out of reach" in Scenario 1, before it
  // recovered by driving closer). A real commissioning engineer would
  // calibrate this once rather than ship an untested coordinate — this
  // fixed approach step is that calibration, baked in as a constant. It's
  // still 100% non-adaptive: a fixed distance, not a response to anything
  // perceived.
  constexpr double APPROACH_NUDGE_M = 0.15;
}

struct StepResult {
  std::string step;
  bool success;
  std::string message;
  long duration_ms;
};

static std::string json_escape(const std::string& s)
{
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

class BaselineController : public rclcpp::Node
{
public:
  BaselineController() : Node("baseline_controller")
  {
    nav_cli_   = create_client<mm_interfaces::srv::NavigateTo>("tool_server/navigate_to");
    drive_cli_ = create_client<mm_interfaces::srv::DriveDistance>("tool_server/drive_distance");
    pick_cli_  = create_client<mm_interfaces::srv::PickAtPose>("tool_server/pick_at_pose");
    place_cli_ = create_client<mm_interfaces::srv::Place>("tool_server/place");
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

  void run()
  {
    using namespace baseline_params;
    RCLCPP_INFO(get_logger(), "=== BASELINE CONTROLLER: hardcoded pick-and-deliver ===");
    const std::string task = std::string("pick up ") + OBJECT_LABEL + " and place it in " + DELIVER_LOCATION;

    RCLCPP_INFO(get_logger(), "Waiting for tool_server services...");
    nav_cli_->wait_for_service();
    drive_cli_->wait_for_service();
    pick_cli_->wait_for_service();
    place_cli_->wait_for_service();
    RCLCPP_INFO(get_logger(), "All services ready.");

    auto t0 = std::chrono::steady_clock::now();
    std::vector<StepResult> steps;
    bool success = true;

    // Step 1: navigate to the (hardcoded) pick location
    success = run_step(steps, "navigate_to", [&] {
      auto req = std::make_shared<mm_interfaces::srv::NavigateTo::Request>();
      req->location = PICK_LOCATION;
      auto res = call_blocking(nav_cli_, req);
      return std::make_pair(res->success, res->message);
    });

    // Step 2: fixed approach nudge — see APPROACH_NUDGE_M comment above.
    if (success) {
      success = run_step(steps, "drive_distance", [&] {
        auto req = std::make_shared<mm_interfaces::srv::DriveDistance::Request>();
        req->distance_m = APPROACH_NUDGE_M;
        auto res = call_blocking(drive_cli_, req);
        return std::make_pair(res->success, res->message);
      });
    }

    // Step 3: pick at the hardcoded cube position — NOT perception-driven.
    if (success) {
      success = run_step(steps, "pick_at_pose", [&] {
        auto req = std::make_shared<mm_interfaces::srv::PickAtPose::Request>();
        req->object_label = OBJECT_LABEL;
        req->x = HARDCODED_CUBE_X;
        req->y = HARDCODED_CUBE_Y;
        req->z = HARDCODED_CUBE_Z;
        auto res = call_blocking(pick_cli_, req);
        return std::make_pair(res->success, res->message);
      });
    }

    // Step 4: navigate to the (hardcoded) delivery location
    if (success) {
      success = run_step(steps, "navigate_to", [&] {
        auto req = std::make_shared<mm_interfaces::srv::NavigateTo::Request>();
        req->location = DELIVER_LOCATION;
        auto res = call_blocking(nav_cli_, req);
        return std::make_pair(res->success, res->message);
      });
    }

    // Step 5: place
    if (success) {
      success = run_step(steps, "place", [&] {
        auto req = std::make_shared<mm_interfaces::srv::Place::Request>();
        req->location_name = DELIVER_LOCATION;
        auto res = call_blocking(place_cli_, req);
        return std::make_pair(res->success, res->message);
      });
    }

    double duration_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count() / 1000.0;

    RCLCPP_INFO(get_logger(), "=== BASELINE CONTROLLER %s (%.1fs) ===",
                success ? "SUCCEEDED" : "FAILED", duration_sec);

    log_run(task, success, duration_sec, steps);
  }

private:
  // No retries, no fallback — runs `fn`, records the step, and returns
  // whether the WHOLE run should keep going. num_recovery_attempts in the
  // log is always 0 because there is nothing here that retries.
  template<typename Fn>
  bool run_step(std::vector<StepResult>& steps, const std::string& name, Fn fn)
  {
    auto t0 = std::chrono::steady_clock::now();
    auto [ok, msg] = fn();
    long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(get_logger(), "[%s] %s — %s", name.c_str(), ok ? "OK" : "FAIL", msg.c_str());
    steps.push_back({name, ok, msg, ms});
    return ok;
  }

  static void log_run(const std::string& task, bool success, double duration_sec,
                       const std::vector<StepResult>& steps)
  {
    const char* home = std::getenv("HOME");
    std::filesystem::path log_dir = std::filesystem::path(home ? home : ".") / "aset_ws" / "baseline_logs";
    std::filesystem::create_directories(log_dir);
    std::filesystem::path log_path = log_dir / "baseline_results.csv";

    bool write_header = !std::filesystem::exists(log_path);
    std::ofstream f(log_path, std::ios::app);
    if (write_header) {
      f << "timestamp,task,success,duration_sec,num_recovery_attempts,steps_json\n";
    }

    std::ostringstream steps_json;
    steps_json << "[";
    for (size_t i = 0; i < steps.size(); ++i) {
      if (i > 0) steps_json << ",";
      steps_json << "{\"step\":\"" << json_escape(steps[i].step) << "\","
                 << "\"success\":" << (steps[i].success ? "true" : "false") << ","
                 << "\"message\":\"" << json_escape(steps[i].message) << "\","
                 << "\"duration_ms\":" << steps[i].duration_ms << "}";
    }
    steps_json << "]";

    f << std::fixed << std::chrono::duration_cast<std::chrono::seconds>(
           std::chrono::system_clock::now().time_since_epoch()).count()
      << ",\"" << json_escape(task) << "\""
      << "," << (success ? "true" : "false")
      << "," << duration_sec
      << ",0"
      << ",\"" << json_escape(steps_json.str()) << "\"\n";
    f.close();

    std::cout << "Logged to " << log_path.string() << std::endl;
  }

  rclcpp::Client<mm_interfaces::srv::NavigateTo>::SharedPtr    nav_cli_;
  rclcpp::Client<mm_interfaces::srv::DriveDistance>::SharedPtr drive_cli_;
  rclcpp::Client<mm_interfaces::srv::PickAtPose>::SharedPtr    pick_cli_;
  rclcpp::Client<mm_interfaces::srv::Place>::SharedPtr         place_cli_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaselineController>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
