/**
 * tool_test.cpp  —  Scripted pick-and-deliver without LLM.
 *
 * Sequence:
 *  1. get_state         — print current scene state
 *  2. navigate_to table — drive to the table
 *  3. detect_objects    — identify cubes
 *  4. pick red_cube     — grasp it
 *  5. navigate_to dispatch_zone
 *  6. place dispatch_zone
 *  7. get_state         — confirm delivery
 *
 * Usage (after phase4.launch.py is running):
 *   ros2 run mobile_manipulator_core tool_test
 */

#include <rclcpp/rclcpp.hpp>
#include "mm_interfaces/srv/get_scene_state.hpp"
#include "mm_interfaces/srv/navigate_to.hpp"
#include "mm_interfaces/srv/detect_objects.hpp"
#include "mm_interfaces/srv/pick.hpp"
#include "mm_interfaces/srv/place.hpp"

using namespace std::chrono_literals;

class ToolTest : public rclcpp::Node
{
public:
  ToolTest() : Node("tool_test")
  {
    state_cli_  = create_client<mm_interfaces::srv::GetSceneState>("tool_server/get_state");
    nav_cli_    = create_client<mm_interfaces::srv::NavigateTo>("tool_server/navigate_to");
    detect_cli_ = create_client<mm_interfaces::srv::DetectObjects>("tool_server/detect_objects");
    pick_cli_   = create_client<mm_interfaces::srv::Pick>("tool_server/pick");
    place_cli_  = create_client<mm_interfaces::srv::Place>("tool_server/place");
  }

  // Must be defined before run() so auto return-type deduction works at the call sites
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
    RCLCPP_INFO(get_logger(), "=== TOOL TEST: pick-and-deliver ===");

    wait_for_services();

    // Step 1: get state
    step_banner(1, "get_state");
    {
      auto req = std::make_shared<mm_interfaces::srv::GetSceneState::Request>();
      auto res = call_blocking(state_cli_, req);
      RCLCPP_INFO(get_logger(), "Scene state:\n%s", res->state_text.c_str());
    }

    // Step 2: navigate to table
    step_banner(2, "navigate_to table");
    {
      auto req = std::make_shared<mm_interfaces::srv::NavigateTo::Request>();
      req->location = "table";
      auto res = call_blocking(nav_cli_, req);
      print_result("navigate_to", res->success, res->message);
      if (!res->success) { RCLCPP_ERROR(get_logger(), "Aborting."); return; }
    }

    // Step 3: detect objects
    step_banner(3, "detect_objects");
    {
      auto req = std::make_shared<mm_interfaces::srv::DetectObjects::Request>();
      auto res = call_blocking(detect_cli_, req);
      print_result("detect_objects", res->success, res->message);
      for (const auto& d : res->detections) {
        RCLCPP_INFO(get_logger(), "  [%s] %.2f  pos=(%.2f, %.2f, %.2f)",
                    d.label.c_str(), d.confidence, d.x, d.y, d.z);
      }
    }

    // Step 4: pick red_cube
    step_banner(4, "pick red_cube");
    {
      auto req = std::make_shared<mm_interfaces::srv::Pick::Request>();
      req->object_label = "red_cube";
      auto res = call_blocking(pick_cli_, req);
      print_result("pick", res->success, res->message);
      if (!res->success) { RCLCPP_ERROR(get_logger(), "Aborting."); return; }
    }

    // Step 5: navigate to dispatch_zone
    step_banner(5, "navigate_to dispatch_zone");
    {
      auto req = std::make_shared<mm_interfaces::srv::NavigateTo::Request>();
      req->location = "dispatch_zone";
      auto res = call_blocking(nav_cli_, req);
      print_result("navigate_to", res->success, res->message);
      if (!res->success) { RCLCPP_ERROR(get_logger(), "Aborting."); return; }
    }

    // Step 6: place
    step_banner(6, "place dispatch_zone");
    {
      auto req = std::make_shared<mm_interfaces::srv::Place::Request>();
      req->location_name = "dispatch_zone";
      auto res = call_blocking(place_cli_, req);
      print_result("place", res->success, res->message);
    }

    // Step 7: final state
    step_banner(7, "get_state (final)");
    {
      auto req = std::make_shared<mm_interfaces::srv::GetSceneState::Request>();
      auto res = call_blocking(state_cli_, req);
      RCLCPP_INFO(get_logger(), "Final scene state:\n%s", res->state_text.c_str());
    }

    RCLCPP_INFO(get_logger(), "=== TOOL TEST COMPLETE ===");
  }

private:
  void wait_for_services()
  {
    RCLCPP_INFO(get_logger(), "Waiting for tool_server services...");
    state_cli_->wait_for_service();
    nav_cli_->wait_for_service();
    detect_cli_->wait_for_service();
    pick_cli_->wait_for_service();
    place_cli_->wait_for_service();
    RCLCPP_INFO(get_logger(), "All services ready.");
  }

  static void step_banner(int n, const char* name)
  {
    printf("\n──────────────────────────────────────────\n");
    printf("  STEP %d: %s\n", n, name);
    printf("──────────────────────────────────────────\n");
  }

  static void print_result(const char* tool, bool ok, const std::string& msg)
  {
    printf("  [%s] %s  —  %s\n", tool, ok ? "OK " : "FAIL", msg.c_str());
  }

  rclcpp::Client<mm_interfaces::srv::GetSceneState>::SharedPtr state_cli_;
  rclcpp::Client<mm_interfaces::srv::NavigateTo>::SharedPtr    nav_cli_;
  rclcpp::Client<mm_interfaces::srv::DetectObjects>::SharedPtr detect_cli_;
  rclcpp::Client<mm_interfaces::srv::Pick>::SharedPtr          pick_cli_;
  rclcpp::Client<mm_interfaces::srv::Place>::SharedPtr         place_cli_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolTest>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
