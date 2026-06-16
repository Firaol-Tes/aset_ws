/**
 * tool_server_node.cpp  —  Unified tool server for the mobile manipulator.
 *
 * Provides services (all under /tool_server/...):
 *   navigate_to        mm_interfaces/NavigateTo
 *   detect_objects     mm_interfaces/DetectObjects
 *   move_arm_to_named  mm_interfaces/MoveArmToNamed
 *   move_arm_to_pose   mm_interfaces/MoveArmToPose
 *   pick               mm_interfaces/Pick
 *   place              mm_interfaces/Place
 *   open_gripper       mm_interfaces/OpenGripper
 *   close_gripper      mm_interfaces/CloseGripper
 *   get_state          mm_interfaces/GetSceneState
 *
 * Logs every call to /tmp/tool_log_<timestamp>.csv
 */

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "mm_interfaces/msg/detection_array.hpp"
#include "mm_interfaces/srv/navigate_to.hpp"
#include "mm_interfaces/srv/detect_objects.hpp"
#include "mm_interfaces/srv/move_arm_to_named.hpp"
#include "mm_interfaces/srv/move_arm_to_pose.hpp"
#include "mm_interfaces/srv/pick.hpp"
#include "mm_interfaces/srv/place.hpp"
#include "mm_interfaces/srv/open_gripper.hpp"
#include "mm_interfaces/srv/close_gripper.hpp"
#include "mm_interfaces/srv/get_scene_state.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Location { double x, y, yaw; };

class ToolServer : public rclcpp::Node
{
public:
  explicit ToolServer()
  : Node("tool_server_node",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // ── Nav2 action client — MUST be in a separate Reentrant callback group.
    // All callbacks default to the node's MutuallyExclusive group, which means
    // only one callback runs at a time.  When a blocking service callback holds
    // that group, the action client's goal_response/result callbacks can never
    // fire.  A Reentrant group allows them to run concurrently.
    nav_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose", nav_cb_group_);

    // ── TF + joint states for get_state ────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    joint_sub_   = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr m) { latest_joints_ = m; });

    // ── Grasp/release publishers (direct, one per known object) ────────────
    for (const auto& obj : {"red_cube", "green_cube", "blue_cube"}) {
      std::string s(obj);
      grasp_pubs_[s]   = create_publisher<std_msgs::msg::Bool>("/grasp/"   + s, 1);
      release_pubs_[s] = create_publisher<std_msgs::msg::Bool>("/release/" + s, 1);
    }

    // ── Detection subscriber ────────────────────────────────────────────────
    detect_sub_ = create_subscription<mm_interfaces::msg::DetectionArray>(
      "/detected_objects", 10,
      [this](mm_interfaces::msg::DetectionArray::SharedPtr m) {
        latest_detections_ = m;
      });

    // ── Predefined locations ────────────────────────────────────────────────
    locations_["table"]         = {1.6,  0.0,  0.0};
    locations_["dispatch_zone"] = {-1.8, 0.0,  3.14159};
    locations_["shelf_area"]    = {0.0,  1.5,  1.5708};
    locations_["home"]          = {0.0,  0.0,  0.0};

    // ── Services ───────────────────────────────────────────────────────────
    using namespace std::placeholders;
    nav_srv_ = create_service<mm_interfaces::srv::NavigateTo>(
      "tool_server/navigate_to",
      std::bind(&ToolServer::nav_cb, this, _1, _2));
    detect_srv_ = create_service<mm_interfaces::srv::DetectObjects>(
      "tool_server/detect_objects",
      std::bind(&ToolServer::detect_cb, this, _1, _2));
    arm_named_srv_ = create_service<mm_interfaces::srv::MoveArmToNamed>(
      "tool_server/move_arm_to_named",
      std::bind(&ToolServer::arm_named_cb, this, _1, _2));
    arm_pose_srv_ = create_service<mm_interfaces::srv::MoveArmToPose>(
      "tool_server/move_arm_to_pose",
      std::bind(&ToolServer::arm_pose_cb, this, _1, _2));
    pick_srv_ = create_service<mm_interfaces::srv::Pick>(
      "tool_server/pick",
      std::bind(&ToolServer::pick_cb, this, _1, _2));
    place_srv_ = create_service<mm_interfaces::srv::Place>(
      "tool_server/place",
      std::bind(&ToolServer::place_cb, this, _1, _2));
    open_srv_ = create_service<mm_interfaces::srv::OpenGripper>(
      "tool_server/open_gripper",
      std::bind(&ToolServer::open_cb, this, _1, _2));
    close_srv_ = create_service<mm_interfaces::srv::CloseGripper>(
      "tool_server/close_gripper",
      std::bind(&ToolServer::close_cb, this, _1, _2));
    state_srv_ = create_service<mm_interfaces::srv::GetSceneState>(
      "tool_server/get_state",
      std::bind(&ToolServer::state_cb, this, _1, _2));

    // ── CSV log ────────────────────────────────────────────────────────────
    std::string log_path =
      "/tmp/tool_log_" + std::to_string(static_cast<long>(now().seconds())) + ".csv";
    csv_.open(log_path);
    csv_ << "timestamp,tool,args,success,duration_ms\n";
    RCLCPP_INFO(get_logger(), "Tool log: %s", log_path.c_str());
  }

  // Call AFTER node is added to executor (shared_from_this() now valid)
  void init_moveit()
  {
    arm_mgi_     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                     shared_from_this(), "arm");
    gripper_mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                     shared_from_this(), "gripper");

    arm_mgi_->setMaxVelocityScalingFactor(0.3);
    arm_mgi_->setMaxAccelerationScalingFactor(0.2);
    gripper_mgi_->setMaxVelocityScalingFactor(0.5);
    RCLCPP_INFO(get_logger(), "MoveGroupInterface ready");
  }

private:
  // ── Logging helper ────────────────────────────────────────────────────────

  void log_call(const std::string& tool, const std::string& args,
                bool success, long ms)
  {
    csv_ << std::fixed << now().seconds()
         << "," << tool << "," << args
         << "," << (success ? "1" : "0")
         << "," << ms << "\n";
    csv_.flush();
  }

  // ── Navigate to named location ─────────────────────────────────────────────

  void nav_cb(
    const mm_interfaces::srv::NavigateTo::Request::SharedPtr req,
    mm_interfaces::srv::NavigateTo::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[navigate_to] location='%s'", req->location.c_str());

    auto it = locations_.find(req->location);
    if (it == locations_.end()) {
      resp->success = false;
      resp->message = "Unknown location: " + req->location;
      log_call("navigate_to", req->location, false, ms_since(t0)); return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      resp->success = false;
      resp->message = "navigate_to_pose action server not available";
      log_call("navigate_to", req->location, false, ms_since(t0)); return;
    }

    const auto& loc = it->second;
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = now();
    goal.pose.pose.position.x = loc.x;
    goal.pose.pose.position.y = loc.y;
    goal.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, loc.yaw);
    goal.pose.pose.orientation = tf2::toMsg(q);

    // Use explicit callbacks + promises so other executor threads can dispatch
    // the goal/result callbacks while this service-callback thread waits.
    auto goal_prom   = std::make_shared<std::promise<GoalHandleNav::SharedPtr>>();
    auto goal_fut    = goal_prom->get_future();
    auto result_prom = std::make_shared<std::promise<rclcpp_action::ResultCode>>();
    auto result_fut  = result_prom->get_future();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback =
      [goal_prom](const GoalHandleNav::SharedPtr& gh) mutable {
        goal_prom->set_value(gh);
      };
    opts.result_callback =
      [result_prom](const GoalHandleNav::WrappedResult& wr) mutable {
        result_prom->set_value(wr.code);
      };

    nav_client_->async_send_goal(goal, opts);

    if (goal_fut.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      resp->success = false;
      resp->message = "Timed out waiting for goal acceptance";
      log_call("navigate_to", req->location, false, ms_since(t0)); return;
    }
    auto gh = goal_fut.get();
    if (!gh) {
      resp->success = false;
      resp->message = "Goal rejected by server";
      log_call("navigate_to", req->location, false, ms_since(t0)); return;
    }

    if (result_fut.wait_for(std::chrono::seconds(300)) != std::future_status::ready) {
      resp->success = false;
      resp->message = "Navigation timed out (300 s)";
      log_call("navigate_to", req->location, false, ms_since(t0)); return;
    }

    bool ok = (result_fut.get() == rclcpp_action::ResultCode::SUCCEEDED);
    resp->success = ok;
    resp->message = ok ? "Arrived at " + req->location : "Navigation failed";
    log_call("navigate_to", req->location, ok, ms_since(t0));
    RCLCPP_INFO(get_logger(), "[navigate_to] %s", resp->message.c_str());
  }

  // ── Detect objects (read latest cached detection) ─────────────────────────

  void detect_cb(
    const mm_interfaces::srv::DetectObjects::Request::SharedPtr,
    mm_interfaces::srv::DetectObjects::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[detect_objects]");

    if (!latest_detections_ || latest_detections_->detections.empty()) {
      resp->success = false;
      resp->message = "No detections available";
      log_call("detect_objects", "", false, ms_since(t0)); return;
    }

    resp->detections = latest_detections_->detections;
    resp->success    = true;
    resp->message    = "Found " + std::to_string(resp->detections.size()) + " objects";
    log_call("detect_objects", "", true, ms_since(t0));
    RCLCPP_INFO(get_logger(), "[detect_objects] %s", resp->message.c_str());
  }

  // ── Move arm to named pose ────────────────────────────────────────────────

  void arm_named_cb(
    const mm_interfaces::srv::MoveArmToNamed::Request::SharedPtr req,
    mm_interfaces::srv::MoveArmToNamed::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[move_arm_to_named] pose='%s'", req->pose_name.c_str());

    // Determine which group to use: gripper poses go to gripper group
    auto& mgi = (req->pose_name == "open" || req->pose_name == "closed")
                ? *gripper_mgi_ : *arm_mgi_;
    mgi.setNamedTarget(req->pose_name);

    bool ok = (mgi.move() == moveit::core::MoveItErrorCode::SUCCESS);
    resp->success = ok;
    resp->message = ok ? "Moved to " + req->pose_name : "Move failed";
    log_call("move_arm_to_named", req->pose_name, ok, ms_since(t0));
    RCLCPP_INFO(get_logger(), "[move_arm_to_named] %s", resp->message.c_str());
  }

  // ── Move arm to Cartesian pose ────────────────────────────────────────────

  void arm_pose_cb(
    const mm_interfaces::srv::MoveArmToPose::Request::SharedPtr req,
    mm_interfaces::srv::MoveArmToPose::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[move_arm_to_pose] x=%.3f y=%.3f z=%.3f",
                req->x, req->y, req->z);

    geometry_msgs::msg::Pose target;
    target.position.x = req->x;
    target.position.y = req->y;
    target.position.z = req->z;
    // Orient gripper downward
    tf2::Quaternion q;
    q.setRPY(0.0, 1.5708, 0.0);
    target.orientation = tf2::toMsg(q);

    arm_mgi_->setPoseReferenceFrame("map");
    arm_mgi_->setPoseTarget(target);

    bool ok = (arm_mgi_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    resp->success = ok;
    resp->message = ok ? "Moved to pose" : "Pose move failed";
    log_call("move_arm_to_pose",
             std::to_string(req->x)+","+std::to_string(req->y)+","+std::to_string(req->z),
             ok, ms_since(t0));
    RCLCPP_INFO(get_logger(), "[move_arm_to_pose] %s", resp->message.c_str());
  }

  // ── Pick ─────────────────────────────────────────────────────────────────

  void pick_cb(
    const mm_interfaces::srv::Pick::Request::SharedPtr req,
    mm_interfaces::srv::Pick::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[pick] label='%s'", req->object_label.c_str());

    const std::string& label = req->object_label;

    if (grasp_pubs_.find(label) == grasp_pubs_.end()) {
      resp->success = false;
      resp->message = "Unknown object: " + label;
      log_call("pick", label, false, ms_since(t0)); return;
    }

    // 1. Open gripper
    gripper_mgi_->setNamedTarget("open");
    gripper_mgi_->move();

    // 2. Reach toward object
    arm_mgi_->setNamedTarget("ready");
    if (arm_mgi_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      resp->success = false;
      resp->message = "Arm failed to reach ready pose";
      log_call("pick", label, false, ms_since(t0)); return;
    }

    // 3. Close gripper (visual grasp)
    gripper_mgi_->setNamedTarget("closed");
    gripper_mgi_->move();

    // 4. Attach via DetachableJoint
    std_msgs::msg::Bool attach_msg;
    attach_msg.data = true;
    grasp_pubs_[label]->publish(attach_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // 5. Lift to home (carry position)
    arm_mgi_->setNamedTarget("home");
    if (arm_mgi_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "[pick] arm failed to retract to home");
    }

    held_object_ = label;
    resp->success = true;
    resp->message = "Picked " + label;
    log_call("pick", label, true, ms_since(t0));
    RCLCPP_INFO(get_logger(), "[pick] %s", resp->message.c_str());
  }

  // ── Place ────────────────────────────────────────────────────────────────

  void place_cb(
    const mm_interfaces::srv::Place::Request::SharedPtr req,
    mm_interfaces::srv::Place::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[place] location='%s'", req->location_name.c_str());

    if (held_object_.empty()) {
      resp->success = false;
      resp->message = "No object currently held";
      log_call("place", req->location_name, false, ms_since(t0)); return;
    }

    // 1. Extend arm to place position
    arm_mgi_->setNamedTarget("ready");
    arm_mgi_->move();

    // 2. Open gripper
    gripper_mgi_->setNamedTarget("open");
    gripper_mgi_->move();

    // 3. Detach via DetachableJoint
    std_msgs::msg::Bool release_msg;
    release_msg.data = true;
    release_pubs_[held_object_]->publish(release_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    std::string placed = held_object_;
    held_object_.clear();

    // 4. Retract arm
    arm_mgi_->setNamedTarget("home");
    arm_mgi_->move();

    resp->success = true;
    resp->message = "Placed " + placed + " at " + req->location_name;
    log_call("place", req->location_name, true, ms_since(t0));
    RCLCPP_INFO(get_logger(), "[place] %s", resp->message.c_str());
  }

  // ── Open / Close gripper ──────────────────────────────────────────────────

  void open_cb(
    const mm_interfaces::srv::OpenGripper::Request::SharedPtr,
    mm_interfaces::srv::OpenGripper::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    gripper_mgi_->setNamedTarget("open");
    bool ok = (gripper_mgi_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    resp->success = ok;
    resp->message = ok ? "Gripper opened" : "Failed to open gripper";
    log_call("open_gripper", "", ok, ms_since(t0));
  }

  void close_cb(
    const mm_interfaces::srv::CloseGripper::Request::SharedPtr,
    mm_interfaces::srv::CloseGripper::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    gripper_mgi_->setNamedTarget("closed");
    bool ok = (gripper_mgi_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    resp->success = ok;
    resp->message = ok ? "Gripper closed" : "Failed to close gripper";
    log_call("close_gripper", "", ok, ms_since(t0));
  }

  // ── Get state (built directly — avoids service-within-service deadlock) ──

  void state_cb(
    const mm_interfaces::srv::GetSceneState::Request::SharedPtr,
    mm_interfaces::srv::GetSceneState::Response::SharedPtr resp)
  {
    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(3);
    ss << "=== SCENE STATE  (t=" << now().seconds() << ") ===\n\n";

    // Robot pose via TF
    bool got_pose = false;
    for (const char* ref : {"map", "odom"}) {
      try {
        auto tf = tf_buffer_->lookupTransform(ref, "base_footprint", tf2::TimePointZero);
        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;
        tf2::Quaternion q(
          tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z, tf.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        ss << "ROBOT POSE [" << ref << " frame]:\n"
           << "  x=" << x << "  y=" << y << "  yaw=" << yaw << " rad\n\n";
        (void)roll; (void)pitch;
        got_pose = true; break;
      } catch (...) {}
    }
    if (!got_pose) ss << "ROBOT POSE: (TF unavailable)\n\n";

    // Arm/gripper joint positions
    ss << "ARM JOINTS:\n";
    if (latest_joints_) {
      for (size_t i = 0; i < latest_joints_->name.size(); ++i) {
        const auto& n = latest_joints_->name[i];
        if (n.rfind("joint", 0) == 0 && !latest_joints_->position.empty()) {
          ss << "  " << n << ": " << latest_joints_->position[i] << " rad\n";
        }
      }
    } else {
      ss << "  (joint_states not received)\n";
    }
    ss << "\n";

    // Detected objects (latest cached)
    ss << "DETECTED OBJECTS:\n";
    if (latest_detections_ && !latest_detections_->detections.empty()) {
      for (const auto& d : latest_detections_->detections) {
        ss << "  " << d.label << " [" << d.color << "]"
           << "  pos=(" << d.x << ", " << d.y << ", " << d.z << ")"
           << "  conf=" << d.confidence << "\n";
      }
    } else {
      ss << "  (none)\n";
    }
    ss << "\n";

    // Known locations
    ss << "KNOWN LOCATIONS:\n";
    for (const auto& [name, loc] : locations_) {
      ss << "  " << name
         << ":  x=" << loc.x << "  y=" << loc.y << "  yaw=" << loc.yaw << "\n";
    }

    resp->state_text = ss.str();
  }

  // ── Utility ──────────────────────────────────────────────────────────────

  static long ms_since(const std::chrono::steady_clock::time_point& t0)
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - t0).count();
  }

  // ── Members ───────────────────────────────────────────────────────────────

  rclcpp::CallbackGroup::SharedPtr nav_cb_group_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<mm_interfaces::msg::DetectionArray>::SharedPtr detect_sub_;

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> grasp_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> release_pubs_;

  rclcpp::Service<mm_interfaces::srv::NavigateTo>::SharedPtr     nav_srv_;
  rclcpp::Service<mm_interfaces::srv::DetectObjects>::SharedPtr  detect_srv_;
  rclcpp::Service<mm_interfaces::srv::MoveArmToNamed>::SharedPtr arm_named_srv_;
  rclcpp::Service<mm_interfaces::srv::MoveArmToPose>::SharedPtr  arm_pose_srv_;
  rclcpp::Service<mm_interfaces::srv::Pick>::SharedPtr           pick_srv_;
  rclcpp::Service<mm_interfaces::srv::Place>::SharedPtr          place_srv_;
  rclcpp::Service<mm_interfaces::srv::OpenGripper>::SharedPtr    open_srv_;
  rclcpp::Service<mm_interfaces::srv::CloseGripper>::SharedPtr   close_srv_;
  rclcpp::Service<mm_interfaces::srv::GetSceneState>::SharedPtr  state_srv_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi_;

  sensor_msgs::msg::JointState::SharedPtr latest_joints_;
  mm_interfaces::msg::DetectionArray::SharedPtr latest_detections_;
  std::map<std::string, Location> locations_;
  std::string held_object_;

  std::ofstream csv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolServer>();

  // MultiThreadedExecutor: service callbacks can block (Nav2 / MoveIt) while
  // other callbacks (action results, subscriptions) continue to be processed.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  // MoveGroupInterface requires shared_from_this() → init AFTER exec.add_node
  // and AFTER the executor has started spinning (spin thread below).
  std::thread exec_thread([&exec]() { exec.spin(); });

  node->init_moveit();

  exec_thread.join();
  rclcpp::shutdown();
  return 0;
}
