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
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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
#include "mm_interfaces/srv/drive_distance.hpp"

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
      grasp_pubs_[s]   = create_publisher<std_msgs::msg::Empty>("/grasp/"   + s, 1);
      release_pubs_[s] = create_publisher<std_msgs::msg::Empty>("/release/" + s, 1);
    }

    // /cmd_vel already flows to diff_drive_controller via nav2.launch.py's
    // twist_stamper — drive_distance_cb reuses that same path.
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

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
    drive_srv_ = create_service<mm_interfaces::srv::DriveDistance>(
      "tool_server/drive_distance",
      std::bind(&ToolServer::drive_distance_cb, this, _1, _2));

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

    arm_mgi_->setMaxVelocityScalingFactor(0.8);
    arm_mgi_->setMaxAccelerationScalingFactor(0.4);
    gripper_mgi_->setMaxVelocityScalingFactor(0.5);
    gripper_mgi_->setMaxAccelerationScalingFactor(0.5);
    RCLCPP_INFO(get_logger(), "MoveGroupInterface ready");

    add_pick_table_collision_object();
    release_all_cubes_at_startup();
  }

  // DetachableJoint's default state is ATTACHED at spawn, so without this
  // all 3 cubes would be welded to the gripper from t=0 and fly through the
  // air as soon as the robot moves.
  void release_all_cubes_at_startup()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // let bridge discover pubs
    for (auto& [color, pub] : release_pubs_) {
      pub->publish(std_msgs::msg::Empty());
      RCLCPP_INFO(get_logger(), "Startup detach published for %s", color.c_str());
    }
  }

  // Registers the pick_table (warehouse.sdf) as a MoveIt collision object so
  // OMPL avoids routing the arm through/under it. Without this, position-only
  // IK (orientation tolerance = π) is free to pick ANY joint configuration
  // that reaches the target position — including ones where the elbow swings
  // under the table, since the planner has no idea the table exists.
  void add_pick_table_collision_object()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "map";
    table.id = "pick_table";

    // Tabletop: world centre (2.05, 0, 0.36), size 0.12 x 0.60 x 0.04
    shape_msgs::msg::SolidPrimitive top;
    top.type = shape_msgs::msg::SolidPrimitive::BOX;
    top.dimensions = {0.12, 0.60, 0.04};
    geometry_msgs::msg::Pose top_pose;
    top_pose.position.x = 2.05; top_pose.position.y = 0.0; top_pose.position.z = 0.36;
    top_pose.orientation.w = 1.0;

    // Front legs: world centres (2.10, ±0.25, 0.17), size 0.04 x 0.04 x 0.34
    shape_msgs::msg::SolidPrimitive leg;
    leg.type = shape_msgs::msg::SolidPrimitive::BOX;
    leg.dimensions = {0.04, 0.04, 0.34};
    geometry_msgs::msg::Pose leg_l_pose;
    leg_l_pose.position.x = 2.10; leg_l_pose.position.y = 0.25; leg_l_pose.position.z = 0.17;
    leg_l_pose.orientation.w = 1.0;
    geometry_msgs::msg::Pose leg_r_pose = leg_l_pose;
    leg_r_pose.position.y = -0.25;

    table.primitives = {top, leg, leg};
    table.primitive_poses = {top_pose, leg_l_pose, leg_r_pose};
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    // move_group's own TF buffer can take a few seconds to receive "map"
    // after startup ("Unknown frame: map" + apply_planning_scene timeout
    // observed otherwise) — retry instead of silently leaving the table
    // unregistered, which would reopen the arm-hits-table risk.
    for (int attempt = 1; attempt <= 5; ++attempt) {
      if (psi.applyCollisionObject(table)) {
        RCLCPP_INFO(get_logger(), "pick_table added to MoveIt planning scene");
        return;
      }
      RCLCPP_WARN(get_logger(), "pick_table add attempt %d/5 failed, retrying...", attempt);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    RCLCPP_ERROR(get_logger(), "Failed to add pick_table to planning scene after 5 attempts");
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
  // Real IK to the cube's detected position, descending from above (not a
  // horizontal approach — that needs the gripper facing the cube, which
  // position-only IK doesn't guarantee).

  void pick_cb(
    const mm_interfaces::srv::Pick::Request::SharedPtr req,
    mm_interfaces::srv::Pick::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    const std::string& label = req->object_label;
    RCLCPP_INFO(get_logger(), "[pick] label='%s'", label.c_str());

    if (grasp_pubs_.find(label) == grasp_pubs_.end()) {
      resp->success = false;
      resp->message = "Unknown object: " + label;
      log_call("pick", label, false, ms_since(t0)); return;
    }

    // 1. Locate the object in the latest detection cache
    if (!latest_detections_ || latest_detections_->detections.empty()) {
      resp->success = false;
      resp->message = "No detections available — call detect_objects first";
      log_call("pick", label, false, ms_since(t0)); return;
    }
    double ox_map = 0.0, oy_map = 0.0, oz_map = 0.0;
    bool found = false;
    for (const auto& d : latest_detections_->detections) {
      if (d.label == label) { ox_map = d.x; oy_map = d.y; oz_map = d.z; found = true; break; }
    }
    if (!found) {
      resp->success = false;
      resp->message = label + " not found in current detections";
      log_call("pick", label, false, ms_since(t0)); return;
    }
    RCLCPP_INFO(get_logger(), "[pick] %s at map(%.3f, %.3f, %.3f)", label.c_str(), ox_map, oy_map, oz_map);

    // Convert target to "odom" once: AMCL can drift the "map" frame mid-pick
    // even while the robot sits still, so "map" targets go stale across the
    // several arm moves below. "odom" stays consistent for that duration.
    double ox = 0.0, oy = 0.0, oz = 0.0;  // odom-frame target (reused below)
    {
      geometry_msgs::msg::PoseStamped cube_map;
      cube_map.header.frame_id = "map";
      cube_map.header.stamp = rclcpp::Time(0);
      cube_map.pose.position.x = ox_map;
      cube_map.pose.position.y = oy_map;
      cube_map.pose.position.z = oz_map;
      cube_map.pose.orientation.w = 1.0;
      try {
        auto cube_odom = tf_buffer_->transform(cube_map, "odom", tf2::durationFromSec(0.5));
        ox = cube_odom.pose.position.x;
        oy = cube_odom.pose.position.y;
        oz = cube_odom.pose.position.z;
      } catch (const std::exception& e) {
        resp->success = false;
        resp->message = std::string("TF map->odom failed: ") + e.what();
        log_call("pick", label, false, ms_since(t0)); return;
      }
    }
    RCLCPP_INFO(get_logger(), "[pick] %s at odom(%.3f, %.3f, %.3f)", label.c_str(), ox, oy, oz);

    // 2. Open gripper before approaching
    gripper_mgi_->setNamedTarget("open");
    gripper_mgi_->move();

    // 3. Stage to "ready" — elevates the arm so the OMPL planner has a good
    //    starting configuration and doesn't sweep links through the table
    //    on the way to the pre-grasp pose.
    arm_mgi_->setNamedTarget("ready");
    if (arm_mgi_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      resp->success = false;
      resp->message = "Failed to reach ready staging pose";
      log_call("pick", label, false, ms_since(t0)); return;
    }

    // Position-only IK: any gripper orientation is fine since the approach
    // is vertical and the weld doesn't care which way the gripper faces.
    arm_mgi_->setGoalOrientationTolerance(M_PI);
    arm_mgi_->setGoalPositionTolerance(0.005);
    arm_mgi_->setPoseReferenceFrame("odom");

    // Nominal approach orientation (may be overridden by solver since tolerance=π)
    tf2::Quaternion q_app;
    q_app.setRPY(0.0, M_PI / 2.0, 0.0);

    // 4. Pre-grasp: 10 cm ABOVE the cube. Table top is at z≈0.38 m, so
    //    oz+0.10 (≈0.50 m) is well clear of it.
    geometry_msgs::msg::Pose pre_grasp;
    pre_grasp.position.x = ox;
    pre_grasp.position.y = oy;
    pre_grasp.position.z = oz + 0.10;
    pre_grasp.orientation = tf2::toMsg(q_app);
    arm_mgi_->setPoseTarget(pre_grasp);

    if (arm_mgi_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      resp->success = false;
      resp->message = label + " is out of reach (pre-grasp IK failed)";
      log_call("pick", label, false, ms_since(t0)); return;
    }

    // Lock in the orientation pre-grasp actually achieved (instead of
    // re-opening full M_PI freedom for the descend) so the wrist doesn't
    // jump to a different fold for this small move, and so we can correct
    // for the finger offset below using a known orientation.
    tf2::Quaternion q_locked = q_app;
    try {
      auto pg_tf = tf_buffer_->lookupTransform("odom", "gripper_actuator", tf2::TimePointZero);
      tf2::fromMsg(pg_tf.transform.rotation, q_locked);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "[pick] couldn't read pre-grasp orientation: %s", e.what());
    }

    // gripper_1 (joint6) and gripper_2 (joint7) attach to gripper_actuator at
    // local (∓0.011, 0, 0.02344) — the midpoint between them, where the cube
    // actually needs to be, is offset (0, 0, 0.02344) in gripper_actuator's
    // own frame, not at its origin. Rotate that offset into world frame
    // using the locked orientation so the IK target accounts for it.
    tf2::Vector3 finger_offset_local(0.0, 0.0, 0.02344);
    tf2::Vector3 finger_offset_world = tf2::quatRotate(q_locked, finger_offset_local);

    // Descend so the FINGER MIDPOINT (not gripper_actuator's raw origin)
    // lands just above the cube's TOP FACE (half-height 0.02m) — going to
    // centre means the rigid gripper occupies the same space as the solid
    // cube, shoving it aside before arriving.
    constexpr double GRASP_STANDOFF = 0.03;
    geometry_msgs::msg::Pose grasp_pose = pre_grasp;
    grasp_pose.position.x = ox - finger_offset_world.x();
    grasp_pose.position.y = oy - finger_offset_world.y();
    grasp_pose.position.z = oz + GRASP_STANDOFF - finger_offset_world.z();
    grasp_pose.orientation = tf2::toMsg(q_locked);
    arm_mgi_->setGoalOrientationTolerance(0.2);  // hold the locked orientation closely
    arm_mgi_->setPoseTarget(grasp_pose);

    if (arm_mgi_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      arm_mgi_->setGoalOrientationTolerance(M_PI);
      arm_mgi_->setNamedTarget("home");
      arm_mgi_->move();
      resp->success = false;
      resp->message = label + " grasp position unreachable";
      log_call("pick", label, false, ms_since(t0)); return;
    }
    arm_mgi_->setGoalOrientationTolerance(M_PI);

    // Proximity check: compare the actual FINGER MIDPOINT (gripper_actuator
    // position + the same rotated offset) to the cube, not the raw actuator
    // origin — GRASP_STANDOFF (3cm) + detection noise margin (~2cm).
    constexpr double PROXIMITY_THRESHOLD = 0.05;
    try {
      auto tf = tf_buffer_->lookupTransform("odom", "gripper_actuator",
                                            tf2::TimePointZero);
      tf2::Quaternion q_actual;
      tf2::fromMsg(tf.transform.rotation, q_actual);
      tf2::Vector3 offset_world = tf2::quatRotate(q_actual, finger_offset_local);
      double dx = tf.transform.translation.x + offset_world.x() - ox;
      double dy = tf.transform.translation.y + offset_world.y() - oy;
      double dz = tf.transform.translation.z + offset_world.z() - oz;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      RCLCPP_INFO(get_logger(), "[pick] finger-midpoint→cube dist: %.3f m", dist);
      if (dist > PROXIMITY_THRESHOLD) {
        arm_mgi_->setNamedTarget("home");
        arm_mgi_->move();
        resp->success = false;
        resp->message = label + ": gripper " + std::to_string(int(dist * 100))
                        + " cm from cube (need ≤5 cm)";
        log_call("pick", label, false, ms_since(t0)); return;
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "[pick] TF lookup failed: %s — proceeding", e.what());
    }

    // 7. Close gripper and weld the cube via DetachableJoint
    gripper_mgi_->setNamedTarget("closed");
    gripper_mgi_->move();

    grasp_pubs_[label]->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 8. Lift 10 cm (straight up from grasp position; relative to
    //    grasp_pose, which already has the finger-offset correction)
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z = grasp_pose.position.z + 0.10;
    arm_mgi_->setGoalOrientationTolerance(0.2);  // keep the locked orientation
    arm_mgi_->setPoseTarget(lift_pose);
    arm_mgi_->move();
    arm_mgi_->setGoalOrientationTolerance(M_PI);

    // 9. Carry at home (tucked position for navigation)
    arm_mgi_->setNamedTarget("home");
    arm_mgi_->move();

    held_object_ = label;
    resp->success = true;
    resp->message = "Picked " + label;
    log_call("pick", label, true, ms_since(t0));
    RCLCPP_INFO(get_logger(), "[pick] %s", resp->message.c_str());
  }

  // ── Place ────────────────────────────────────────────────────────────────
  // Tries IK to above the location coordinates; falls back to "ready" if IK
  // fails (e.g. location out of arm reach from current robot parking spot).

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

    // 1. Try IK to location coordinates (same height the cube was picked at)
    const double PLACE_Z = 0.43;  // ~cube height above floor, near table level
    bool placed_via_ik = false;

    auto it = locations_.find(req->location_name);
    if (it != locations_.end()) {
      arm_mgi_->setGoalOrientationTolerance(M_PI);
      tf2::Quaternion q_pl;
      q_pl.setRPY(0.0, M_PI / 2.0, 0.0);
      arm_mgi_->setPoseReferenceFrame("map");

      geometry_msgs::msg::Pose pre_place;
      pre_place.position.x = it->second.x;
      pre_place.position.y = it->second.y;
      pre_place.position.z = PLACE_Z + 0.12;
      pre_place.orientation = tf2::toMsg(q_pl);
      arm_mgi_->setPoseTarget(pre_place);

      if (arm_mgi_->move() == moveit::core::MoveItErrorCode::SUCCESS) {
        // Descend to place height (best-effort: cube is attached, so safe)
        geometry_msgs::msg::Pose place_pose = pre_place;
        place_pose.position.z = PLACE_Z;
        arm_mgi_->setPoseTarget(place_pose);
        arm_mgi_->move();
        placed_via_ik = true;
      }
    }

    if (!placed_via_ik) {
      // Fallback: extend arm to "ready" and drop from there
      arm_mgi_->setNamedTarget("ready");
      arm_mgi_->move();
    }

    // 2. Open gripper and release DetachableJoint
    gripper_mgi_->setNamedTarget("open");
    gripper_mgi_->move();

    release_pubs_[held_object_]->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    std::string placed = held_object_;
    held_object_.clear();

    // 3. Retract arm to home
    arm_mgi_->setNamedTarget("home");
    arm_mgi_->move();

    resp->success = true;
    resp->message = "Placed " + placed + " at " + req->location_name
                    + (placed_via_ik ? "" : " (fallback drop)");
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

  // ── Drive a relative distance ───────────────────────────────────────────
  // Blind, odometry-only straight-line nudge — NOT obstacle-aware (no
  // costmap), unlike navigate_to. Capped to a short distance so a bad call
  // can't drive the robot far into something. Use navigate_to for anything
  // beyond a small local adjustment.
  void drive_distance_cb(
    const mm_interfaces::srv::DriveDistance::Request::SharedPtr req,
    mm_interfaces::srv::DriveDistance::Response::SharedPtr resp)
  {
    auto t0 = std::chrono::steady_clock::now();
    double distance_m = req->distance_m;
    RCLCPP_INFO(get_logger(), "[drive_distance] distance_m=%.3f", distance_m);

    constexpr double MAX_DISTANCE = 2.0;
    if (std::abs(distance_m) > MAX_DISTANCE) {
      resp->success = false;
      resp->message = "distance_m magnitude exceeds " + std::to_string(MAX_DISTANCE)
                      + " m cap; use navigate_to for longer moves";
      log_call("drive_distance", std::to_string(distance_m), false, ms_since(t0)); return;
    }

    geometry_msgs::msg::TransformStamped start_tf;
    try {
      start_tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (const std::exception& e) {
      resp->success = false;
      resp->message = std::string("TF lookup failed: ") + e.what();
      log_call("drive_distance", std::to_string(distance_m), false, ms_since(t0)); return;
    }
    double sx = start_tf.transform.translation.x;
    double sy = start_tf.transform.translation.y;

    constexpr double SPEED = 0.15;  // m/s, conservative
    double target_abs = std::abs(distance_m);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = (distance_m >= 0.0 ? SPEED : -SPEED);

    double timeout_s = target_abs / SPEED + 5.0;  // generous margin
    rclcpp::Rate rate(20.0);
    double traveled = 0.0;
    while (rclcpp::ok()) {
      cmd_vel_pub_->publish(cmd);
      try {
        auto cur_tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        double dx = cur_tf.transform.translation.x - sx;
        double dy = cur_tf.transform.translation.y - sy;
        traveled = std::sqrt(dx*dx + dy*dy);
      } catch (const std::exception&) { /* keep last known traveled, keep trying */ }

      if (traveled >= target_abs) break;
      if (ms_since(t0) > timeout_s * 1000.0) break;
      rate.sleep();
    }
    cmd.linear.x = 0.0;
    cmd_vel_pub_->publish(cmd);

    bool ok = traveled >= target_abs * 0.9;  // 10% tolerance
    resp->success = ok;
    std::ostringstream msg;
    msg << "Drove " << traveled << " m (" << distance_m << " requested)";
    resp->message = msg.str();
    log_call("drive_distance", std::to_string(distance_m), ok, ms_since(t0));
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

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> grasp_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> release_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Service<mm_interfaces::srv::NavigateTo>::SharedPtr     nav_srv_;
  rclcpp::Service<mm_interfaces::srv::DetectObjects>::SharedPtr  detect_srv_;
  rclcpp::Service<mm_interfaces::srv::MoveArmToNamed>::SharedPtr arm_named_srv_;
  rclcpp::Service<mm_interfaces::srv::MoveArmToPose>::SharedPtr  arm_pose_srv_;
  rclcpp::Service<mm_interfaces::srv::Pick>::SharedPtr           pick_srv_;
  rclcpp::Service<mm_interfaces::srv::Place>::SharedPtr          place_srv_;
  rclcpp::Service<mm_interfaces::srv::OpenGripper>::SharedPtr    open_srv_;
  rclcpp::Service<mm_interfaces::srv::CloseGripper>::SharedPtr   close_srv_;
  rclcpp::Service<mm_interfaces::srv::GetSceneState>::SharedPtr  state_srv_;
  rclcpp::Service<mm_interfaces::srv::DriveDistance>::SharedPtr  drive_srv_;

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
