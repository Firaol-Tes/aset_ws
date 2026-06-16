/**
 * state_manager_node.cpp  —  Aggregates robot state into human-readable text.
 *
 * Subscribes to:
 *   /diff_drive_controller/odom   nav_msgs/Odometry
 *   /joint_states                 sensor_msgs/JointState
 *   /detected_objects             mm_interfaces/DetectionArray
 *
 * TF lookup: map → base_footprint  (falls back to odom if map not available)
 *
 * Service:
 *   /get_scene_state              mm_interfaces/GetSceneState
 *
 * Parameters:
 *   locations_file   path to locations.yaml
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "mm_interfaces/msg/detection_array.hpp"
#include "mm_interfaces/srv/get_scene_state.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>

struct Location { double x, y, yaw; };

class StateManager : public rclcpp::Node
{
public:
  StateManager() : Node("state_manager_node")
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) { odom_ = m; });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr m) { joints_ = m; });

    detect_sub_ = create_subscription<mm_interfaces::msg::DetectionArray>(
      "/detected_objects", 10,
      [this](mm_interfaces::msg::DetectionArray::SharedPtr m) { detections_ = m; });

    state_srv_ = create_service<mm_interfaces::srv::GetSceneState>(
      "/get_scene_state",
      [this](const mm_interfaces::srv::GetSceneState::Request::SharedPtr,
             mm_interfaces::srv::GetSceneState::Response::SharedPtr resp) {
        resp->state_text = build_state();
      });

    declare_parameter("locations_file", "");
    std::string loc_file = get_parameter("locations_file").as_string();
    if (!loc_file.empty()) load_locations(loc_file);

    RCLCPP_INFO(get_logger(), "State manager ready (locations: %zu)", locations_.size());
  }

private:
  std::string build_state()
  {
    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(3);

    ss << "=== SCENE STATE  (t=" << now().seconds() << ") ===\n\n";

    // ── Robot pose via TF ─────────────────────────────────────────────────────
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
        got_pose = true;
        break;
      } catch (...) {}
    }
    if (!got_pose) ss << "ROBOT POSE: (TF unavailable)\n\n";

    // ── Arm joint positions ───────────────────────────────────────────────────
    ss << "ARM JOINTS:\n";
    if (joints_) {
      for (size_t i = 0; i < joints_->name.size(); ++i) {
        const auto& n = joints_->name[i];
        // Only show arm/gripper joints (names that start with "joint")
        if (n.rfind("joint", 0) == 0 && !joints_->position.empty()) {
          ss << "  " << n << ": " << joints_->position[i] << " rad\n";
        }
      }
    } else {
      ss << "  (joint_states not received)\n";
    }
    ss << "\n";

    // ── Detected objects ──────────────────────────────────────────────────────
    ss << "DETECTED OBJECTS:\n";
    if (detections_ && !detections_->detections.empty()) {
      for (const auto& d : detections_->detections) {
        ss << "  " << d.label << " [" << d.color << "]"
           << "  pos=(" << d.x << ", " << d.y << ", " << d.z << ")"
           << "  conf=" << d.confidence << "\n";
      }
    } else {
      ss << "  (none)\n";
    }
    ss << "\n";

    // ── Known locations ───────────────────────────────────────────────────────
    ss << "KNOWN LOCATIONS:\n";
    for (const auto& [name, loc] : locations_) {
      ss << "  " << name
         << ":  x=" << loc.x << "  y=" << loc.y << "  yaw=" << loc.yaw << "\n";
    }

    return ss.str();
  }

  void load_locations(const std::string& path)
  {
    try {
      YAML::Node root = YAML::LoadFile(path);
      for (auto it = root.begin(); it != root.end(); ++it) {
        if (it->first.IsScalar() && it->second.IsMap()) {
          std::string name = it->first.as<std::string>();
          Location loc{};
          if (it->second["x"])   loc.x   = it->second["x"].as<double>();
          if (it->second["y"])   loc.y   = it->second["y"].as<double>();
          if (it->second["yaw"]) loc.yaw = it->second["yaw"].as<double>();
          locations_[name] = loc;
        }
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu locations from %s",
                  locations_.size(), path.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load locations: %s", e.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<mm_interfaces::msg::DetectionArray>::SharedPtr detect_sub_;
  rclcpp::Service<mm_interfaces::srv::GetSceneState>::SharedPtr state_srv_;

  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::JointState::SharedPtr joints_;
  mm_interfaces::msg::DetectionArray::SharedPtr detections_;

  std::map<std::string, Location> locations_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateManager>());
  rclcpp::shutdown();
  return 0;
}
