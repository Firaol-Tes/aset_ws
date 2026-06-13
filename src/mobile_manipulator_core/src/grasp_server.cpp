#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <unordered_map>

/**
 * grasp_server — ROS 2 service wrapper for Gazebo DetachableJoint grasping.
 *
 * Services:
 *   /grasp_attach  (std_srvs/SetBool)  — request.data=true: attach current object
 *                                         request.data=false: release it
 *
 * Parameter:
 *   current_object  (string)  — name of object to grasp: "red_cube", "green_cube",
 *                               "blue_cube".  Set before calling /grasp_attach.
 *
 * The node publishes std_msgs/Bool to /grasp/<object> or /release/<object>, which
 * are bridged by ros_gz_bridge to the Gazebo DetachableJoint gz topics.
 *
 * Usage example (command line):
 *   ros2 param set /grasp_server current_object red_cube
 *   ros2 service call /grasp_attach std_srvs/srv/SetBool "{data: true}"
 *   ros2 service call /grasp_attach std_srvs/srv/SetBool "{data: false}"
 */

using SetBool = std_srvs::srv::SetBool;
using BoolMsg = std_msgs::msg::Bool;

class GraspServer : public rclcpp::Node
{
public:
  GraspServer()
  : Node("grasp_server")
  {
    declare_parameter<std::string>("current_object", "red_cube");

    // One publisher per graspable object (for attach and release topics)
    for (const auto & obj : {"red_cube", "green_cube", "blue_cube"}) {
      grasp_pubs_[obj]   = create_publisher<BoolMsg>("/grasp/"   + std::string(obj), 1);
      release_pubs_[obj] = create_publisher<BoolMsg>("/release/" + std::string(obj), 1);
    }

    service_ = create_service<SetBool>(
      "/grasp_attach",
      [this](const SetBool::Request::SharedPtr req,
             SetBool::Response::SharedPtr res) {
        std::string obj = get_parameter("current_object").as_string();
        if (grasp_pubs_.find(obj) == grasp_pubs_.end()) {
          res->success = false;
          res->message = "Unknown object: " + obj;
          RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
          return;
        }
        BoolMsg msg;
        msg.data = true;
        if (req->data) {
          RCLCPP_INFO(get_logger(), "Attaching %s", obj.c_str());
          grasp_pubs_[obj]->publish(msg);
        } else {
          RCLCPP_INFO(get_logger(), "Releasing %s", obj.c_str());
          release_pubs_[obj]->publish(msg);
        }
        res->success = true;
        res->message = req->data ? ("attached " + obj) : ("released " + obj);
      });

    RCLCPP_INFO(get_logger(), "grasp_server ready. "
      "Set 'current_object' param then call /grasp_attach.");
  }

private:
  rclcpp::Service<SetBool>::SharedPtr service_;
  std::unordered_map<std::string, rclcpp::Publisher<BoolMsg>::SharedPtr> grasp_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<BoolMsg>::SharedPtr> release_pubs_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspServer>());
  rclcpp::shutdown();
  return 0;
}
