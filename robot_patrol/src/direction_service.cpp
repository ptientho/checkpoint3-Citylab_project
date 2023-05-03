#include "robot_patrol/direction_service.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "robot_patrol_interface/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "unistd.h"
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using GetDirection = robot_patrol_interface::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using Velocity = geometry_msgs::msg::Twist;

void DirectionService::patrol_callback(
    const std::shared_ptr<GetDirection::Request> req,
    const std::shared_ptr<GetDirection::Response> res) {

  // divide ranges into 3 section
  RCLCPP_DEBUG(this->get_logger(), "Service started...");
  std::vector<float> right_sec(req->laser_data.ranges.begin() + 179,
                               req->laser_data.ranges.begin() + 299);
  std::vector<float> middle_sec(req->laser_data.ranges.begin() + 300,
                                req->laser_data.ranges.begin() + 419);
  std::vector<float> left_sec(req->laser_data.ranges.begin() + 420,
                              req->laser_data.ranges.begin() + 539);

  // find the minimum value in each section
  rightMin = *std::min_element(right_sec.begin(), right_sec.end());
  middleMin = *std::min_element(middle_sec.begin(), middle_sec.end());
  leftMin = *std::min_element(left_sec.begin(), left_sec.end());

  float max_value = std::max(middleMin, std::max(leftMin, rightMin));

  RCLCPP_INFO(this->get_logger(), "max value: %f", max_value);
  if (middleMin == max_value) {
    // forward
    res->direction = "forward";

  } else if (leftMin == max_value) {

    // left
    res->direction = "left";

  } else if (rightMin == max_value) {

    // right
    res->direction = "right";
  }
  RCLCPP_INFO(this->get_logger(), "Move direction %s", res->direction.c_str());
}

DirectionService::DirectionService(std::string &service_name)
    : Node("server_direction_node") {

  srv_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  direction_srv_ = this->create_service<GetDirection>(
      service_name, std::bind(&DirectionService::patrol_callback, this, _1, _2),
      rmw_qos_profile_default, srv_group_);

  RCLCPP_INFO(this->get_logger(), "Initialized Server...");
}

DirectionService::~DirectionService() {}




int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  std::string srv_name = "/direction_service";
  std::shared_ptr<DirectionService> node =
      std::make_shared<DirectionService>(srv_name);

  rclcpp::executors::MultiThreadedExecutor my_exe;
  my_exe.add_node(node);
  my_exe.spin();

  rclcpp::shutdown();
  return 0;
}


