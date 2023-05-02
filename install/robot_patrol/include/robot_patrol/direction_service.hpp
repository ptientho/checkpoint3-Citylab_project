#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_interface/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using GetDirection = robot_patrol_interface::srv::GetDirection;
using Velocity = geometry_msgs::msg::Twist;

class DirectionService : public rclcpp::Node {

private:
  rclcpp::Service<GetDirection>::SharedPtr direction_srv_;
  rclcpp::CallbackGroup::SharedPtr srv_group_;

  float middleMin;
  float leftMin;
  float rightMin;

  void patrol_callback(const std::shared_ptr<GetDirection::Request> req,
                       const std::shared_ptr<GetDirection::Response> res);

public:
  DirectionService(std::string &service_name);
  ~DirectionService();
};