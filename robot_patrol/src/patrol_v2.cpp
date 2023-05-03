#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol_interface/srv/detail/get_direction__struct.hpp"
#include "robot_patrol_interface/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>

using GetDirection = robot_patrol_interface::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;
using Velocity = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;

class CallPatrolService : public rclcpp::Node {

private:
  rclcpp::Client<GetDirection>::SharedPtr client;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::CallbackGroup::SharedPtr laser_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  std::shared_ptr<LaserScan> temp_msg = nullptr;
  rclcpp::Publisher<Velocity>::SharedPtr vel_pub;
  Velocity vel_msg;

  void laser_callback(const LaserScan::SharedPtr msg) {

    // request->laser_data = *msg;
    temp_msg = msg;
    // RCLCPP_INFO(this->get_logger(), "laser range at 0: %f",
    //             temp_msg->ranges[0]);
  }
  // use
  void send_request() {

    while (!client->wait_for_service(1s)) {

      try {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Interrupted while waiting for the service. Exiting.");
          throw 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
      } catch (int e) {
        break;
        rclcpp::shutdown();
      }
    }
    auto request = std::make_shared<GetDirection::Request>();

    request->laser_data = *temp_msg;

    auto result_future = client->async_send_request(request);

    // Wait for the result.
    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
    //                                       result_future) ==
    //    rclcpp::FutureReturnCode::SUCCESS) {
    std::future_status status = result_future.wait_for(2s);
    if (status == std::future_status::ready) {
      auto result = result_future.get();

      // move robot!
      try {
        if (result->direction == "forward") {
          // move forward
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                      "The robot response: forward");
          vel_msg.linear.x = 0.1;
          vel_msg.angular.z = 0.0;
          vel_pub->publish(vel_msg);

        } else if (result->direction == "left") {

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot response: left");
          vel_msg.linear.x = 0.1;
          vel_msg.angular.z = 0.4;
          vel_pub->publish(vel_msg);

        } else if (result->direction == "right") {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                      "The robot response: right");
          vel_msg.linear.x = 0.1;
          vel_msg.angular.z = -0.4;
          vel_pub->publish(vel_msg);

        } else {
          throw 1;
        }
      } catch (int e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "The command doesn't exist!");
      }

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service /moving");
    }
  }

public:
  CallPatrolService() : Node("test_service_client") {

    laser_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    timer_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    client = this->create_client<GetDirection>("direction_service");

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = laser_group_;
    laser_sub = this->create_subscription<LaserScan>(
        "scan", 10,
        std::bind(&CallPatrolService::laser_callback, this,
                  std::placeholders::_1),
        options1);

    vel_pub = this->create_publisher<Velocity>("cmd_vel", 10);

    timer = this->create_wall_timer(
        500ms, std::bind(&CallPatrolService::send_request, this), timer_group_);
  }

  ~CallPatrolService() {}
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  std::shared_ptr<CallPatrolService> nh = std::make_shared<CallPatrolService>();

  rclcpp::executors::MultiThreadedExecutor my_exe;
  my_exe.add_node(nh);
  my_exe.spin();

  rclcpp::shutdown();

  return 0;
}