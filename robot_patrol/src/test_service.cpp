#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
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
using namespace std::chrono_literals;

class TestClient : public rclcpp::Node {

private:
  rclcpp::Client<GetDirection>::SharedPtr client;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  std::shared_ptr<LaserScan> temp_msg = nullptr;

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
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot response: %s",
                  result->direction.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service /moving");
    }
  }

public:
  TestClient() : Node("test_service_client") {

    client_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    timer_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    client = this->create_client<GetDirection>("direction_service");

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = client_group_;
    laser_sub = this->create_subscription<LaserScan>(
        "scan", 10,
        std::bind(&TestClient::laser_callback, this, std::placeholders::_1),
        options1);

    timer = this->create_wall_timer(
        500ms, std::bind(&TestClient::send_request, this), timer_group_);
  }

  ~TestClient() {}
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  std::shared_ptr<TestClient> nh = std::make_shared<TestClient>();

  rclcpp::executors::MultiThreadedExecutor my_exe;
  my_exe.add_node(nh);
  my_exe.spin();

  rclcpp::shutdown();

  return 0;
}