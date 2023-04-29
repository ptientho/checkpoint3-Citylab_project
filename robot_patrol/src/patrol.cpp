#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "unistd.h"
#include <functional>
#include <memory>

using LaserScan = sensor_msgs::msg::LaserScan;
using Velocity = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {

private:
  rclcpp::Subscription<LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<Velocity>::SharedPtr pub_vel_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr scan_group_;
  rclcpp::CallbackGroup::SharedPtr pub_group_;

  float middle;
  float left;
  float right;
  Velocity vel_msg;
  void laser_callback(const LaserScan::SharedPtr msg) {

    int num = msg->ranges.size();
    // RCLCPP_INFO(this->get_logger(),"Scan size: %i",num);

    middle = msg->ranges[num / 2];
    left = msg->ranges[(num / 2) + 180 - 1];
    right = msg->ranges[num / 4];
    RCLCPP_INFO(this->get_logger(),
                "Received scan values: middle[%f], left[%f], right[%f]", middle,
                left, right);
  }

  void timer_callback() {

    // check frint
    if (middle > 0.5) {

      vel_msg.linear.x = 0.1;
      vel_msg.angular.z = 0.0;
      // move forward
      pub_vel_->publish(vel_msg);

    } else if (middle <= 0.5) {
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = -0.6;
      // turn right
      pub_vel_->publish(vel_msg);
      // sleep(1);
    }
  }

public:
  Patrol() : Node("patrol_node") {

    scan_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions option1;
    option1.callback_group = scan_group_;

    scan_ = this->create_subscription<LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
        option1);

    pub_vel_ = this->create_publisher<Velocity>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
        300ms, std::bind(&Patrol::timer_callback, this), pub_group_);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  std::shared_ptr<Patrol> node = std::make_shared<Patrol>();

  rclcpp::executors::MultiThreadedExecutor my_exe;
  my_exe.add_node(node);
  my_exe.spin();

  rclcpp::shutdown();
  return 0;
}
