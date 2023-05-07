#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol_interface/action/detail/go_to_point__struct.hpp"
#include "robot_patrol_interface/action/go_to_point.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

using GoToPoint = robot_patrol_interface::action::GoToPoint;
using Pose = nav_msgs::msg::Odometry;
using Speed = geometry_msgs::msg::Twist;
using GoalHandleAction = rclcpp_action::ServerGoalHandle<GoToPoint>;

class GoToPointAction : public rclcpp::Node {
public:
  explicit GoToPointAction(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_point_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPoint>(
        this, "/go_to_point",
        std::bind(&GoToPointAction::handle_goal, this, _1, _2),
        std::bind(&GoToPointAction::handle_cancel, this, _1),
        std::bind(&GoToPointAction::handle_accepted, this, _1));
    // define subscriber
    odom_sub_ = this->create_subscription<Pose>(
        "odom", 10, std::bind(&GoToPointAction::odom_callback, this, _1));
    vel_pub_ = this->create_publisher<Speed>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Action Server Initialized..");
  }

private:
  rclcpp_action::Server<GoToPoint>::SharedPtr action_server_;
  rclcpp::Subscription<Pose>::SharedPtr odom_sub_;
  rclcpp::Publisher<Speed>::SharedPtr vel_pub_;
  Speed vel;
  geometry_msgs::msg::Point32 current_pos;

  void odom_callback(const Pose::SharedPtr odom_msg) { // odom message

    // get the orientation as a quaternion
    tf2::Quaternion quat;
    tf2::fromMsg(odom_msg->pose.pose.orientation, quat);
    // convert quaternion to roll, pitch, yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    float yaw_degree = yaw * 180.0 / M_PI;

    this->current_pos.x = odom_msg->pose.pose.position.x;
    this->current_pos.y = odom_msg->pose.pose.position.y;
    this->current_pos.z = yaw_degree;

    RCLCPP_DEBUG(this->get_logger(), "x: %f, y: %f, theta: %f",
                 this->current_pos.x, this->current_pos.y, this->current_pos.z);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with order x:%f, y:%f, theta:%f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAction> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPointAction::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // define variable
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPoint::Feedback>();
    auto &message = feedback->current_pos;

    auto result = std::make_shared<GoToPoint::Result>();
    rclcpp::Rate loop_rate(10);

    // get goal pose
    auto goal_x = goal->goal_pos.x;
    auto goal_y = goal->goal_pos.y;
    auto goal_theta = goal->goal_pos.z;
    // error pose threshold
    geometry_msgs::msg::Point32 error;
    error.x = 0.01;
    error.y = 0.1;
    error.z = 1.5;

    // current position error
    float current_err_x_val = std::abs(goal_x - this->current_pos.x);
    float current_err_y_val = std::abs(goal_y - this->current_pos.y);
    float goal_orientation =
        std::atan2(goal_y - this->current_pos.y, goal_x - this->current_pos.x) *
        180.0 / M_PI;

    float current_err_orientation =
        std::abs(goal_orientation - this->current_pos.z);
    float current_err_z_val = std::abs(goal_theta - this->current_pos.z);
    // RCLCPP_INFO(this->get_logger(), "Goal Orientation: %f vs Current Angle:
    // %f",
    //             goal_orientation, this->current_pos.z);

    // velocity limits
    float max_linear_vel = 0.3;
    float max_angular_vel = 0.3;
    float linear_vel = max_linear_vel * current_err_x_val;
    float angular_vel = max_angular_vel * current_err_z_val;

    bool is_oriented = false;
    bool on_target = false;

    // 1.orient robot to the goal
    while (rclcpp::ok()) {

      if (goal_handle->is_canceling()) {
        result->status = false;
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub_->publish(vel);
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if (current_err_orientation <= 1.0 && is_oriented == false) {

        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub_->publish(vel);
        is_oriented = true;

      } else if (current_err_orientation > 1.0 && is_oriented == false) {

        // update error
        current_err_orientation =
            std::abs(goal_orientation - this->current_pos.z);
        // update angular velocity
        angular_vel = max_angular_vel * current_err_orientation;
        angular_vel =
            std::max(std::min(angular_vel, max_angular_vel), -max_angular_vel);
        if (goal_orientation < 0.0)
          vel.angular.z = -angular_vel;
        else
          vel.angular.z = angular_vel;

        vel_pub_->publish(vel);
        // RCLCPP_INFO(this->get_logger(), "Current Error Orientation: %f",
        //             current_err_orientation);
      }

      if (is_oriented == true) {

        // move forward
        if (current_err_x_val <= error.x && current_err_y_val <= error.y &&
            on_target == false) {

          vel.linear.x = 0.0;
          vel.angular.z = 0.0;
          vel_pub_->publish(vel);
          on_target = true;

          // RCLCPP_INFO(this->get_logger(), "Current Error x:%f, y:%f, z:%f",
          //             current_err_x_val, current_err_y_val,
          //             current_err_z_val);

        } else if ((current_err_x_val > error.x ||
                    current_err_y_val > error.y) &&
                   on_target == false) {

          current_err_x_val = std::abs(goal_x - this->current_pos.x);
          current_err_y_val = std::abs(goal_y - this->current_pos.y);

          linear_vel = max_linear_vel * current_err_x_val;
          linear_vel =
              std::max(std::min(linear_vel, max_linear_vel), -max_linear_vel);

          vel.linear.x = linear_vel;
          vel_pub_->publish(vel);

          // RCLCPP_INFO(this->get_logger(), "Current Error x:%f, y:%f",
          //             current_err_x_val, current_err_y_val);
        }

        if (on_target == true) {

          if (current_err_z_val <= error.z) {
            result->status = true;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            vel_pub_->publish(vel);
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            return;
          } else {

            current_err_z_val = std::abs(goal_theta - this->current_pos.z);

            angular_vel = max_angular_vel * current_err_z_val;
            angular_vel = std::max(std::min(angular_vel, max_angular_vel),
                                   -max_angular_vel);
            if (goal_theta < 0.0)
              vel.angular.z = -angular_vel;
            else
              vel.angular.z = angular_vel;

            vel_pub_->publish(vel);

            // RCLCPP_INFO(this->get_logger(), "Current Error z:%f",
            //             current_err_z_val);
          }
        }
      }

      // update feedback
      message = current_pos;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPointAction>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}