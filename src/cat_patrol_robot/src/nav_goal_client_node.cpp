// nav_goal_client_node.cpp
// A standalone NavigateToPose action client. Sends one hard-coded goal,
// prints feedback (distance remaining) and the final result.
//
// Learning goals (plan.md Phase 3):
//   - rclcpp_action::Client<NavigateToPose>
//   - goal handle (ClientGoalHandle::SharedPtr)
//   - goal-response / feedback / result callbacks
//   - waiting on a result without blocking the executor
//   - goal cancellation (async_cancel_goal) — needed in Phase 6

#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

class NavGoalClient : public rclcpp::Node
{
public:
  NavGoalClient() : rclcpp::Node("nav_goal_client")
  {
    // Hard-coded goal pose in the map frame. Edit x/y/yaw to a real free
    // cell in living_room_v1 before running.
    this->declare_parameter("goal_x", 1.0);
    this->declare_parameter("goal_y", 0.0);
    this->declare_parameter("goal_yaw", 0.0);  // radians

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Defer the send until the executor is spinning, so the callbacks fire.
    timer_ = this->create_wall_timer(500ms, std::bind(&NavGoalClient::send_goal, this));
  }

private:
  void send_goal()
  {
    timer_->cancel();  // one-shot

    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
      rclcpp::shutdown();
      return;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = this->get_parameter("goal_x").as_double();
    goal.pose.pose.position.y = this->get_parameter("goal_y").as_double();
    const double yaw = this->get_parameter("goal_yaw").as_double();
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO(get_logger(), "Sending goal: x=%.2f y=%.2f yaw=%.2f",
                goal.pose.pose.position.x, goal.pose.pose.position.y, yaw);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr & h) {
        if (!h) {
          RCLCPP_ERROR(get_logger(), "Goal REJECTED by server");
          rclcpp::shutdown();
        } else {
          RCLCPP_INFO(get_logger(), "Goal ACCEPTED, navigating...");
        }
      };
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO(get_logger(), "  distance remaining: %.2f m", fb->distance_remaining);
      };
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "goal succeeded"); break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "goal aborted"); break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "goal canceled"); break;
          default:
            RCLCPP_ERROR(get_logger(), "unknown result code"); break;
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal, opts);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavGoalClient>());
  rclcpp::shutdown();
  return 0;
}
