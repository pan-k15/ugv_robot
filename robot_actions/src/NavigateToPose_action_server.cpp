#include <chrono>
#include <condition_variable>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/navigate_to_pose.hpp"

using RobotNav = robot_interfaces::action::NavigateToPose;
using Nav2Nav  = nav2_msgs::action::NavigateToPose;
using ServerGH = rclcpp_action::ServerGoalHandle<RobotNav>;
using ClientGH = rclcpp_action::ClientGoalHandle<Nav2Nav>;

class NavigateToPoseActionServer : public rclcpp::Node
{
public:
  NavigateToPoseActionServer() : Node("navigate_to_pose_action_server")
  {
    // Reentrant group so nav2 client callbacks can fire while execute() blocks
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ = rclcpp_action::create_server<RobotNav>(
      this,
      "robot_navigate_to_pose",
      [this](auto uuid, auto goal) { return on_goal(uuid, goal); },
      [this](auto gh)              { return on_cancel(gh); },
      [this](auto gh)              { on_accepted(gh); });

    nav2_client_ = rclcpp_action::create_client<Nav2Nav>(
      this, "navigate_to_pose", cb_group_);

    RCLCPP_INFO(get_logger(), "NavigateToPose action server ready on /robot_navigate_to_pose");
  }

private:
  rclcpp::CallbackGroup::SharedPtr          cb_group_;
  rclcpp_action::Server<RobotNav>::SharedPtr action_server_;
  rclcpp_action::Client<Nav2Nav>::SharedPtr  nav2_client_;

  // ── goal / cancel handlers (fast, called by executor) ──────────────────

  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RobotNav::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Goal received  x=%.2f  y=%.2f  yaw=%.2f",
      goal->x, goal->y, goal->yaw);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(std::shared_ptr<ServerGH>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(std::shared_ptr<ServerGH> server_gh)
  {
    std::thread{[this, server_gh] { execute(server_gh); }}.detach();
  }

  // ── long-running execution (runs in a detached worker thread) ──────────

  void execute(std::shared_ptr<ServerGH> server_gh)
  {
    const auto & goal   = *server_gh->get_goal();
    auto         result = std::make_shared<RobotNav::Result>();

    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
      result->success = false;
      result->message = "navigate_to_pose action server not available";
      server_gh->abort(result);
      return;
    }

    // Build nav2 goal
    Nav2Nav::Goal nav2_goal;
    nav2_goal.pose.header.frame_id    = "map";
    nav2_goal.pose.header.stamp       = get_clock()->now();
    nav2_goal.pose.pose.position.x    = goal.x;
    nav2_goal.pose.pose.position.y    = goal.y;
    const double half = goal.yaw / 2.0;
    nav2_goal.pose.pose.orientation.z = std::sin(half);
    nav2_goal.pose.pose.orientation.w = std::cos(half);

    // Shared state — written by nav2 callbacks, read by this thread
    std::mutex              mtx;
    std::condition_variable cv;
    ClientGH::SharedPtr     nav2_gh;
    bool                    accepted   = false;
    bool                    done       = false;
    float                   distance   = -1.0f;
    int                     recoveries = 0;
    ClientGH::WrappedResult nav2_result{};

    auto opts = rclcpp_action::Client<Nav2Nav>::SendGoalOptions{};

    opts.goal_response_callback =
      [&](const ClientGH::SharedPtr & gh) {
        std::lock_guard lock(mtx);
        nav2_gh  = gh;
        accepted = (gh != nullptr);
        cv.notify_all();
      };

    opts.feedback_callback =
      [&](ClientGH::SharedPtr,
          const std::shared_ptr<const Nav2Nav::Feedback> & fb) {
        std::lock_guard lock(mtx);
        distance   = fb->distance_remaining;
        recoveries = fb->number_of_recoveries;
      };

    opts.result_callback =
      [&](const ClientGH::WrappedResult & wr) {
        std::lock_guard lock(mtx);
        nav2_result = wr;
        done = true;
        cv.notify_all();
      };

    nav2_client_->async_send_goal(nav2_goal, opts);

    // Wait for goal acceptance (5 s)
    {
      std::unique_lock lock(mtx);
      cv.wait_for(lock, std::chrono::seconds(5),
        [&] { return accepted || done; });
    }

    if (!accepted) {
      result->success = false;
      result->message = done ? "Goal rejected by navigate_to_pose"
                              : "Timed out waiting for goal acceptance";
      server_gh->abort(result);
      return;
    }

    // Feedback loop — publish our feedback every 500 ms, check for cancel
    while (true) {
      std::unique_lock lock(mtx);
      const bool finished = cv.wait_for(
        lock, std::chrono::milliseconds(500), [&] { return done; });

      if (finished) break;

      // Build and publish feedback while holding the lock
      auto fb = std::make_shared<RobotNav::Feedback>();
      fb->distance_remaining = distance;
      if (recoveries > 0) {
        fb->current_state = "Recovering (" + std::to_string(recoveries) + ")";
      } else if (distance >= 0.0f && distance < 0.5f) {
        fb->current_state = "Arriving";
      } else {
        fb->current_state = "Following path";
      }
      lock.unlock();

      server_gh->publish_feedback(fb);

      if (server_gh->is_canceling()) {
        nav2_client_->async_cancel_goal(nav2_gh);
        std::unique_lock lk(mtx);
        cv.wait(lk, [&] { return done; });
        result->success = false;
        result->message = "Goal cancelled";
        server_gh->canceled(result);
        return;
      }
    }

    // Final result
    if (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      result->success = true;
      result->message = "Reached (" + std::to_string(goal.x) +
                        ", " + std::to_string(goal.y) + ")";
      server_gh->succeed(result);
    } else {
      result->success = false;
      result->message = "Navigation failed (code " +
                        std::to_string(static_cast<int>(nav2_result.code)) + ")";
      server_gh->abort(result);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
