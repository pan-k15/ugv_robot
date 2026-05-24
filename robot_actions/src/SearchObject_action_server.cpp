#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/search_object.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection2_d_array.hpp"

using SearchObject = robot_interfaces::action::SearchObject;
using ServerGH     = rclcpp_action::ServerGoalHandle<SearchObject>;
using namespace std::chrono_literals;

static constexpr float SCAN_SPEED   = 0.3f;        // rad/s — full rotation scan speed
static constexpr float SCAN_ANGLE   = 2.0f * M_PI; // full 360°
static constexpr int   DEPTH_RADIUS = 5;            // px — search radius for valid depth

class SearchObjectActionServer : public rclcpp::Node
{
public:
  SearchObjectActionServer()
  : Node("search_object_action_server"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    action_server_ = rclcpp_action::create_server<SearchObject>(
      this, "search_object",
      [this](auto uuid, auto goal) { return on_goal(uuid, goal); },
      [this](auto gh)              { return on_cancel(gh); },
      [this](auto gh)              { on_accepted(gh); });

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/vision/detections", 10,
      [this](vision_msgs::msg::Detection2DArray::SharedPtr m) {
        std::lock_guard lock(mtx_); latest_dets_ = m;
      }, sub_opts);

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      [this](sensor_msgs::msg::Image::SharedPtr m) {
        std::lock_guard lock(mtx_); latest_rgb_ = m;
      }, sub_opts);

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/depth_camera/image_raw", 10,
      [this](sensor_msgs::msg::Image::SharedPtr m) {
        std::lock_guard lock(mtx_); latest_depth_ = m;
      }, sub_opts);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/depth_camera/camera_info", 1,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr m) {
        std::lock_guard lock(mtx_);
        latest_info_ = *m;
        has_info_    = true;
      }, sub_opts);

    RCLCPP_INFO(get_logger(), "SearchObject action server ready on /search_object");
  }

private:
  rclcpp::CallbackGroup::SharedPtr                                   cb_group_;
  rclcpp_action::Server<SearchObject>::SharedPtr                     action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            cmd_vel_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr       info_sub_;

  tf2_ros::Buffer             tf_buffer_;
  tf2_ros::TransformListener  tf_listener_;

  std::mutex                                         mtx_;
  vision_msgs::msg::Detection2DArray::SharedPtr      latest_dets_;
  sensor_msgs::msg::Image::SharedPtr                 latest_rgb_;
  sensor_msgs::msg::Image::SharedPtr                 latest_depth_;
  sensor_msgs::msg::CameraInfo                       latest_info_;
  bool                                               has_info_{false};

  // ── action callbacks ──────────────────────────────────────────────────────

  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SearchObject::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Search goal: class='%s'  min_conf=%.2f",
      goal->target_class.c_str(), goal->min_confidence);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(std::shared_ptr<ServerGH>)
  {
    RCLCPP_INFO(get_logger(), "Search cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(std::shared_ptr<ServerGH> gh)
  {
    std::thread{[this, gh] { execute(gh); }}.detach();
  }

  // ── execution (detached worker thread) ────────────────────────────────────

  void execute(std::shared_ptr<ServerGH> server_gh)
  {
    const auto & goal = *server_gh->get_goal();
    auto result       = std::make_shared<SearchObject::Result>();

    std::string target = goal.target_class;
    std::transform(target.begin(), target.end(), target.begin(), ::tolower);

    const double scan_duration = static_cast<double>(SCAN_ANGLE / SCAN_SPEED);
    const auto   t_start       = std::chrono::steady_clock::now();

    int det_count = 0;

    geometry_msgs::msg::Twist spin_cmd;
    spin_cmd.angular.z = SCAN_SPEED;

    rclcpp::Rate rate(10.0);  // 10 Hz main loop

    while (rclcpp::ok()) {
      if (server_gh->is_canceling()) {
        stop_robot();
        result->found = false;
        server_gh->canceled(result);
        return;
      }

      const double elapsed  = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
      const float  progress = std::min(100.0f,
        static_cast<float>(elapsed / scan_duration * 100.0));

      // Snapshot shared state
      vision_msgs::msg::Detection2DArray::SharedPtr dets;
      sensor_msgs::msg::Image::SharedPtr            rgb_msg;
      sensor_msgs::msg::Image::SharedPtr            depth_msg;
      sensor_msgs::msg::CameraInfo                  cam_info;
      bool info_ready;
      {
        std::lock_guard lock(mtx_);
        dets      = latest_dets_;
        rgb_msg   = latest_rgb_;
        depth_msg = latest_depth_;
        cam_info  = latest_info_;
        info_ready = has_info_;
      }

      // Find best detection for target class this tick
      bool  found_now = false;
      float best_conf = 0.0f;
      int   best_u = 0, best_v = 0;

      if (dets) {
        const double age = (get_clock()->now() -
          rclcpp::Time(dets->header.stamp)).seconds();
        if (age < 2.0) {
          for (const auto & det : dets->detections) {
            for (const auto & hyp : det.results) {
              std::string cls = hyp.hypothesis.class_id;
              std::transform(cls.begin(), cls.end(), cls.begin(), ::tolower);
              if (cls == target && hyp.hypothesis.score >= goal.min_confidence
                  && hyp.hypothesis.score > best_conf)
              {
                best_conf  = static_cast<float>(hyp.hypothesis.score);
                best_u     = static_cast<int>(det.bbox.center.position.x);
                best_v     = static_cast<int>(det.bbox.center.position.y);
                found_now  = true;
              }
            }
          }
        }
      }

      if (found_now) {
        det_count++;
        stop_robot();

        double x = 0.0, y = 0.0, z = 0.0;
        get_3d_position(best_u, best_v, depth_msg, cam_info, info_ready, x, y, z);

        std::string img_path = save_image(rgb_msg, goal.target_class);

        auto fb = std::make_shared<SearchObject::Feedback>();
        fb->search_state     = "Found";
        fb->progress_percent = progress;
        fb->detections_count = det_count;
        server_gh->publish_feedback(fb);

        result->found      = true;
        result->x          = x;
        result->y          = y;
        result->z          = z;
        result->image_path = img_path;
        server_gh->succeed(result);
        return;
      }

      // Full rotation complete — not found
      if (elapsed >= scan_duration) {
        stop_robot();
        auto fb = std::make_shared<SearchObject::Feedback>();
        fb->search_state     = "Not found";
        fb->progress_percent = 100.0f;
        fb->detections_count = det_count;
        server_gh->publish_feedback(fb);
        result->found = false;
        server_gh->abort(result);
        return;
      }

      // Still scanning — spin and publish feedback
      cmd_vel_pub_->publish(spin_cmd);

      auto fb = std::make_shared<SearchObject::Feedback>();
      fb->search_state = det_count > 0
        ? "Detected (" + std::to_string(det_count) + ")"
        : "Scanning";
      fb->progress_percent = progress;
      fb->detections_count = det_count;
      server_gh->publish_feedback(fb);

      rate.sleep();
    }

    stop_robot();
    result->found = false;
    server_gh->abort(result);
  }

  // ── helpers ───────────────────────────────────────────────────────────────

  void stop_robot()
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }

  void get_3d_position(
    int u, int v,
    const sensor_msgs::msg::Image::SharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo & cam_info,
    bool info_ready,
    double & x, double & y, double & z)
  {
    if (!depth_msg || !info_ready) return;

    try {
      auto cv_depth = cv_bridge::toCvCopy(depth_msg, "32FC1");
      const auto & mat = cv_depth->image;

      float depth = NAN;
      for (int r = 0; r <= DEPTH_RADIUS && !std::isfinite(depth); ++r) {
        for (int du = -r; du <= r && !std::isfinite(depth); ++du) {
          for (int dv = -r; dv <= r && !std::isfinite(depth); ++dv) {
            const int nu = u + du, nv = v + dv;
            if (nu >= 0 && nu < mat.cols && nv >= 0 && nv < mat.rows) {
              const float d = mat.at<float>(nv, nu);
              if (std::isfinite(d) && d > 0.0f) depth = d;
            }
          }
        }
      }

      if (!std::isfinite(depth)) return;

      const double fx = cam_info.k[0], fy = cam_info.k[4];
      const double cx = cam_info.k[2], cy = cam_info.k[5];

      geometry_msgs::msg::PointStamped pt;
      pt.header.frame_id = "depth_optical_frame";
      pt.header.stamp    = depth_msg->header.stamp;
      pt.point.x = (u - cx) * depth / fx;
      pt.point.y = (v - cy) * depth / fy;
      pt.point.z = depth;

      for (const std::string frame : {"map", "odom"}) {
        try {
          auto pt_out = tf_buffer_.transform(pt, frame,
            tf2::durationFromSec(1.0));
          x = pt_out.point.x;
          y = pt_out.point.y;
          z = pt_out.point.z;
          return;
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN(get_logger(), "TF to %s failed: %s", frame.c_str(), e.what());
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Depth processing error: %s", e.what());
    }
  }

  std::string save_image(
    const sensor_msgs::msg::Image::SharedPtr & rgb_msg,
    const std::string & label)
  {
    if (!rgb_msg) return {};
    try {
      auto cv_rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8");
      const auto ts = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      std::string path = "/tmp/search_" + label + "_" + std::to_string(ts) + ".jpg";
      if (cv::imwrite(path, cv_rgb->image)) return path;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Image save error: %s", e.what());
    }
    return {};
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchObjectActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
