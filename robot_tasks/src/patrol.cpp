#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <limits>
#include <vector>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────

struct Waypoint { double x, y, yaw; };

enum class State { ALIGNING, DRIVING, AVOIDING, WAITING };

// ─────────────────────────────────────────────────────────────────────────────

class PatrolNode : public rclcpp::Node
{
public:
  PatrolNode()
  : Node("patrol"), current_wp_(0), state_(State::ALIGNING), waiting_(false)
  {
    // ── Parameters ────────────────────────────────────────────────────
    declare_parameter("waypoints_x",     std::vector<double>{});
    declare_parameter("waypoints_y",     std::vector<double>{});
    declare_parameter("waypoints_yaw",   std::vector<double>{});
    declare_parameter("frame_id",        std::string("odom"));
    declare_parameter("forward_speed",   0.20);
    declare_parameter("turn_speed",      0.60);
    declare_parameter("goal_tolerance",  0.30);
    declare_parameter("safety_dist",     0.45);
    declare_parameter("align_threshold", 0.15);
    declare_parameter("front_half_deg",  30.0);
    declare_parameter("wait_time",       1.50);  // seconds to pause at each waypoint
    declare_parameter("loop",            true);   // repeat patrol indefinitely

    auto wx   = get_parameter("waypoints_x").as_double_array();
    auto wy   = get_parameter("waypoints_y").as_double_array();
    auto wyaw = get_parameter("waypoints_yaw").as_double_array();

    if (wx.empty() || wx.size() != wy.size()) {
      RCLCPP_FATAL(get_logger(),
        "waypoints_x and waypoints_y must be non-empty and the same length.");
      rclcpp::shutdown();
      return;
    }
    for (std::size_t i = 0; i < wx.size(); ++i) {
      waypoints_.push_back({wx[i], wy[i], (i < wyaw.size()) ? wyaw[i] : 0.0});
    }

    frame_id_   = get_parameter("frame_id").as_string();
    fwd_        = get_parameter("forward_speed").as_double();
    turn_       = get_parameter("turn_speed").as_double();
    tol_        = get_parameter("goal_tolerance").as_double();
    safe_       = get_parameter("safety_dist").as_double();
    align_th_   = get_parameter("align_threshold").as_double();
    fhalf_      = get_parameter("front_half_deg").as_double() * M_PI / 180.0;
    wait_time_  = get_parameter("wait_time").as_double();
    loop_       = get_parameter("loop").as_bool();

    // ── TF ────────────────────────────────────────────────────────────
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    // ── Subs / pubs ───────────────────────────────────────────────────
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr m){ scan_ = m; });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mk_pub_  = create_publisher<visualization_msgs::msg::MarkerArray>(
                 "/patrol_markers", 10);

    ctrl_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PatrolNode::tick, this));

    viz_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PatrolNode::publish_markers, this));

    // ── Log waypoint list ─────────────────────────────────────────────
    RCLCPP_INFO(get_logger(),
      "Patrol ready: %zu waypoints in frame [%s]  loop=%s",
      waypoints_.size(), frame_id_.c_str(), loop_ ? "true" : "false");

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  WP[%zu]  x=%.2f  y=%.2f  yaw=%.2f",
        i, waypoints_[i].x, waypoints_[i].y, waypoints_[i].yaw);
    }
  }

private:
  // ── State ────────────────────────────────────────────────────────────
  std::vector<Waypoint> waypoints_;
  std::size_t           current_wp_;
  State                 state_;
  bool                  waiting_;
  rclcpp::Time          wait_start_;

  // ── Params ───────────────────────────────────────────────────────────
  std::string frame_id_;
  double fwd_, turn_, tol_, safe_, align_th_, fhalf_, wait_time_;
  bool   loop_;

  // ── ROS ──────────────────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  mk_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr        scan_sub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_, viz_timer_;

  // ── Helpers ──────────────────────────────────────────────────────────

  double sector_min(double lo, double hi) const
  {
    if (!scan_) { return std::numeric_limits<double>::infinity(); }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
      double a = scan_->angle_min + i * scan_->angle_increment;
      if (a < lo || a > hi) { continue; }
      float r = scan_->ranges[i];
      if (std::isfinite(r) && r >= scan_->range_min && r <= scan_->range_max) {
        best = std::min(best, static_cast<double>(r));
      }
    }
    return best;
  }

  void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

  // ── Control tick (10 Hz) ─────────────────────────────────────────────

  void tick()
  {
    if (waypoints_.empty()) { return; }

    // ── Get robot pose ───────────────────────────────────────────────
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buf_->lookupTransform(frame_id_, "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const double rx   = tf.transform.translation.x;
    const double ry   = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    const double ryaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // ── Waiting at waypoint ───────────────────────────────────────────
    if (waiting_) {
      const double elapsed = (get_clock()->now() - wait_start_).seconds();
      if (elapsed >= wait_time_) {
        waiting_ = false;
        advance_waypoint();
      }
      return;
    }

    const Waypoint & wp = waypoints_[current_wp_];
    const double dx   = wp.x - rx;
    const double dy   = wp.y - ry;
    const double dist = std::hypot(dx, dy);

    geometry_msgs::msg::Twist cmd;

    // ── Obstacle avoidance override ───────────────────────────────────
    const double d_front = sector_min(-fhalf_, fhalf_);
    if (d_front < safe_) {
      if (state_ != State::AVOIDING) {
        RCLCPP_WARN(get_logger(), "Obstacle (%.2f m) — avoiding", d_front);
        state_ = State::AVOIDING;
      }
      const double d_left  = sector_min( fhalf_,  M_PI_2);
      const double d_right = sector_min(-M_PI_2, -fhalf_);
      cmd.angular.z = (d_left >= d_right) ? turn_ : -turn_;
      cmd_pub_->publish(cmd);
      return;
    }
    if (state_ == State::AVOIDING) {
      state_ = State::ALIGNING;
    }

    // ── Goal reached ──────────────────────────────────────────────────
    if (dist < tol_) {
      RCLCPP_INFO(get_logger(),
        "Reached WP[%zu/%zu]  (%.2f, %.2f)  waiting %.1f s …",
        current_wp_, waypoints_.size() - 1, wp.x, wp.y, wait_time_);
      stop();
      waiting_    = true;
      wait_start_ = get_clock()->now();
      return;
    }

    // ── Heading error ─────────────────────────────────────────────────
    const double desired_yaw = std::atan2(dy, dx);
    const double err = std::atan2(
      std::sin(desired_yaw - ryaw),
      std::cos(desired_yaw - ryaw));

    if (std::abs(err) > align_th_) {
      if (state_ != State::ALIGNING) { state_ = State::ALIGNING; }
      cmd.angular.z = turn_ * (err > 0.0 ? 1.0 : -1.0);
    } else {
      if (state_ != State::DRIVING) { state_ = State::DRIVING; }
      cmd.linear.x  = fwd_ * std::min(1.0, dist / 1.0);
      cmd.angular.z = 0.5 * err;
    }
    cmd_pub_->publish(cmd);
  }

  // ── Advance to next waypoint ─────────────────────────────────────────

  void advance_waypoint()
  {
    const std::size_t next = current_wp_ + 1;
    if (next >= waypoints_.size()) {
      if (loop_) {
        current_wp_ = 0;
        RCLCPP_INFO(get_logger(), "Patrol loop complete — restarting from WP[0]");
      } else {
        RCLCPP_INFO(get_logger(), "Patrol complete.");
        ctrl_timer_->cancel();
        return;
      }
    } else {
      current_wp_ = next;
    }
    RCLCPP_INFO(get_logger(),
      "→ WP[%zu/%zu]  (%.2f, %.2f)",
      current_wp_, waypoints_.size() - 1,
      waypoints_[current_wp_].x, waypoints_[current_wp_].y);
    state_ = State::ALIGNING;
  }

  // ── RViz markers ─────────────────────────────────────────────────────

  void publish_markers()
  {
    if (waypoints_.empty()) { return; }

    visualization_msgs::msg::MarkerArray arr;
    const auto now = get_clock()->now();

    // Clear previous
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    // ── Closed path (blue line strip) ────────────────────────────────
    visualization_msgs::msg::Marker path;
    path.header.frame_id = frame_id_;
    path.header.stamp    = now;
    path.ns              = "patrol_path";
    path.id              = 0;
    path.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action          = visualization_msgs::msg::Marker::ADD;
    path.scale.x         = 0.04;
    path.color.b         = 1.0;
    path.color.a         = 0.5;
    path.pose.orientation.w = 1.0;
    for (const auto & wp : waypoints_) {
      geometry_msgs::msg::Point p;
      p.x = wp.x; p.y = wp.y; p.z = 0.05;
      path.points.push_back(p);
    }
    {  // close the loop
      geometry_msgs::msg::Point p;
      p.x = waypoints_[0].x; p.y = waypoints_[0].y; p.z = 0.05;
      path.points.push_back(p);
    }
    arr.markers.push_back(path);

    // ── Waypoint cylinders ────────────────────────────────────────────
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id    = frame_id_;
      mk.header.stamp       = now;
      mk.ns                 = "patrol_wps";
      mk.id                 = static_cast<int>(i);
      mk.type               = visualization_msgs::msg::Marker::CYLINDER;
      mk.action             = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x    = waypoints_[i].x;
      mk.pose.position.y    = waypoints_[i].y;
      mk.pose.orientation.w = 1.0;

      if (i == current_wp_ && !waiting_) {
        // Active target — yellow, taller
        mk.scale.x = mk.scale.y = 0.35;
        mk.scale.z = 0.60;
        mk.color.r = 1.0; mk.color.g = 1.0; mk.color.a = 1.0;
      } else if (i == current_wp_ && waiting_) {
        // Waiting here — green
        mk.scale.x = mk.scale.y = 0.35;
        mk.scale.z = 0.60;
        mk.color.g = 1.0; mk.color.a = 1.0;
      } else {
        // Upcoming — blue
        mk.scale.x = mk.scale.y = 0.20;
        mk.scale.z = 0.30;
        mk.color.b = 1.0; mk.color.a = 0.80;
      }
      arr.markers.push_back(mk);

      // Index label
      visualization_msgs::msg::Marker txt;
      txt.header.frame_id    = frame_id_;
      txt.header.stamp       = now;
      txt.ns                 = "patrol_labels";
      txt.id                 = static_cast<int>(i);
      txt.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action             = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x    = waypoints_[i].x;
      txt.pose.position.y    = waypoints_[i].y;
      txt.pose.position.z    = 0.70;
      txt.pose.orientation.w = 1.0;
      txt.scale.z            = 0.25;
      txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 1.0;
      txt.text               = std::to_string(i);
      arr.markers.push_back(txt);
    }

    mk_pub_->publish(arr);
  }
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNode>());
  rclcpp::shutdown();
  return 0;
}
