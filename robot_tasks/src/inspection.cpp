#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────

struct Waypoint { double x, y; };

// Running aggregate of all sightings of one object instance
struct Sighting {
  std::string class_id;
  double      confidence;
  double      world_x, world_y;
  int         count;
};

enum class State { ALIGNING, DRIVING, AVOIDING, SCANNING, DONE };

// ─────────────────────────────────────────────────────────────────────────────

class InspectionNode : public rclcpp::Node
{
public:
  InspectionNode()
  : Node("inspection"), current_wp_(0), state_(State::ALIGNING)
  {
    // ── Parameters ────────────────────────────────────────────────────
    declare_parameter("waypoints_x",     std::vector<double>{0.0, 2.0, 2.0, 0.0});
    declare_parameter("waypoints_y",     std::vector<double>{0.0, 0.0, 2.0, 2.0});
    declare_parameter("frame_id",        std::string("odom"));
    declare_parameter("forward_speed",   0.20);
    declare_parameter("turn_speed",      0.60);
    declare_parameter("scan_turn_speed", 0.80);  // rad/s during 360° scan
    declare_parameter("goal_tolerance",  0.30);
    declare_parameter("safety_dist",     0.45);
    declare_parameter("align_threshold", 0.15);
    declare_parameter("front_half_deg",  30.0);
    declare_parameter("scan_duration",   8.0);   // seconds — covers ~360° at 0.8 rad/s
    declare_parameter("camera_fov_deg",  90.0);  // horizontal FOV of front camera
    declare_parameter("img_width",       640.0); // detection bbox pixel width
    declare_parameter("merge_dist",      0.50);  // m — merge duplicate sightings
    declare_parameter("loop",            false);

    auto wx = get_parameter("waypoints_x").as_double_array();
    auto wy = get_parameter("waypoints_y").as_double_array();
    if (wx.empty() || wx.size() != wy.size()) {
      RCLCPP_FATAL(get_logger(),
        "waypoints_x and waypoints_y must be non-empty and the same length.");
      rclcpp::shutdown();
      return;
    }
    for (std::size_t i = 0; i < wx.size(); ++i) {
      waypoints_.push_back({wx[i], wy[i]});
    }

    frame_id_       = get_parameter("frame_id").as_string();
    fwd_            = get_parameter("forward_speed").as_double();
    turn_           = get_parameter("turn_speed").as_double();
    scan_turn_      = get_parameter("scan_turn_speed").as_double();
    tol_            = get_parameter("goal_tolerance").as_double();
    safe_           = get_parameter("safety_dist").as_double();
    align_th_       = get_parameter("align_threshold").as_double();
    fhalf_          = get_parameter("front_half_deg").as_double() * M_PI / 180.0;
    scan_duration_  = get_parameter("scan_duration").as_double();
    camera_fov_rad_ = get_parameter("camera_fov_deg").as_double() * M_PI / 180.0;
    img_width_      = get_parameter("img_width").as_double();
    merge_dist_     = get_parameter("merge_dist").as_double();
    loop_           = get_parameter("loop").as_bool();

    // ── TF ────────────────────────────────────────────────────────────
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    // ── Subscriptions ─────────────────────────────────────────────────
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr m){ scan_ = m; });

    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/vision/detections", 10,
      std::bind(&InspectionNode::on_detections, this, std::placeholders::_1));

    // ── Publishers ────────────────────────────────────────────────────
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mk_pub_  = create_publisher<visualization_msgs::msg::MarkerArray>(
                 "/inspection/markers", 10);

    // ── Timers ────────────────────────────────────────────────────────
    ctrl_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&InspectionNode::tick, this));

    viz_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&InspectionNode::publish_markers, this));

    RCLCPP_INFO(get_logger(),
      "Inspection ready: %zu waypoints  scan=%.1f s  loop=%s",
      waypoints_.size(), scan_duration_, loop_ ? "true" : "false");
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  WP[%zu]  x=%.2f  y=%.2f",
        i, waypoints_[i].x, waypoints_[i].y);
    }
  }

private:
  // ── State ─────────────────────────────────────────────────────────────
  std::vector<Waypoint> waypoints_;
  std::size_t           current_wp_;
  State                 state_;
  rclcpp::Time          scan_start_;

  std::vector<Sighting> sightings_;

  // ── Params ────────────────────────────────────────────────────────────
  std::string frame_id_;
  double fwd_, turn_, scan_turn_, tol_, safe_, align_th_, fhalf_;
  double scan_duration_, camera_fov_rad_, img_width_, merge_dist_;
  bool   loop_;

  // ── ROS ───────────────────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan::SharedPtr                               scan_;
  std::shared_ptr<tf2_ros::Buffer>                                     tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener>                          tf_lis_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr              cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr   mk_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr         scan_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr  det_sub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_, viz_timer_;

  // ── Helpers ───────────────────────────────────────────────────────────

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

  // Nearest finite LiDAR range within a 5° window around `angle` (robot frame)
  double range_at(double angle) const
  {
    if (!scan_) { return std::numeric_limits<double>::infinity(); }
    constexpr double kWindow = 5.0 * M_PI / 180.0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
      double a = scan_->angle_min + i * scan_->angle_increment;
      if (std::fabs(a - angle) > kWindow) { continue; }
      float r = scan_->ranges[i];
      if (std::isfinite(r) && r >= scan_->range_min && r <= scan_->range_max) {
        best = std::min(best, static_cast<double>(r));
      }
    }
    return best;
  }

  void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

  bool get_pose(double & rx, double & ry, double & ryaw) const
  {
    try {
      auto tf = tf_buf_->lookupTransform(frame_id_, "base_footprint", tf2::TimePointZero);
      rx   = tf.transform.translation.x;
      ry   = tf.transform.translation.y;
      const auto & q = tf.transform.rotation;
      ryaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (...) { return false; }
  }

  void record_sighting(const std::string & cls, double conf, double wx, double wy)
  {
    // Merge into an existing sighting of the same class within merge_dist_
    for (auto & s : sightings_) {
      if (s.class_id == cls &&
          std::hypot(s.world_x - wx, s.world_y - wy) < merge_dist_)
      {
        s.world_x    = (s.world_x * s.count + wx) / (s.count + 1);
        s.world_y    = (s.world_y * s.count + wy) / (s.count + 1);
        s.confidence = std::max(s.confidence, conf);
        ++s.count;
        return;
      }
    }
    sightings_.push_back({cls, conf, wx, wy, 1});
    RCLCPP_INFO(get_logger(),
      "[DETECT] %s  conf=%.2f  world=(%.2f, %.2f)",
      cls.c_str(), conf, wx, wy);
  }

  // ── Detection callback (processes only during SCANNING) ───────────────

  void on_detections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (state_ != State::SCANNING) { return; }

    double rx, ry, ryaw;
    if (!get_pose(rx, ry, ryaw)) { return; }

    for (const auto & det : msg->detections) {
      if (det.results.empty()) { continue; }
      const auto & hyp  = det.results[0].hypothesis;
      const double conf = hyp.score;
      const auto & cls  = hyp.class_id;

      // Normalised horizontal offset in [-0.5, 0.5], then scale to camera FOV
      const double nx      = det.bbox.center.position.x / img_width_ - 0.5;
      const double bearing = nx * camera_fov_rad_;  // angle in robot frame

      // Range from LiDAR at this bearing; fall back to 1.5 m if no reading
      double range = range_at(bearing);
      if (!std::isfinite(range)) { range = 1.5; }

      const double world_angle = ryaw + bearing;
      const double obj_x       = rx + range * std::cos(world_angle);
      const double obj_y       = ry + range * std::sin(world_angle);

      record_sighting(cls, conf, obj_x, obj_y);
    }
  }

  // ── Control tick (10 Hz) ──────────────────────────────────────────────

  void tick()
  {
    if (state_ == State::DONE) { return; }

    double rx, ry, ryaw;
    if (!get_pose(rx, ry, ryaw)) { return; }

    // ── SCANNING: rotate for scan_duration_ seconds ───────────────────
    if (state_ == State::SCANNING) {
      const double elapsed = (get_clock()->now() - scan_start_).seconds();
      if (elapsed >= scan_duration_) {
        stop();
        RCLCPP_INFO(get_logger(),
          "Scan complete at WP[%zu] — %zu total unique sightings",
          current_wp_, sightings_.size());
        advance_waypoint();
      } else {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = scan_turn_;
        cmd_pub_->publish(cmd);
      }
      return;
    }

    const Waypoint & wp   = waypoints_[current_wp_];
    const double     dx   = wp.x - rx;
    const double     dy   = wp.y - ry;
    const double     dist = std::hypot(dx, dy);

    geometry_msgs::msg::Twist cmd;

    // ── Obstacle avoidance ────────────────────────────────────────────
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
    if (state_ == State::AVOIDING) { state_ = State::ALIGNING; }

    // ── Reached waypoint → start 360° scan ───────────────────────────
    if (dist < tol_) {
      stop();
      RCLCPP_INFO(get_logger(),
        "At WP[%zu/%zu]  (%.2f, %.2f) — scanning for %.1f s …",
        current_wp_, waypoints_.size() - 1, wp.x, wp.y, scan_duration_);
      state_      = State::SCANNING;
      scan_start_ = get_clock()->now();
      return;
    }

    // ── Align then drive ──────────────────────────────────────────────
    const double desired_yaw = std::atan2(dy, dx);
    const double err = std::atan2(
      std::sin(desired_yaw - ryaw),
      std::cos(desired_yaw - ryaw));

    if (std::fabs(err) > align_th_) {
      state_        = State::ALIGNING;
      cmd.angular.z = turn_ * (err > 0.0 ? 1.0 : -1.0);
    } else {
      state_        = State::DRIVING;
      cmd.linear.x  = fwd_ * std::min(1.0, dist / 1.0);
      cmd.angular.z = 0.5 * err;
    }
    cmd_pub_->publish(cmd);
  }

  // ── Advance to next waypoint ──────────────────────────────────────────

  void advance_waypoint()
  {
    const std::size_t next = current_wp_ + 1;
    if (next >= waypoints_.size()) {
      if (loop_) {
        current_wp_ = 0;
        RCLCPP_INFO(get_logger(), "Inspection loop complete — restarting from WP[0]");
      } else {
        RCLCPP_INFO(get_logger(),
          "Inspection complete.  %zu unique objects detected:", sightings_.size());
        for (const auto & s : sightings_) {
          RCLCPP_INFO(get_logger(),
            "  %-20s  conf=%.2f  pos=(%.2f, %.2f)  seen=%d×",
            s.class_id.c_str(), s.confidence, s.world_x, s.world_y, s.count);
        }
        state_ = State::DONE;
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
    visualization_msgs::msg::MarkerArray arr;
    const auto now = get_clock()->now();

    // Clear previous frame
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    // ── Waypoints ────────────────────────────────────────────────────
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id    = frame_id_;
      mk.header.stamp       = now;
      mk.ns                 = "inspection_wps";
      mk.id                 = static_cast<int>(i);
      mk.type               = visualization_msgs::msg::Marker::CYLINDER;
      mk.action             = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x    = waypoints_[i].x;
      mk.pose.position.y    = waypoints_[i].y;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = 0.25;
      mk.scale.z              = 0.40;
      if (i == current_wp_ && state_ == State::SCANNING) {
        mk.color.r = 1.0; mk.color.g = 0.5; mk.color.a = 1.0;  // orange: scanning
      } else if (i == current_wp_) {
        mk.color.r = 1.0; mk.color.g = 1.0; mk.color.a = 1.0;  // yellow: active
      } else {
        mk.color.b = 1.0; mk.color.a = 0.60;                    // blue: pending
      }
      arr.markers.push_back(mk);

      // Label
      visualization_msgs::msg::Marker lbl;
      lbl.header.frame_id    = frame_id_;
      lbl.header.stamp       = now;
      lbl.ns                 = "inspection_wp_labels";
      lbl.id                 = static_cast<int>(i);
      lbl.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      lbl.action             = visualization_msgs::msg::Marker::ADD;
      lbl.pose.position.x    = waypoints_[i].x;
      lbl.pose.position.y    = waypoints_[i].y;
      lbl.pose.position.z    = 0.55;
      lbl.pose.orientation.w = 1.0;
      lbl.scale.z            = 0.22;
      lbl.color.r = lbl.color.g = lbl.color.b = 1.0;
      lbl.color.a            = 1.0;
      lbl.text               = "WP" + std::to_string(i);
      arr.markers.push_back(lbl);
    }

    // ── Detected objects ─────────────────────────────────────────────
    for (std::size_t i = 0; i < sightings_.size(); ++i) {
      const auto & s = sightings_[i];

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id    = frame_id_;
      sphere.header.stamp       = now;
      sphere.ns                 = "detections";
      sphere.id                 = static_cast<int>(i);
      sphere.type               = visualization_msgs::msg::Marker::SPHERE;
      sphere.action             = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x    = s.world_x;
      sphere.pose.position.y    = s.world_y;
      sphere.pose.position.z    = 0.50;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.30;
      sphere.color.r = 1.0; sphere.color.g = 0.2; sphere.color.a = 0.90;
      arr.markers.push_back(sphere);

      visualization_msgs::msg::Marker txt;
      txt.header.frame_id    = frame_id_;
      txt.header.stamp       = now;
      txt.ns                 = "detection_labels";
      txt.id                 = static_cast<int>(i);
      txt.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action             = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x    = s.world_x;
      txt.pose.position.y    = s.world_y;
      txt.pose.position.z    = 0.85;
      txt.pose.orientation.w = 1.0;
      txt.scale.z            = 0.20;
      txt.color.r = txt.color.g = txt.color.b = 1.0;
      txt.color.a            = 1.0;
      txt.text = s.class_id + " (" +
                 std::to_string(static_cast<int>(s.confidence * 100.0)) + "%)";
      arr.markers.push_back(txt);
    }

    mk_pub_->publish(arr);
  }
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InspectionNode>());
  rclcpp::shutdown();
  return 0;
}
