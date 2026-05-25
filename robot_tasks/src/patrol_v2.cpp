#include <iomanip>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/detect_object.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using DetectObject = robot_interfaces::srv::DetectObject;
using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────

struct Waypoint { double x, y, yaw; };
enum class State { ALIGNING, DRIVING, AVOIDING, WAITING };

struct DetResult {
  std::string target_class;
  bool        found{false};
  float       confidence{0.f};
  double      obj_x{0.0}, obj_y{0.0};
};

struct WpRecord {
  std::size_t         idx;
  double              x, y;
  std::vector<DetResult> results;   // one entry per target class
};

// ─────────────────────────────────────────────────────────────────────────────

class PatrolV2Node : public rclcpp::Node
{
public:
  PatrolV2Node()
  : Node("patrol_v2"),
    current_wp_(0),
    state_(State::ALIGNING),
    waiting_(false),
    detecting_(false)
  {
    // ── Parameters ────────────────────────────────────────────────────────────
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
    declare_parameter("wait_time",       1.50);
    declare_parameter("loop",            false); // one pass then report by default
    declare_parameter("target_classes",
      std::vector<std::string>{"fire extinguisher"});
    declare_parameter("scan_wait",       0.50);  // settle pause before calling service

    const auto wx   = get_parameter("waypoints_x").as_double_array();
    const auto wy   = get_parameter("waypoints_y").as_double_array();
    const auto wyaw = get_parameter("waypoints_yaw").as_double_array();

    if (wx.empty() || wx.size() != wy.size()) {
      RCLCPP_FATAL(get_logger(),
        "waypoints_x and waypoints_y must be non-empty and equal length.");
      rclcpp::shutdown();
      return;
    }
    for (std::size_t i = 0; i < wx.size(); ++i) {
      waypoints_.push_back({wx[i], wy[i], (i < wyaw.size()) ? wyaw[i] : 0.0});
      records_.push_back({i, wx[i], wy[i], {}});
    }

    frame_id_       = get_parameter("frame_id").as_string();
    fwd_            = get_parameter("forward_speed").as_double();
    turn_           = get_parameter("turn_speed").as_double();
    tol_            = get_parameter("goal_tolerance").as_double();
    safe_           = get_parameter("safety_dist").as_double();
    align_th_       = get_parameter("align_threshold").as_double();
    fhalf_          = get_parameter("front_half_deg").as_double() * M_PI / 180.0;
    wait_time_      = get_parameter("wait_time").as_double();
    loop_           = get_parameter("loop").as_bool();
    target_classes_ = get_parameter("target_classes").as_string_array();
    scan_wait_      = get_parameter("scan_wait").as_double();

    // ── TF ────────────────────────────────────────────────────────────────────
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    // ── DetectObject client (Reentrant — responses processed while tick runs) ─
    detect_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    detect_client_ = create_client<DetectObject>(
      "/detect_object",
      rmw_qos_profile_services_default,
      detect_cbg_);

    // ── Subs / pubs ───────────────────────────────────────────────────────────
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr m){ scan_ = m; });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mk_pub_  = create_publisher<visualization_msgs::msg::MarkerArray>(
                 "/patrol_markers", 10);

    ctrl_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PatrolV2Node::tick, this));

    viz_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PatrolV2Node::publish_markers, this));

    RCLCPP_INFO(get_logger(),
      "PatrolV2 ready — %zu waypoints  loop=%s  targets=[%s]",
      waypoints_.size(), loop_ ? "true" : "false",
      targets_str().c_str());

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  WP[%zu]  x=%.2f  y=%.2f  yaw=%.2f",
        i, waypoints_[i].x, waypoints_[i].y, waypoints_[i].yaw);
    }
  }

private:
  // ── State ─────────────────────────────────────────────────────────────────
  std::vector<Waypoint>  waypoints_;
  std::size_t            current_wp_;
  State                  state_;
  bool                   waiting_;
  rclcpp::Time           wait_start_;
  std::atomic<bool>      detecting_;  // true while detection thread is running
  std::vector<WpRecord>  records_;    // results per waypoint (reset on loop)

  // ── Params ────────────────────────────────────────────────────────────────
  std::string              frame_id_;
  double fwd_, turn_, tol_, safe_, align_th_, fhalf_, wait_time_, scan_wait_;
  bool                     loop_;
  std::vector<std::string> target_classes_;

  // ── ROS ───────────────────────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::shared_ptr<tf2_ros::Buffer>             tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener>  tf_lis_;
  rclcpp::CallbackGroup::SharedPtr             detect_cbg_;
  rclcpp::Client<DetectObject>::SharedPtr      detect_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mk_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_, viz_timer_;

  // ── Helpers ───────────────────────────────────────────────────────────────

  std::string targets_str() const
  {
    std::string s;
    for (const auto & t : target_classes_) {
      if (!s.empty()) s += ", ";
      s += t;
    }
    return s;
  }

  double sector_min(double lo, double hi) const
  {
    if (!scan_) { return std::numeric_limits<double>::infinity(); }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double a = scan_->angle_min + i * scan_->angle_increment;
      if (a < lo || a > hi) { continue; }
      const float r = scan_->ranges[i];
      if (std::isfinite(r) && r >= scan_->range_min && r <= scan_->range_max)
        best = std::min(best, static_cast<double>(r));
    }
    return best;
  }

  void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

  // ── Control tick (10 Hz) ──────────────────────────────────────────────────

  void tick()
  {
    if (waypoints_.empty()) { return; }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buf_->lookupTransform(frame_id_, "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const double rx   = tf.transform.translation.x;
    const double ry   = tf.transform.translation.y;
    const auto & q    = tf.transform.rotation;
    const double ryaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // ── Hold at waypoint until detection thread finishes, then wait_time ─────
    if (waiting_) {
      if (detecting_) { return; }   // detection still running — keep holding
      if ((get_clock()->now() - wait_start_).seconds() >= wait_time_) {
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

    // ── Obstacle avoidance ────────────────────────────────────────────────────
    const double d_front = sector_min(-fhalf_, fhalf_);
    if (d_front < safe_) {
      if (state_ != State::AVOIDING)
        RCLCPP_WARN(get_logger(), "Obstacle at %.2f m — avoiding", d_front);
      state_ = State::AVOIDING;
      const double d_left  = sector_min( fhalf_,  M_PI_2);
      const double d_right = sector_min(-M_PI_2, -fhalf_);
      cmd.angular.z = (d_left >= d_right) ? turn_ : -turn_;
      cmd_pub_->publish(cmd);
      return;
    }
    if (state_ == State::AVOIDING) state_ = State::ALIGNING;

    // ── Goal reached — stop, launch detection thread ──────────────────────────
    if (dist < tol_) {
      stop();
      RCLCPP_INFO(get_logger(), "Reached WP[%zu/%zu]  (%.2f, %.2f)",
        current_wp_, waypoints_.size() - 1, wp.x, wp.y);
      waiting_    = true;
      wait_start_ = get_clock()->now();

      if (!target_classes_.empty()) {
        detecting_            = true;
        const std::size_t idx = current_wp_;
        std::thread([this, idx] {
          std::this_thread::sleep_for(
            std::chrono::duration<double>(scan_wait_));  // let robot settle
          run_detection(idx);
          detecting_ = false;
        }).detach();
      }
      return;
    }

    // ── Heading / drive ───────────────────────────────────────────────────────
    const double desired_yaw = std::atan2(dy, dx);
    const double err = std::atan2(
      std::sin(desired_yaw - ryaw),
      std::cos(desired_yaw - ryaw));

    if (std::abs(err) > align_th_) {
      state_        = State::ALIGNING;
      cmd.angular.z = turn_ * (err > 0.0 ? 1.0 : -1.0);
    } else {
      state_         = State::DRIVING;
      cmd.linear.x   = fwd_ * std::min(1.0, dist / 1.0);
      cmd.angular.z  = 0.5 * err;
    }
    cmd_pub_->publish(cmd);
  }

  // ── Detection worker — runs in a detached thread ───────────────────────────
  //
  // Calls /detect_object once per target class and stores results in records_.
  // The future.wait_for() blocks this thread; the MultiThreadedExecutor handles
  // the service response on another thread without deadlocking.

  void run_detection(std::size_t wp_idx)
  {
    if (!detect_client_->wait_for_service(3s)) {
      RCLCPP_WARN(get_logger(),
        "WP[%zu] /detect_object not available — skipping", wp_idx);
      return;
    }

    auto & rec = records_[wp_idx];
    rec.results.clear();

    for (const auto & cls : target_classes_) {
      auto req = std::make_shared<DetectObject::Request>();
      req->target_class = cls;

      auto future = detect_client_->async_send_request(req);

      if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_WARN(get_logger(),
          "WP[%zu] detect '%s' timed out", wp_idx, cls.c_str());
        rec.results.push_back({cls, false, 0.f, 0.0, 0.0});
        continue;
      }

      const auto res = future.get();
      DetResult dr;
      dr.target_class = cls;
      dr.found        = res->found;
      dr.confidence   = res->confidence;
      dr.obj_x        = res->pose.pose.position.x;
      dr.obj_y        = res->pose.pose.position.y;
      rec.results.push_back(dr);

      if (dr.found) {
        RCLCPP_INFO(get_logger(),
          "WP[%zu] DETECTED '%s'  conf=%.2f  pos=(%.2f, %.2f)",
          wp_idx, cls.c_str(), dr.confidence, dr.obj_x, dr.obj_y);
      } else {
        RCLCPP_INFO(get_logger(),
          "WP[%zu] not found: '%s'", wp_idx, cls.c_str());
      }
    }
  }

  // ── Advance to next waypoint ───────────────────────────────────────────────

  void advance_waypoint()
  {
    const std::size_t next = current_wp_ + 1;
    if (next >= waypoints_.size()) {
      print_checklist();
      if (loop_) {
        for (auto & r : records_) r.results.clear();   // reset for next loop
        current_wp_ = 0;
        RCLCPP_INFO(get_logger(), "Patrol loop complete — restarting from WP[0]");
      } else {
        ctrl_timer_->cancel();
        return;
      }
    } else {
      current_wp_ = next;
    }
    RCLCPP_INFO(get_logger(), "→ WP[%zu/%zu]  (%.2f, %.2f)",
      current_wp_, waypoints_.size() - 1,
      waypoints_[current_wp_].x, waypoints_[current_wp_].y);
    state_ = State::ALIGNING;
  }

  // ── End-of-patrol checklist ────────────────────────────────────────────────

  void print_checklist()
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "\n╔══════════════════════════════════════════════════════╗\n";
    ss << "║            INSPECTION PATROL  —  CHECKLIST          ║\n";
    ss << "╠══════════════════════════════════════════════════════╣\n";

    int total = 0, hits = 0;
    for (const auto & rec : records_) {
      ss << "║  WP[" << rec.idx << "]  (" << rec.x << ", " << rec.y << ")\n";
      if (rec.results.empty()) {
        ss << "║    (no detection performed)\n";
        continue;
      }
      for (const auto & dr : rec.results) {
        ++total;
        if (dr.found) {
          ++hits;
          ss << "║    [+] " << dr.target_class
             << "  conf=" << dr.confidence
             << "  pos=(" << dr.obj_x << ", " << dr.obj_y << ")\n";
        } else {
          ss << "║    [ ] " << dr.target_class << "\n";
        }
      }
    }

    ss << "╠══════════════════════════════════════════════════════╣\n";
    ss << "║  " << hits << " / " << total << " checks positive\n";
    ss << "╚══════════════════════════════════════════════════════╝";

    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }

  // ── RViz markers ──────────────────────────────────────────────────────────
  //
  // Cylinder colours:
  //   blue   — not yet visited
  //   yellow — active navigation target
  //   red    — stopped here, detection in progress
  //   green  — visited, at least one target detected
  //   grey   — visited, nothing detected

  void publish_markers()
  {
    if (waypoints_.empty()) { return; }

    visualization_msgs::msg::MarkerArray arr;
    const auto now = get_clock()->now();

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    // ── Closed route line ─────────────────────────────────────────────────────
    visualization_msgs::msg::Marker path;
    path.header.frame_id    = frame_id_;
    path.header.stamp       = now;
    path.ns                 = "patrol_path";
    path.id                 = 0;
    path.type               = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action             = visualization_msgs::msg::Marker::ADD;
    path.scale.x            = 0.04;
    path.color.b            = 1.0;
    path.color.a            = 0.5;
    path.pose.orientation.w = 1.0;
    for (const auto & wp : waypoints_) {
      geometry_msgs::msg::Point p;
      p.x = wp.x; p.y = wp.y; p.z = 0.05;
      path.points.push_back(p);
    }
    {
      geometry_msgs::msg::Point p;
      p.x = waypoints_[0].x; p.y = waypoints_[0].y; p.z = 0.05;
      path.points.push_back(p);
    }
    arr.markers.push_back(path);

    // ── Per-waypoint cylinders + labels ───────────────────────────────────────
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      bool checked = !records_[i].results.empty();
      bool hit     = false;
      for (const auto & dr : records_[i].results) {
        if (dr.found) { hit = true; break; }
      }

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

      if (i == current_wp_ && detecting_) {
        // Actively scanning — red
        mk.scale.x = mk.scale.y = 0.35; mk.scale.z = 0.60;
        mk.color.r = 1.0; mk.color.a = 1.0;
      } else if (i == current_wp_ && !waiting_) {
        // Driving toward — yellow
        mk.scale.x = mk.scale.y = 0.35; mk.scale.z = 0.60;
        mk.color.r = 1.0; mk.color.g = 1.0; mk.color.a = 1.0;
      } else if (hit) {
        // Detection found — green
        mk.scale.x = mk.scale.y = 0.35; mk.scale.z = 0.60;
        mk.color.g = 1.0; mk.color.a = 1.0;
      } else if (checked) {
        // Visited, nothing found — grey
        mk.scale.x = mk.scale.y = 0.20; mk.scale.z = 0.30;
        mk.color.r = 0.6; mk.color.g = 0.6; mk.color.b = 0.6; mk.color.a = 0.8;
      } else {
        // Not yet visited — blue
        mk.scale.x = mk.scale.y = 0.20; mk.scale.z = 0.30;
        mk.color.b = 1.0; mk.color.a = 0.8;
      }
      arr.markers.push_back(mk);

      // Index label — append [+] when a detection was found here
      visualization_msgs::msg::Marker txt;
      txt.header.frame_id    = frame_id_;
      txt.header.stamp       = now;
      txt.ns                 = "patrol_labels";
      txt.id                 = static_cast<int>(i);
      txt.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action             = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x    = waypoints_[i].x;
      txt.pose.position.y    = waypoints_[i].y;
      txt.pose.position.z    = 0.75;
      txt.pose.orientation.w = 1.0;
      txt.scale.z            = 0.25;
      txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 1.0;
      txt.text = std::to_string(i) + (hit ? " [+]" : "");
      arr.markers.push_back(txt);
    }

    mk_pub_->publish(arr);
  }
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolV2Node>();
  // MultiThreadedExecutor is required: detection thread calls async_send_request
  // and waits on its future while the executor handles the response callback.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
