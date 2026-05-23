#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <limits>

class ExploreNode : public rclcpp::Node
{
public:
  ExploreNode() : Node("explore")
  {
    declare_parameter("forward_speed",  0.2);   // m/s
    declare_parameter("turn_speed",     0.6);   // rad/s
    declare_parameter("safety_dist",    0.6);   // m  – stop if closer
    declare_parameter("front_half_deg", 30.0);  // half-width of "front" sector

    forward_speed_  = get_parameter("forward_speed").as_double();
    turn_speed_     = get_parameter("turn_speed").as_double();
    safety_dist_    = get_parameter("safety_dist").as_double();
    front_half_rad_ = get_parameter("front_half_deg").as_double() * M_PI / 180.0;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ExploreNode::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Explore node ready  fwd=%.2f m/s  turn=%.2f rad/s  safety=%.2f m",
      forward_speed_, turn_speed_, safety_dist_);
  }

private:
  enum class State { FORWARD, TURN_LEFT, TURN_RIGHT };
  State state_ = State::FORWARD;

  double forward_speed_;
  double turn_speed_;
  double safety_dist_;
  double front_half_rad_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Minimum valid range inside the angular window [lo, hi] (robot frame, rad).
  double sector_min(const sensor_msgs::msg::LaserScan & s, double lo, double hi) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      double a = s.angle_min + i * s.angle_increment;
      if (a < lo || a > hi) { continue; }
      float r = s.ranges[i];
      if (std::isfinite(r) && r >= s.range_min && r <= s.range_max) {
        if (r < best) { best = r; }
      }
    }
    return best;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double half = front_half_rad_;

    // Three sectors: front (±half), left (half … 90°), right (-90° … -half)
    double d_front = sector_min(*msg, -half,       half);
    double d_left  = sector_min(*msg,  half,        M_PI_2);
    double d_right = sector_min(*msg, -M_PI_2,     -half);

    geometry_msgs::msg::Twist cmd;

    if (d_front > safety_dist_) {
      // Path is clear – drive forward
      if (state_ != State::FORWARD) {
        RCLCPP_INFO(get_logger(), "→ FORWARD (front=%.2f m)", d_front);
        state_ = State::FORWARD;
      }
      cmd.linear.x  = forward_speed_;
      cmd.angular.z = 0.0;
    } else {
      // Obstacle ahead – turn toward the more open side
      bool turn_left = (d_left >= d_right);
      if (turn_left) {
        if (state_ != State::TURN_LEFT) {
          RCLCPP_INFO(get_logger(),
            "→ TURN LEFT (front=%.2f  L=%.2f  R=%.2f)", d_front, d_left, d_right);
          state_ = State::TURN_LEFT;
        }
        cmd.angular.z =  turn_speed_;
      } else {
        if (state_ != State::TURN_RIGHT) {
          RCLCPP_INFO(get_logger(),
            "→ TURN RIGHT (front=%.2f  L=%.2f  R=%.2f)", d_front, d_left, d_right);
          state_ = State::TURN_RIGHT;
        }
        cmd.angular.z = -turn_speed_;
      }
      cmd.linear.x = 0.0;
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExploreNode>());
  rclcpp::shutdown();
  return 0;
}
