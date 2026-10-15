// Safety layer of the simulated chassis.
//
// The planner publishes /cmd_vel. This node forwards it to /cmd_vel_safe, which
// is what the Gazebo differential drive plugin subscribes to. The command is
// zeroed (the robot stops) whenever
//   * no /cmd_vel was received for longer than `cmd_timeout`  (planner died /
//     stopped publishing -> data timeout),
//   * no /state_estimation was received for longer than `odom_timeout`
//     (state estimate lost -> the robot must not continue driving blind),
//   * the command contains NaN/Inf (solver failure).
// Commands are additionally clamped to the configured limits.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ego_ground_sim
{

class CmdVelSafety : public rclcpp::Node
{
public:
  CmdVelSafety()
  : rclcpp::Node("cmd_vel_safety")
  {
    cmd_timeout_ = this->declare_parameter<double>("cmd_timeout", 0.5);
    odom_timeout_ = this->declare_parameter<double>("odom_timeout", 1.0);
    max_linear_ = this->declare_parameter<double>("max_linear", 2.0);
    max_angular_ = this->declare_parameter<double>("max_angular", 2.5);
    const double rate = this->declare_parameter<double>("publish_rate", 50.0);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&CmdVelSafety::cmdCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10, std::bind(&CmdVelSafety::odomCallback, this, std::placeholders::_1));

    last_cmd_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_odom_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    // The node clock is used here on purpose: the "age" of the data has to be
    // measured on the same clock that timestamps the simulation.
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
      std::bind(&CmdVelSafety::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(), "watchdog: cmd timeout %.2fs, odom timeout %.2fs, limits v=%.2f w=%.2f",
      cmd_timeout_, odom_timeout_, max_linear_, max_angular_);
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    cmd_ = *msg;
    last_cmd_ = this->now();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/) {last_odom_ = this->now();}

  void timerCallback()
  {
    const rclcpp::Time now = this->now();
    geometry_msgs::msg::Twist out;

    const double cmd_age = (now - last_cmd_).seconds();
    const double odom_age = (now - last_odom_).seconds();
    const bool cmd_fresh = last_cmd_.nanoseconds() > 0 && cmd_age <= cmd_timeout_;
    const bool odom_fresh = last_odom_.nanoseconds() > 0 && odom_age <= odom_timeout_;
    const bool cmd_valid = std::isfinite(cmd_.linear.x) && std::isfinite(cmd_.angular.z);

    if (!cmd_fresh) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "no /cmd_vel for %.2fs -> stopping", cmd_age);
    } else if (!odom_fresh) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "no /state_estimation for %.2fs -> stopping", odom_age);
    } else if (!cmd_valid) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "invalid (NaN/Inf) velocity command -> stopping");
    } else {
      out.linear.x = std::clamp(cmd_.linear.x, -max_linear_, max_linear_);
      out.angular.z = std::clamp(cmd_.angular.z, -max_angular_, max_angular_);
    }

    cmd_pub_->publish(out);
  }

  double cmd_timeout_, odom_timeout_, max_linear_, max_angular_;
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time last_cmd_, last_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ego_ground_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ego_ground_sim::CmdVelSafety>());
  rclcpp::shutdown();
  return 0;
}
