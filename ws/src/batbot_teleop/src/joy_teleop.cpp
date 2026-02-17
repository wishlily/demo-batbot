#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

static const char * help_msg = R"(
🎮 Joy Teleop Help
==================
Control your robot with a xbox joystick:

Movement:
- Left stick : Move the robot (forward/backward)
- Right stick: Move the robot (left/right)

Functions:
- A button: Emergency stop
)";

class JoyTeleop : public rclcpp::Node
{
public:
  JoyTeleop() : Node("joy_teleop"), linear_gear_(1.0 / 3), angular_gear_(1.0 / 4)
  {
    // Declare parameters
    this->declare_parameter("linear_speed_limit", 1.0);
    this->declare_parameter("angular_speed_limit", 5.0);
    this->declare_parameter("linear_accel_limit", 0.8);
    this->declare_parameter("angular_accel_limit", 2.0);
    this->declare_parameter("axis_deadzone", 0.2);
    this->declare_parameter("cmd_publish_rate", 30.0);

    linear_limit_ = this->get_parameter("linear_speed_limit").as_double();
    angular_limit_ = this->get_parameter("angular_speed_limit").as_double();
    linear_accel_limit_ = this->get_parameter("linear_accel_limit").as_double();
    angular_accel_limit_ = this->get_parameter("angular_accel_limit").as_double();
    axis_deadzone_ = this->get_parameter("axis_deadzone").as_double();
    cmd_publish_rate_ = this->get_parameter("cmd_publish_rate").as_double();
    if (cmd_publish_rate_ <= 0.0) {
      cmd_publish_rate_ = 30.0;
    }

    // Car Publishers
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Joystick Subscriber
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyTeleop::joyCallback, this, std::placeholders::_1));
    cmd_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / cmd_publish_rate_),
      std::bind(&JoyTeleop::publishSmoothedCmd, this));
    last_update_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), help_msg);
    RCLCPP_INFO(this->get_logger(), "🎮 Joy Teleop Node Started.");
  }

private:
  inline double now_sec() { return this->get_clock()->now().seconds(); }

  enum class joyButton {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    BACK = 6,
    START = 7,
    XBOX = 8,
    BUTTON_COUNT
  };

  struct ButtonDebounce
  {
    std::array<double, static_cast<int>(joyButton::BUTTON_COUNT)> last_time;
    std::array<bool, static_cast<int>(joyButton::BUTTON_COUNT)> last_state;
    double debounce_time = 0.3;  // 300 ms

    ButtonDebounce()
    {
      last_time.fill(0.0);
      last_state.fill(false);
    }
  };

  bool getButton(const sensor_msgs::msg::Joy::SharedPtr joy, joyButton button)
  {
    int no = static_cast<int>(button);
    bool state = joy->buttons[static_cast<int>(button)] == 1;
    bool last_state = button_debounce_.last_state[no];
    double current_time = now_sec();
    if (state != last_state) {
      button_debounce_.last_time[no] = current_time;
      button_debounce_.last_state[no] = state;
      return last_state;
    }
    if (current_time - button_debounce_.last_time[no] >= button_debounce_.debounce_time) {
      button_debounce_.last_time[no] = current_time;
      button_debounce_.last_state[no] = state;
      return state;
    }
    return last_state;
  }

  enum class joyAxis { LEFT_X = 0, LEFT_Y = 1, LEFT_T = 2, RIGHT_X = 3, RIGHT_Y = 4, RIGHT_T = 5 };

  inline double getJoyAxis(const sensor_msgs::msg::Joy::SharedPtr joy, joyAxis axis)
  {
    int idx = static_cast<int>(axis);
    if (idx >= static_cast<int>(joy->axes.size())) {
      return 0.0;
    }
    return joy->axes[idx];
  }

  double calcSpeed(
    const sensor_msgs::msg::Joy::SharedPtr joy, joyAxis axis, double gear, double limit)
  {
    double val = getJoyAxis(joy, axis);
    if (std::abs(val) < axis_deadzone_) {
      return 0.0;
    }
    val *= gear;
    if (val > limit) {
      return limit;
    }
    if (val < -limit) {
      return -limit;
    }
    return val;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    setSpeed(joy);
    setStop(joy);
  }

  void setSpeed(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    target_linear_speed_ = calcSpeed(joy, joyAxis::LEFT_Y, linear_gear_, linear_limit_);
    target_angular_speed_ = calcSpeed(joy, joyAxis::RIGHT_X, angular_gear_, angular_limit_);
  }

  double limitRate(double current, double target, double rate_limit, double dt)
  {
    if (rate_limit <= 0.0 || dt <= 0.0) {
      return target;
    }
    double max_step = rate_limit * dt;
    double delta = target - current;
    if (delta > max_step) {
      return current + max_step;
    }
    if (delta < -max_step) {
      return current - max_step;
    }
    return target;
  }

  void publishSmoothedCmd()
  {
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    current_linear_speed_ = limitRate(
      current_linear_speed_, target_linear_speed_, linear_accel_limit_, dt);
    current_angular_speed_ = limitRate(
      current_angular_speed_, target_angular_speed_, angular_accel_limit_, dt);

    if (std::abs(current_linear_speed_ - last_published_linear_speed_) < 1e-3 &&
      std::abs(current_angular_speed_ - last_published_angular_speed_) < 1e-3)
    {
      return;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = current_linear_speed_;
    twist.angular.z = current_angular_speed_;
    pub_cmd_vel_->publish(twist);
    last_published_linear_speed_ = current_linear_speed_;
    last_published_angular_speed_ = current_angular_speed_;
  }

  void setStop(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    if (!getButton(joy, joyButton::A)) {
      return;
    }
    target_linear_speed_ = 0.0;
    target_angular_speed_ = 0.0;
    current_linear_speed_ = 0.0;
    current_angular_speed_ = 0.0;

    geometry_msgs::msg::Twist twist;
    pub_cmd_vel_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "🛑 Emergency Stop");
  }

  // -------- ROS objects --------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  // -------- State variables --------
  double linear_limit_, angular_limit_;
  double linear_accel_limit_, angular_accel_limit_;
  double axis_deadzone_, cmd_publish_rate_;
  double linear_gear_, angular_gear_;
  double target_linear_speed_ = 0.0;
  double target_angular_speed_ = 0.0;
  double current_linear_speed_ = 0.0;
  double current_angular_speed_ = 0.0;
  double last_published_linear_speed_ = 0.0;
  double last_published_angular_speed_ = 0.0;
  rclcpp::Time last_update_time_;
  ButtonDebounce button_debounce_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
