#include <chrono>
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

    linear_limit_ = this->get_parameter("linear_speed_limit").as_double();
    angular_limit_ = this->get_parameter("angular_speed_limit").as_double();

    // Car Publishers
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Joystick Subscriber
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyTeleop::joyCallback, this, std::placeholders::_1));

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
    return joy->axes[static_cast<int>(axis)];
  }

  double calcSpeed(
    const sensor_msgs::msg::Joy::SharedPtr joy, joyAxis axis, double gear, double limit)
  {
    double val = getJoyAxis(joy, axis);
    if (std::abs(val) < 0.2) {
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
    static double last_linear_speed;
    static double last_angular_speed;

    double linear_speed = calcSpeed(joy, joyAxis::LEFT_Y, linear_gear_, linear_limit_);
    double angular_speed = calcSpeed(joy, joyAxis::RIGHT_X, angular_gear_, angular_limit_);

    if (linear_speed == last_linear_speed && angular_speed == last_angular_speed) {
      return;
    }
    last_linear_speed = linear_speed;
    last_angular_speed = angular_speed;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;
    pub_cmd_vel_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "🏎️  Speed -> L: %.2f, A: %.2f", linear_speed, angular_speed);
  }

  void setStop(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    if (!getButton(joy, joyButton::A)) {
      return;
    }
    geometry_msgs::msg::Twist twist;
    pub_cmd_vel_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "🛑 Emergency Stop");
  }

  // -------- ROS objects --------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  // -------- State variables --------
  double linear_limit_, angular_limit_;
  double linear_gear_, angular_gear_;
  ButtonDebounce button_debounce_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
