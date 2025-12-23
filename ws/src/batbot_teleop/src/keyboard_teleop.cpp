#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cctype>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

static const char * help_msg = R"(
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease linear speed by 10%
e/c : increase/decrease angular speed by 10%
t : toggle between x-axis and y-axis control
s : stop publishing toggles
CTRL+C to quit
)";

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop"),
    linear_speed_(0.2),
    angular_speed_(1.0),
    stop_mode_(false),
    axis_switch_(true)
  {
    this->declare_parameter("linear_speed_limit", 1.0);
    this->declare_parameter("angular_speed_limit", 5.0);

    linear_limit_ = this->get_parameter("linear_speed_limit").as_double();
    angular_limit_ = this->get_parameter("angular_speed_limit").as_double();

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    tcgetattr(STDIN_FILENO, &original_settings_);
    auto new_settings = original_settings_;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    RCLCPP_INFO(this->get_logger(), help_msg);

    timer_ = this->create_wall_timer(50ms, std::bind(&KeyboardTeleop::loop, this));
  }

  ~KeyboardTeleop() { tcsetattr(STDIN_FILENO, TCSANOW, &original_settings_); }

private:
  char getKey()
  {
    fd_set set;
    struct timeval timeout{0, 100000};
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
    if (rv > 0) {
      char c;
      read(STDIN_FILENO, &c, 1);
      return c;
    }
    return 0;
  }

  void loop()
  {
    char key = getKey();

    if (key == 0) return;
    key = std::tolower(key);

    if (key == 's') {
      stop_mode_ = !stop_mode_;
      RCLCPP_WARN(this->get_logger(), "Stop mode: %s", stop_mode_ ? "ON" : "OFF");
      return;
    }

    if (key == 't') {
      axis_switch_ = !axis_switch_;
      RCLCPP_INFO(
        this->get_logger(), "Axis switched: controlling %s now",
        axis_switch_ ? "X axis" : "Y axis");
      return;
    }

    if (speed_bindings_.count(key)) {
      linear_speed_ *= speed_bindings_.at(key).first;
      angular_speed_ *= speed_bindings_.at(key).second;

      linear_speed_ = std::min(linear_speed_, linear_limit_);
      angular_speed_ = std::min(angular_speed_, angular_limit_);

      RCLCPP_INFO(this->get_logger(), "Linear: %.2f, Angular: %.2f", linear_speed_, angular_speed_);
    }

    if (move_bindings_.count(key)) {
      x_ = move_bindings_.at(key).first;
      th_ = move_bindings_.at(key).second;
    }

    if (key == ' ') {
      x_ = 0;
      th_ = 0;
    }

    publish();
  }

  void publish()
  {
    geometry_msgs::msg::Twist msg;
    if (!stop_mode_) {
      msg.linear.x = axis_switch_ ? linear_speed_ * x_ : 0.0;
      msg.linear.y = axis_switch_ ? 0.0 : linear_speed_ * x_;
      msg.angular.z = angular_speed_ * th_;
    }
    pub_->publish(msg);
  }

  // Data
  double linear_speed_, angular_speed_, linear_limit_, angular_limit_;
  int x_ = 0, th_ = 0;
  bool stop_mode_, axis_switch_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  termios original_settings_;

  const std::unordered_map<char, std::pair<int, int>> move_bindings_{
    {'i', {1, 0}}, {'o', {1, -1}}, {'j', {0, 1}},  {'l', {0, -1}},
    {'u', {1, 1}}, {',', {-1, 0}}, {'.', {-1, 1}}, {'m', {-1, -1}}};

  const std::unordered_map<char, std::pair<double, double>> speed_bindings_{
    {'q', {1.1, 1.1}}, {'z', {.9, .9}},   {'w', {1.1, 1.0}},
    {'x', {.9, 1.0}},  {'e', {1.0, 1.1}}, {'c', {1.0, .9}}};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
