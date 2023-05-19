#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class FormerSafetyControl : public rclcpp::Node
{
public:
  FormerSafetyControl()
      : Node("former_safety_control"), safety_distance_(0.5f), safety_l_detected_(false), safety_r_detected_(false)
  {
    this->declare_parameter<double>("safety_distance", 0.3);
    this->declare_parameter<double>("rate", 10);

    get_parameter("safety_distance", safety_distance_);

    const auto rate = this->get_parameter("rate").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);

    sub_l_sonar_range_ = this->create_subscription<sensor_msgs::msg::Range>(
        "l_sonar_range", rclcpp::SensorDataQoS(), std::bind(&FormerSafetyControl::sonar_l_callback, this, std::placeholders::_1));

    sub_r_sonar_range_ = this->create_subscription<sensor_msgs::msg::Range>(
        "r_sonar_range", rclcpp::SensorDataQoS(), std::bind(&FormerSafetyControl::sonar_r_callback, this, std::placeholders::_1));

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(period, std::bind(&FormerSafetyControl::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "safety_distance(%f) rate(%f)", safety_distance_, rate);
    RCLCPP_INFO(this->get_logger(), "Initialized...");
  }

  ~FormerSafetyControl() {}

private:
  void timer_callback()
  {
    if (safety_l_detected_ || safety_r_detected_)
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      pub_cmd_vel_->publish(cmd_vel);
    }
  }

  void sonar_l_callback(const sensor_msgs::msg::Range &msg)
  {
    if (msg.range >= msg.min_range && msg.range <= msg.max_range)
    {
      // RCLCPP_INFO(this->get_logger(), "left sonar range(%f)", msg.range);
      safety_l_detected_ = (msg.range < safety_distance_) ? true : false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "wrong range in sonar L -> range(%f) min(%f) max(%f)", msg.range, msg.min_range, msg.max_range);
      safety_l_detected_ = true;
    }
  }

  void sonar_r_callback(const sensor_msgs::msg::Range &msg)
  {
    if (msg.range >= msg.min_range && msg.range <= msg.max_range)
    {
      // RCLCPP_INFO(this->get_logger(), "right sonar range(%f)", msg.range);
      safety_r_detected_ = (msg.range < safety_distance_) ? true : false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "wrong range in sonar R -> range(%f) min(%f) max(%f)", msg.range, msg.min_range, msg.max_range);
      safety_r_detected_ = true;
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_l_sonar_range_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_r_sonar_range_;

  double safety_distance_;

  bool safety_l_detected_;
  bool safety_r_detected_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormerSafetyControl>());
  rclcpp::shutdown();
  return 0;
}