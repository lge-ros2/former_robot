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
      : Node("former_safety_control")
      , safety_distance_(0.5f)
      , max_distance_(5.0f)
      , safety_l_detected_(false)
      , safety_r_detected_(false)
  {
    this->declare_parameter<double>("safety_distance", 0.5);
    this->declare_parameter<double>("max_distance", 5.0);

    get_parameter("safety_distance", safety_distance_);
    get_parameter("max_distance", max_distance_);

    sub_l_sonar_range_ = this->create_subscription<sensor_msgs::msg::Range>(
        "l_sonar_range", rclcpp::SensorDataQoS(), std::bind(&FormerSafetyControl::sonar_l_callback, this, std::placeholders::_1));

    sub_r_sonar_range_ = this->create_subscription<sensor_msgs::msg::Range>(
        "r_sonar_range", rclcpp::SensorDataQoS(), std::bind(&FormerSafetyControl::sonar_r_callback, this, std::placeholders::_1));

    sub_cmd_vel_  = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", rclcpp::QoS(1), std::bind(&FormerSafetyControl::cmd_vel_callback, this, std::placeholders::_1));

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 1);

    RCLCPP_INFO(this->get_logger(), "safety_distance(%f)", safety_distance_);
    RCLCPP_INFO(this->get_logger(), "max_distance(%f)", max_distance_);
    RCLCPP_INFO(this->get_logger(), "Initialized...");
  }

  ~FormerSafetyControl() {}

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::Twist cmd_vel;


    int current_linear_direction = 0;

    if (msg->linear.x > 0)
      current_linear_direction = 1;
    else if (msg->linear.x < 0)
      current_linear_direction = -1;
    else
      current_linear_direction = 0;

    if ((safety_l_detected_ || safety_r_detected_) &&
        (last_linear_direction_ == 0 || current_linear_direction == last_linear_direction_))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;

      if (current_linear_direction != 0)
        last_linear_direction_ = current_linear_direction;
    }
    else
    {
      cmd_vel.linear.x = msg->linear.x;
      cmd_vel.angular.z = msg->angular.z;
    }


    // RCLCPP_INFO(this->get_logger(), "last_linear_direction_(%d) current_linear_direction_(%d)", last_linear_direction_,  current_linear_direction);

    pub_cmd_vel_->publish(cmd_vel);
  }

  void sonar_l_callback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    if (msg->range > 0)
    {
      if (msg->range >= msg->min_range && msg->range <= max_distance_)
      {
        // RCLCPP_INFO(this->get_logger(), "left sonar range(%f)", msg->range);
        const auto safety_detect = (msg->range < safety_distance_) ? true : false;
        if (safety_detect && safety_r_detected_ == false)
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());

        safety_l_detected_ = safety_detect;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "wrong range in sonar L -> range(%f) min(%f) max(%f)", msg->range, msg->min_range, max_distance_);
        safety_l_detected_ = true;
      }
    }
  }

  void sonar_r_callback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    if (msg->range > 0)
    {
      if (msg->range >= msg->min_range && msg->range <= max_distance_)
      {
        // RCLCPP_INFO(this->get_logger(), "right sonar range(%f)", msg->range);
        const auto safety_detect = (msg->range < safety_distance_) ? true : false;
        if (safety_detect && safety_r_detected_ == false)
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());

        safety_r_detected_ = safety_detect;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "wrong range in sonar R -> range(%f) min(%f) max(%f)", msg->range, msg->min_range, max_distance_);
        safety_r_detected_ = true;
      }
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_l_sonar_range_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_r_sonar_range_;

  float safety_distance_;
  float max_distance_;

  int last_linear_direction_; // 1 forward, -1 backward

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