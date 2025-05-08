#include <memory>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "Vehicle2D.hpp"

class Vehicle2DSim : public rclcpp::Node
{
public:
  Vehicle2DSim();

private:
  void publishOdometry();
  void joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg);
  void timer_simulation();

  // ROS-related
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_sim_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::Time last_time_;

  // vehicle simulation
  double steering_input_ = 0.0;
  double throttle_input_ = 0.0;
  double brake_input_ = 0.0;
  double handbrake_input_ = 0.0;
  double dt = 0.001; // simulation time step
  Vehicle2D vehicle_;
};

int main(int argc, char *argv[]);
