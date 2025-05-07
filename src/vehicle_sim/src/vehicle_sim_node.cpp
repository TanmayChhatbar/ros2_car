#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "Vehicle2D.hpp" // include the Vehicle2D header

class Vehicle2DSim : public rclcpp::Node
{
public:
  Vehicle2DSim()
      : Node("vehicle_2d_sim"), vehicle_() // initialize vehicle instance
  {
    // joy subscription for control inputs
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&Vehicle2DSim::joy_callback, this, std::placeholders::_1));

    // timer for simulation steps
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&Vehicle2DSim::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Vehicle simulator started - listening to joystick inputs");
    RCLCPP_INFO(this->get_logger(), "Running simulation at %.1f Hz. Input frequency may be lower", 1.0 / dt);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg)
  {
    steering_input_ = msg->axes.size() > 0 ? msg->axes[0] : 0.0;
    throttle_input_ = msg->axes.size() > 4 ? (- msg->axes[4] + 1) / 2 : 0.0;
    brake_input_    = msg->axes.size() > 5 ? (- msg->axes[5] + 1) / 2 : 0.0;
    handbrake_input_ = msg->buttons.size() > 0 ? msg->buttons[0] : 0.0;

    throttle_input_ = throttle_input_ - brake_input_; // TODO not using brakes yet
    brake_input_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Joystick axes: steering=%.2f, throttle=%.2f, brake=%.2f, handbrake=%.2f",
                steering_input_, throttle_input_, brake_input_, handbrake_input_);
  }

  void timer_callback()
  {
    // get current ROS time
    rclcpp::Time current_time = this->now();

    // step the simulation, passing the time delta if needed
    static rclcpp::Time last_time = current_time;

    uint16_t n_steps = (uint16_t)((current_time - last_time).seconds() / dt);
    last_time = current_time;

    // call the stepSimulation function to catch up with ROS time
    for (uint16_t i = 0; i < n_steps; ++i)
    {
      vehicle_.stepSimulation(dt, steering_input_, throttle_input_, brake_input_);
    }
  }

  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // vehicle simulation
  double steering_input_ = 0.0;
  double throttle_input_ = 0.0;
  double brake_input_ = 0.0;
  double handbrake_input_ = 0.0;
  double dt = 0.001; // simulation time step
  Vehicle2D vehicle_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle2DSim>());
  rclcpp::shutdown();
  return 0;
}
