#include <memory>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "Vehicle2D.hpp" // include the Vehicle2D header

class Vehicle2DSim : public rclcpp::Node
{
public:
  Vehicle2DSim()
      : Node("vehicle_2d_sim"), vehicle_() // initialize vehicle instance
  {
    // publisher for vehicle odometry
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom", 10);
    
    // joy subscription for control inputs
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&Vehicle2DSim::joy_callback, this, std::placeholders::_1));

    // timer for simulation steps
    timer_sim_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&Vehicle2DSim::timer_simulation, this));

    // timer for publishing odometry
    timer_pub_ = this->create_wall_timer(
        std::chrono::duration<double>(0.025),
        std::bind(&Vehicle2DSim::publishOdometry, this));

    // Declare and set the "frame_id" parameter
    this->declare_parameter<std::string>("frame_id", "odom");

    RCLCPP_INFO(this->get_logger(), "Vehicle simulator started - listening to joystick inputs");
    RCLCPP_INFO(this->get_logger(), "Running simulation at %.1f Hz. Input frequency may be lower", 1.0 / dt);
  }

private:

  void publishOdometry()
  {
    // publish odom pose
    double X, Y, yaw;
    vehicle_.getVehicle2DData().getPosition(X, Y);
    vehicle_.getVehicle2DData().getOrientation(yaw);
    geometry_msgs::msg::PoseStamped pose;

    // header
    pose.header.stamp = this->now();
    pose.header.frame_id = this->get_parameter("frame_id").as_string();
    
    // position
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = 0.0;
    
    // rot to quat
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    this->publisher_->publish(pose);
  }

  void joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg)
  {
    // get inputs
    steering_input_ = msg->axes.size() > 0 ? msg->axes[0] : 0.0;
    throttle_input_ = msg->axes.size() > 4 ? (- msg->axes[4] + 1) / 2 : 0.0;
    brake_input_    = msg->axes.size() > 5 ? (- msg->axes[5] + 1) / 2 : 0.0;
    handbrake_input_ = msg->buttons.size() > 0 ? msg->buttons[0] : 0.0;

    // TODO combine throttle and brake inputs, because not using brakes yet
    throttle_input_ = throttle_input_ - brake_input_; 
    brake_input_ = 0.0;
  }

  void timer_simulation()
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_sim_;
  rclcpp::TimerBase::SharedPtr timer_pub_;

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
