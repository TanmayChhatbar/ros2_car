#include "Vehicle2D_ST.hpp"

Vehicle2D_STData::Vehicle2D_STData()
    : X(0.0), Y(0.0), yaw(0.0),
      vx(0.0), vy(0.0), w_yaw(0.0),
      ax(0.0), ay(0.0), a_yaw(0.0),
      p_wheel{0.0, 0.0},
      w_wheel{0.0, 0.0},
      a_wheel{0.0, 0.0},
      Fx(0.0), Fy(0.0), Mz(0.0),
      Fx_wheel{0.0, 0.0},
      Fy_wheel{0.0, 0.0},
      Fz_wheel{0.0, 0.0},
      steering_angle(0.0), motor_torque(0.0),
      wheel_torques{0.0, 0.0} {}

// getters and setters for inputs
double Vehicle2D_STData::getSteeringAngle() const { return steering_angle; }
void Vehicle2D_STData::setSteeringAngle(const double steering_angle_) { steering_angle = steering_angle_; }
double Vehicle2D_STData::getMotorTorque() const { return motor_torque; }
void Vehicle2D_STData::setMotorTorque(const double motor_torque_) { motor_torque = motor_torque_; }
void Vehicle2D_STData::getBrakeTorque(double (&brake_torque_)[2]) const
{
    brake_torque_[0] = brake_torque[0];
    brake_torque_[1] = brake_torque[1];
}
void Vehicle2D_STData::setBrakeTorque(const double brake_torque_[2])
{
    brake_torque[0] = brake_torque_[0];
    brake_torque[1] = brake_torque_[1];
}

// getters and setters for position and orientation
void Vehicle2D_STData::getPosition(double &x_, double &y_) const
{
    x_ = X;
    y_ = Y;
}
void Vehicle2D_STData::setPosition(const double x_, const double y_)
{
    X = x_;
    Y = y_;
}
void Vehicle2D_STData::getOrientation(double &yaw_) const { yaw_ = yaw; }
void Vehicle2D_STData::setOrientation(const double yaw_) { yaw = yaw_; }

// getters and setters for velocities
void Vehicle2D_STData::getLinearVelocities(double &vx_, double &vy_) const
{
    vx_ = vx;
    vy_ = vy;
}
void Vehicle2D_STData::setLinearVelocities(const double vx_, const double vy_)
{
    vx = vx_;
    vy = vy_;
}
void Vehicle2D_STData::getAngularVelocities(double &w_yaw_) const { w_yaw_ = w_yaw; }
void Vehicle2D_STData::setAngularVelocities(const double w_yaw_) { w_yaw = w_yaw_; }

void Vehicle2D_STData::getWheelVelocities(double &w_wheel_front_, double &w_wheel_rear_) const
{
    w_wheel_front_ = w_wheel[0];
    w_wheel_rear_ = w_wheel[1];
}
void Vehicle2D_STData::setWheelVelocities(const double w_wheel_front_, const double w_wheel_rear_)
{
    w_wheel[0] = w_wheel_front_;
    w_wheel[1] = w_wheel_rear_;
}
void Vehicle2D_STData::getWheelVelocities(double (&w_wheel_)[2]) const
{
    w_wheel_[0] = w_wheel[0];
    w_wheel_[1] = w_wheel[1];
}
void Vehicle2D_STData::setWheelVelocities(const double w_wheel_[2])
{
    w_wheel[0] = w_wheel_[0];
    w_wheel[1] = w_wheel_[1];
}
void Vehicle2D_STData::getWheelRotations(double (&p_wheel_)[2]) const
{
    p_wheel_[0] = p_wheel[0];
    p_wheel_[1] = p_wheel[1];
}
void Vehicle2D_STData::setWheelRotations(const double p_wheel_[2])
{
    p_wheel[0] = p_wheel_[0];
    p_wheel[1] = p_wheel_[1];
}

// getters and setters for accelerations
void Vehicle2D_STData::getLinearAccelerations(double &ax_, double &ay_) const
{
    ax_ = ax;
    ay_ = ay;
}
void Vehicle2D_STData::setLinearAccelerations(const double ax_, const double ay_)
{
    ax = ax_;
    ay = ay_;
}
void Vehicle2D_STData::getAngularAccelerations(double &a_yaw_) const { a_yaw_ = a_yaw; }
void Vehicle2D_STData::setAngularAccelerations(const double a_yaw_) { a_yaw = a_yaw_; }
void Vehicle2D_STData::getWheelAccelerations(double &a_front_, double &a_rear_) const
{
    a_front_ = a_wheel[0];
    a_rear_ = a_wheel[1];
}
void Vehicle2D_STData::setWheelAccelerations(const double a_front_, const double a_rear_)
{
    a_wheel[0] = a_front_;
    a_wheel[1] = a_rear_;
}
void Vehicle2D_STData::getWheelAccelerations(double (&a_wheel_)[2]) const
{
    a_wheel_[0] = a_wheel[0];
    a_wheel_[1] = a_wheel[1];
}
void Vehicle2D_STData::setWheelAccelerations(const double a_wheel_[2])
{
    a_wheel[0] = a_wheel_[0];
    a_wheel[1] = a_wheel_[1];
}

// getters and setters for forces and moments
void Vehicle2D_STData::getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const
{
    Fx_ = Fx;
    Fy_ = Fy;
    Mz_ = Mz;
}
void Vehicle2D_STData::setBodyForcesAndMoments(const double Fx_, const double Fy_, const double Mz_)
{
    Fx = Fx_;
    Fy = Fy_;
    Mz = Mz_;
}
void Vehicle2D_STData::getWheelNormalLoads(double (&Fz_wheel_)[2]) const
{
    Fz_wheel_[0] = Fz_wheel[0];
    Fz_wheel_[1] = Fz_wheel[1];
}
void Vehicle2D_STData::setWheelNormalLoads(const double Fz_wheel_[2])
{
    Fz_wheel[0] = Fz_wheel_[0];
    Fz_wheel[1] = Fz_wheel_[1];
}
void Vehicle2D_STData::getWheelForces(double (&Fx_wheel_)[2], double (&Fy_wheel_)[2]) const
{
    Fx_wheel_[0] = Fx_wheel[0];
    Fx_wheel_[1] = Fx_wheel[1];
    Fy_wheel_[0] = Fy_wheel[0];
    Fy_wheel_[1] = Fy_wheel[1];
}
void Vehicle2D_STData::setWheelForces(const double Fx_wheel_[2], const double Fy_wheel_[2])
{
    Fx_wheel[0] = Fx_wheel_[0];
    Fx_wheel[1] = Fx_wheel_[1];
    Fy_wheel[0] = Fy_wheel_[0];
    Fy_wheel[1] = Fy_wheel_[1];
}
void Vehicle2D_STData::getWheelTorques(double (&wheel_torques_)[2]) const
{
    wheel_torques_[0] = wheel_torques[0];
    wheel_torques_[1] = wheel_torques[1];
}
void Vehicle2D_STData::setWheelTorques(const double (&wheel_torques_)[2])
{
    wheel_torques[0] = wheel_torques_[0];
    wheel_torques[1] = wheel_torques_[1];
}
