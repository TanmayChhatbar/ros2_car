#pragma once

class Vehicle2D_STData
{
public:
    Vehicle2D_STData();

    double getSteeringAngle() const;
    void setSteeringAngle(const double steering_angle_);
    double getMotorTorque() const;
    void setMotorTorque(const double motor_torque_);
    void getBrakeTorque(double (&brake_torque_)[2]) const;
    void setBrakeTorque(const double brake_torque_[2]);

    void getPosition(double &x_, double &y_) const;
    void setPosition(const double x_, const double y_);
    void getOrientation(double &yaw_) const;
    void setOrientation(const double yaw_);

    void getLinearVelocities(double &vx_, double &vy_) const;
    void setLinearVelocities(const double vx_, const double vy_);
    void getAngularVelocities(double &w_yaw_) const;
    void setAngularVelocities(const double w_yaw_);

    void getWheelVelocities(double &w_wheel_front_, double &w_wheel_rear_) const;
    void setWheelVelocities(const double w_wheel_front_, const double w_wheel_rear_);
    void getWheelVelocities(double (&w_wheel_)[2]) const;
    void setWheelVelocities(const double w_wheel_[2]);
    void getWheelRotations(double (&p_wheel_)[2]) const;
    void setWheelRotations(const double p_wheel_[2]);

    void getLinearAccelerations(double &ax_, double &ay_) const;
    void setLinearAccelerations(const double ax_, const double ay_);
    void getAngularAccelerations(double &a_yaw_) const;
    void setAngularAccelerations(const double a_yaw_);
    void getWheelAccelerations(double &a_front_, double &a_rear_) const;
    void setWheelAccelerations(const double a_front_, const double a_rear_);
    void getWheelAccelerations(double (&a_wheel_)[2]) const;
    void setWheelAccelerations(const double a_wheel_[2]);

    void getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const;
    void setBodyForcesAndMoments(const double Fx_, const double Fy_, const double Mz_);
    void getWheelNormalLoads(double (&Fz_wheel_)[2]) const;
    void setWheelNormalLoads(const double Fz_wheel_[2]);
    void getWheelForces(double (&Fx_wheel_)[2], double (&Fy_wheel_)[2]) const;
    void setWheelForces(const double Fx_wheel_[2], const double Fy_wheel_[2]);
    void getWheelTorques(double (&wheel_torques_)[2]) const;
    void setWheelTorques(const double (&wheel_torques_)[2]);

private:
    double X, Y, yaw;     // global 2D position and orientation
    double vx, vy, w_yaw; // vehicle frame velocities
    double ax, ay, a_yaw; // vehicle frame accelerations

    double p_wheel[2];
    double w_wheel[2];
    double a_wheel[2];

    double Fx, Fy, Mz;
    double Fx_wheel[2];
    double Fy_wheel[2];
    double Fz_wheel[2];

    double steering_angle;
    double motor_torque;
    double brake_torque[2];
    double wheel_torques[2];

    friend class Vehicle2D_ST;
};
