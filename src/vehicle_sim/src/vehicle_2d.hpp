#pragma once

#include <cmath>    // for mathematical constants if needed
#include <iostream> // for printing


class VehicleInput
{
    public:
        // constructor
        VehicleInput()
            : steering_angle(0.0), motor_torque(0.0) {}

        // getters and setters for steering angle and motor torque
        void getSteeringAngle(double &steering_angle_) const;
        void setSteeringAngle(double steering_angle_);
        void getMotorTorque(double &motor_torque_) const;
        void setMotorTorque(double motor_torque_);
    private:
        double steering_angle; // [rad] steering angle
        double motor_torque;   // [Nm] motor torque

        friend class Vehicle; // allow Vehicle to access private members
};

// vehicle state data
class VehicleData
{
public:
    // constructor
    VehicleData()
        : X(0.0), Y(0.0), yaw(0.0),
          vx(0.0), vy(0.0), w_yaw(0.0),
          v_wheel{0.0, 0.0, 0.0, 0.0} {}

    // getters and setters for position and orientation
    void getPosition(double &x_, double &y_) const;
    void setPosition(double x_, double y_);
    void getOrientation(double &yaw_) const;
    void setOrientation(double yaw_);

    // getters and setters for velocities
    void getLinearVelocities(double &vx_, double &vy_) const;
    void setLinearVelocities(double vx_, double vy_);
    void getAngularVelocities(double &w_yaw_) const;
    void setAngularVelocities(double w_yaw_);

    void getWheelVelocities(double &fl_, double &fr_, double &rl_, double &rr_) const;
    void setWheelVelocities(double fl_, double fr_, double rl_, double rr_);
    void getWheelVelocities(double (&v_wheel_)[4]) const;
    void setWheelVelocities(const double v_wheel_[4]);

    // getters and setters for accelerations
    void getLinearAccelerations(double &vx_, double &vy_) const;
    void setLinearAccelerations(double vx_, double vy_);
    void getAngularAccelerations(double &w_yaw_) const;
    void setAngularAccelerations(double w_yaw_);
    void getWheelAccelerations(double &a_fl_, double &a_fr_, double &a_rl_, double &a_rr_) const;
    void setWheelAccelerations(double a_fl_, double a_fr_, double a_rl_, double a_rr_);
    void getWheelAccelerations(double (&v_wheel_)[4]) const;
    void setWheelAccelerations(const double v_wheel_[4]);

    // getters and setters for forces and moments
    void getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const;
    void getBodyForcesAndMoments(double Fx_, double Fy_, double Mz_);
    void getWheelNormalLoads(double &Fz_fl_, double &Fz_fr_, double &Fz_rl_, double &Fz_rr_) const;
    void setWheelNormalLoads(double Fz_fl_, double Fz_fr_, double Fz_rl_, double Fz_rr_);
    void getWheelNormalLoads(double (&Fz_wheel_)[4]) const;
    void setWheelNormalLoads(const double Fz_wheel_[4]);


private:
    // global 2D position
    double X, Y;

    // orientation (yaw)
    double yaw;

    // vehicle frame velocities
    double vx, vy;
    double w_yaw;

    // vehicle frame accelerations
    double ax, ay;
    double a_yaw;

    // wheel velocities and accelerations
    double v_wheel[4];
    double a_wheel[4];
    
    // forces
    double Fx, Fy, Mz;
    double Fx_wheel[4];
    double Fy_wheel[4];
    double Fz_wheel[4];

    // inputs
    double steering_angle;
    double throttle;

    friend class Vehicle;
};

// pacejka tire model
class TireConfig
{
public:
    // constructor
    TireConfig()
        : B(0.12), C(0.0), D(0.9), E(0.0), f(1.0) {}
    TireConfig(double B, double C, double D, double E, double f)
        : B(B), C(C), D(D), E(E), f(f) {}

    // tire forces
    double setTireConfig(double B, double C, double D, double E);
    double getTireConfig(double &B_, double &C_, double &D_, double &E_) const;

    void calcTireForces(double slipAngle, double slipRatio, double Fz_wheel, double &Fx_wheel, double &Fy_wheel);
private:
    double B; // stiffness factor
    double C; // shape factor
    double D; // peak factor
    double E; // curvature factor
    double f; // coeff of relative stiffness for long and lateral directions

    friend class VehicleConfig; // allow VehicleConfig to access private members
    friend class Vehicle;      // allow Vehicle to access private members
};

// vehicle configuration
class VehicleConfig
{
public:
    VehicleConfig()
        : wheelbase(0.0), track_width(0.0), steer_max(0.0),
          mass(0.0), Izz(0.0), z_cg(0.0), a(0.0), tire_config() {}
    VehicleConfig(double wheelbase_, double track_width_, double steer_max_,
                  double mass_, double Izz_,
                  double z_cg_, double a_, TireConfig tire_config_)
        : wheelbase(wheelbase_), track_width(track_width_), steer_max(steer_max_),
          mass(mass_), Izz(Izz_), z_cg(z_cg_), a(a_), tire_config(tire_config_) {}
    
    // getters for vehicle parameters
    double getMass() const;
    double getIzz() const;
    double getWheelbase() const;
    double getTrackWidth() const;
    double getSteerMax() const;
    double getZcg() const;
    double getA() const;
    double getWheelRadius() const { return r_wheel; }
private:
    double wheelbase;   // [m] distance between front and rear axles
    double track_width; // [m] distance between left and right wheels
    double steer_max;   // [rad] maximum steering angle
    double mass;        // [kg] vehicle mass
    double Izz;         // [kgm2] moment of inertia about the z-axis
    double z_cg;        // [m] static center of gravity height from ground
    double a;           // [m] distance from front axle to center of gravity
    double r_wheel; // [m] wheel radius
    double Tmax; // [Nm] maximum torque
    TireConfig tire_config; // tire parameters

    friend class Vehicle;
};

class Vehicle
{
public:
    Vehicle(const VehicleData &data_, const VehicleConfig &config_, const VehicleInput &input_)
        : data(data_), config(config_) {}
    Vehicle(const VehicleConfig &config_)
        : data(VehicleData()), config(config_) {}

    void calcWheelSlipsAndForces();
    void calcTireNormalLoads(); // update Fz_wheel array
    void calcNetForcesAndMoments();       // update Fx, Fy, Mz
    void calcAccelerations();   // update resultant body accelerations ax, ay, a_yaw
    void calcNewState(double dt);         // update vehicle state (X, Y, yaw, vx, vy, w_yaw)
    void stepSimulation();
    void setInput(double steering_angle, double throttle);
private:
    VehicleData data;
    VehicleConfig config;
    VehicleInput input;
};
