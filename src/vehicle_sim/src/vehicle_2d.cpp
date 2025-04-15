#pragma once

#include <cmath>    // for mathematical constants if needed
#include <iostream> // for printing

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
    void getPosition(double &x_, double &y_) const
    {
        x_ = X;
        y_ = Y;
    }
    void setPosition(double x_, double y_)
    {
        X = x_;
        Y = y_;
    }
    void getOrientation(double &yaw_) const { yaw_ = yaw; }
    void setOrientation(double yaw_) { yaw = yaw_; }

    // getters and setters for velocities
    void getLinearVelocities(double &vx_, double &vy_) const
    {
        vx_ = vx;
        vy_ = vy;
    }
    void setLinearVelocities(double vx_, double vy_)
    {
        vx = vx_;
        vy = vy_;
    }
    void setAngularVelocities(double w_yaw_) { w_yaw = w_yaw_; }
    void getAngularVelocities(double &w_yaw_) const { w_yaw_ = w_yaw; }
    void getWheelVelocities(double &v_fl_, double &v_fr_, double &v_rl_, double &v_rr_)
    {
        v_fl_ = v_wheel[0]; v_fr_ = v_wheel[1];
        v_rl_ = v_wheel[2]; v_rr_ = v_wheel[3];
    }
    void setWheelVelocities(double v_fl_, double v_fr_, double v_rl_, double v_rr_)
    {
        v_wheel[0] = v_fl_; v_wheel[1] = v_fr_;
        v_wheel[2] = v_rl_; v_wheel[3] = v_rr_;
    }
    void getWheelVelocities(double (&v_wheel_)[4]) const
    {
        for (int i = 0; i < 4; ++i)
        {
            v_wheel_[i] = v_wheel[i];
        }
    }
    void setWheelVelocities(const double v_wheel_[4])
    {
        for (int i = 0; i < 4; ++i)
        {
            v_wheel[i] = v_wheel_[i];
        }
    }

    // getters and setters for accelerations
    void getLinearAccelerations(double &ax_, double &ay_) const
    {
        ax_ = ax;
        ay_ = ay;
    }
    void setLinearAccelerations(double ax_, double ay_)
    {
        ax = ax_;
        ay = ay_;
    }
    void getAngularAccelerations(double &a_yaw_) const { a_yaw_ = a_yaw; }
    void setAngularAccelerations(double a_yaw_) { a_yaw = a_yaw_; }
    void getWheelAccelerations(double &a_fl_, double &a_fr_, double &a_rl_, double &a_rr_)
    {
        a_fl_ = a_wheel[0]; a_fr_ = a_wheel[1];
        a_rl_ = a_wheel[2]; a_rr_ = a_wheel[3];
    }
    void setWheelAccelerations(double a_fl_, double a_fr_, double a_rl_, double a_rr_)
    {
        a_wheel[0] = a_fl_; a_wheel[1] = a_fr_;
        a_wheel[2] = a_rl_; a_wheel[3] = a_rr_;
    }
    void getWheelAccelerations(double (&a_wheel_)[4]) const
    {
        for (int i = 0; i < 4; ++i)
        {
            a_wheel_[i] = a_wheel[i];
        }
    }
    void setWheelAccelerations(const double a_wheel_[4])
    {
        for (int i = 0; i < 4; ++i)
        {
            a_wheel[i] = a_wheel_[i];
        }
    }

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

    friend class Vehicle;
};

// pacejka tire model
class TireConfig
{
public:
    // constructor
    TireConfig()
        : B(0.0), C(0.0), D(0.0), E(0.0) {}
    TireConfig(double B, double C, double D, double E)
        : B(B), C(C), D(D), E(E) {}

    // tire forces
    double calculateLateralForce(double slipAngle) const
    {
        return D * std::sin(C * std::atan(B * slipAngle - E * (B * slipAngle - std::atan(B * slipAngle))));
    }

    double calculateLongitudinalForce(double slipRatio) const
    {
        return D * std::sin(C * std::atan(B * slipRatio - E * (B * slipRatio - std::atan(B * slipRatio))));
    }

    double calcCombinedSlipForce(double slipAngle, double slipRatio) const
    {
        return 0.0;
    }
    double setTireConfig(double B_, double C_, double D_, double E_)
    {
        B = B_;
        C = C_;
        D = D_;
        E = E_;
    }

private:
    double B;                   // stiffness factor
    double C;                   // shape factor
    double D;                   // peak factor
    double E;                   // curvature factor
    friend class VehicleConfig; // allow VehicleConfig to access private members
    friend class Vehicle;       // allow Vehicle to access private members
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

private:
    double wheelbase;   // [m] distance between front and rear axles
    double track_width; // [m] distance between left and right wheels
    double steer_max;   // [rad] maximum steering angle
    double mass;        // [kg] vehicle mass
    double Izz;         // [kgm2] moment of inertia about the z-axis
    double z_cg;        // [m] static center of gravity height from ground
    double a;           // [m] distance from front axle to center of gravity

    TireConfig tire_config; // tire parameters

    friend class Vehicle;
};

class Vehicle
{
public:
    Vehicle(const VehicleData &data_, const VehicleConfig &config_)
        : data(data_), config(config_) {}

    void calcTireForces(double &Fx, double &Fy)
    {
        // Calculate forward slip velocity

        // Calculate forward force

        // Calculate lateral slip velocity

        // Calculate lateral force
    }
    void calcLinearAccelerations();
    void calcAngularAccelerations();
    void stepSimulation(double dt)
    {
        // Update vehicle state based on the current input and dt
        double ax, ay, a_yaw;
        calcLinearAccelerations();
        calcAngularAccelerations();

        // calculate accelerations without centrifugal terms
        data.vx += (ax - data.w_yaw * data.vy) * dt;
        data.vy += (ay + data.w_yaw * data.vx) * dt;
        data.w_yaw += a_yaw * dt;

        // update global position
        data.X += (data.vx * std::cos(data.yaw) - data.vy * std::sin(data.yaw)) * dt;
        data.Y += (data.vx * std::sin(data.yaw) + data.vy * std::cos(data.yaw)) * dt;

        // TODO update global position including acceleration terms
        // data.X += (data.vx * std::cos(data.yaw) - data.vy * std::sin(data.yaw)) * dt +
        //      0.5 * (ax * std::cos(data.yaw) - ay * std::sin(data.yaw)) * dt * dt;
        // data.Y += (data.vx * std::sin(data.yaw) + data.vy * std::cos(data.yaw)) * dt +
        //      0.5 * (ax * std::sin(data.yaw) + ay * std::cos(data.yaw)) * dt * dt;
        data.yaw += data.w_yaw * dt + 0.5 * data.a_yaw * dt * dt;
    }

private:
    VehicleData data;
    VehicleConfig config;
};
