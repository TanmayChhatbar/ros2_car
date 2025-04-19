#pragma once
// TODO :
// use VehicleInput

#include <cmath>    // for mathematical constants if needed
#include <iostream> // for printing

class VehicleInput
{
    public:
        // constructor
        VehicleInput()
            : steering_angle(0.0), motor_torque(0.0) {}

        // getters and setters for steering angle and motor torque
        void setSteeringAngle(double steering_angle_)
        {
            steering_angle = steering_angle_;
        }
        void getSteeringAngle(double &steering_angle_) const
        {
            steering_angle_ = steering_angle;
        }
        void setMotorTorque(double motor_torque_)
        {
            motor_torque = motor_torque_;
        }
        void getMotorTorque(double &motor_torque_) const
        {
            motor_torque_ = motor_torque;
        }
        void calcMotorTorque(double Tmax)
        {
            // calculate motor torque based on throttle input
            motor_torque = throttle * Tmax;
        }
        void getInput(double &steering_angle_, double &throttle_) const
        {
            steering_angle_ = steering_angle;
            throttle_ = throttle;
        }
        void setInput(double steering_angle, double throttle)
        {
            setSteeringAngle(steering_angle);
            setThrottle(throttle);
            calcMotorTorque();
        }
    private:
        double steering_angle; // [rad] steering angle
        double motor_torque;   // [Nm] motor torque
        double throttle;      // [0, 1] throttle input

        friend class Vehicle; // allow Vehicle to access private members
};

class VehicleData
{
public:
    // constructor
    VehicleData()
        : X(0.0), Y(0.0), yaw(0.0),
          vx(0.0), vy(0.0), w_yaw(0.0),
          v_wheel{0.0, 0.0, 0.0, 0.0} {}

    // getters and setters for inputs
    double getSteeringAngle() const { return steering_angle; }
    void setSteeringAngle(double steering_angle_) { steering_angle = steering_angle_; }
    double getThrottle() const { return throttle; }
    void setThrottle(double throttle_) { throttle = throttle_; }

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
        v_fl_ = v_wheel[0];
        v_fr_ = v_wheel[1];
        v_rl_ = v_wheel[2];
        v_rr_ = v_wheel[3];
    }
    void setWheelVelocities(double v_fl_, double v_fr_, double v_rl_, double v_rr_)
    {
        v_wheel[0] = v_fl_;
        v_wheel[1] = v_fr_;
        v_wheel[2] = v_rl_;
        v_wheel[3] = v_rr_;
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
        a_fl_ = a_wheel[0];
        a_fr_ = a_wheel[1];
        a_rl_ = a_wheel[2];
        a_rr_ = a_wheel[3];
    }
    void setWheelAccelerations(double a_fl_, double a_fr_, double a_rl_, double a_rr_)
    {
        a_wheel[0] = a_fl_;
        a_wheel[1] = a_fr_;
        a_wheel[2] = a_rl_;
        a_wheel[3] = a_rr_;
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

    // getters and setters for forces and moments
    void getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const
    {
        Fx_ = Fx;
        Fy_ = Fy;
        Mz_ = Mz;
    }
    void setBodyForcesAndMoments(double Fx_, double Fy_, double Mz_)
    {
        Fx = Fx_;
        Fy = Fy_;
        Mz = Mz_;
    }
    void getWheelNormalLoads(double &Fz_fl_, double &Fz_fr_, double &Fz_rl_, double &Fz_rr_) const
    {
        Fz_fl_ = Fz_wheel[0];
        Fz_fr_ = Fz_wheel[1];
        Fz_rl_ = Fz_wheel[2];
        Fz_rr_ = Fz_wheel[3];
    }
    void setWheelNormalLoads(double Fz_fl_, double Fz_fr_, double Fz_rl_, double Fz_rr_)
    {
        Fx_wheel[0] = Fz_fl_;
        Fx_wheel[1] = Fz_fr_;
        Fx_wheel[2] = Fz_rl_;
        Fx_wheel[3] = Fz_rr_;
    }
    void getWheelNormalLoads(double (&Fz_wheel_)[4]) const
    {
        Fz_wheel_[0] = Fz_wheel[0];
        Fz_wheel_[1] = Fz_wheel[1];
        Fz_wheel_[2] = Fz_wheel[2];
        Fz_wheel_[3] = Fz_wheel[3];
    }
    void setWheelNormalLoads(const double Fz_wheel_[4])
    {
        Fx_wheel[0] = Fz_wheel_[0];
        Fx_wheel[1] = Fz_wheel_[1];
        Fx_wheel[2] = Fz_wheel_[2];
        Fx_wheel[3] = Fz_wheel_[3];
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

    // inputs
    double steering_angle;
    double throttle;

    friend class Vehicle;
};

class TireConfig
{
    // pacejka tire model
public:
    // constructor
    TireConfig()
        : B(0.0), C(0.0), D(0.0), E(0.0) {}
    TireConfig(double B, double C, double D, double E)
        : B(B), C(C), D(D), E(E) {}

    // tire forces
    double setTireConfig(double B_, double C_, double D_, double E_, double f_)
    {
        B = B_;
        C = C_;
        D = D_;
        E = E_;
        f = f_;
    }
    double getTireConfig(double &B_, double &C_, double &D_, double &E_, double &f_) const
    {
        B_ = B;
        C_ = C;
        D_ = D;
        E_ = E;
        f_ = f;
    }

    void calcTireForces(double slipAngle, double slipRatio, double Fz_wheel, double &Fx_wheel, double &Fy_wheel)
    {
        // calculate tire forces using pacejka model
        double slipNet = sqrt(slipAngle * slipAngle + slipRatio * slipRatio * f * f);

        // pacejka tire formula
        if (slipNet == 0.0)
        {
            Fx_wheel = 0.0;
            Fy_wheel = 0.0;
            return;
        }
        double Fnet = D * sin(C * atan(B * slipNet - E * (B * slipNet - atan(B * slipNet))));
        Fx_wheel = Fnet * slipRatio * f / slipNet;
        Fy_wheel = Fnet * slipAngle / slipNet;
    }

private:
    double B; // stiffness factor
    double C; // shape factor
    double D; // peak factor
    double E; // curvature factor
    double f; // coeff of relative stiffness for long and lateral directions

    friend class VehicleConfig; // allow VehicleConfig to access private members
    friend class Vehicle;       // allow Vehicle to access private members
};

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

    double getMass() const { return mass; }
    double getIzz() const { return Izz; }
    double getWheelbase() const { return wheelbase; }
    double getTrackWidth() const { return track_width; }
    double getSteerMax() const { return steer_max; }
    double getZcg() const { return z_cg; }
    double getA() const { return a; }
    double getWheelRadius() const { return r_wheel; }

private:
    double wheelbase;   // [m] distance between front and rear axles
    double track_width; // [m] distance between left and right wheels
    double steer_max;   // [rad] maximum steering angle
    double mass;        // [kg] vehicle mass
    double Izz;         // [kgm2] moment of inertia about the z-axis
    double z_cg;        // [m] static center of gravity height from ground
    double a;           // [m] distance from front axle to center of gravity
    double r_wheel;
    double Tmax;

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
    Vehicle() : data(VehicleData()), config(VehicleConfig()) {}

    void calcWheelSlipsAndForces()
    {

        // get vehicle states
        double vx, vy, w_yaw;
        data.getLinearVelocities(vx, vy);
        data.setAngularVelocities(w_yaw);
        double steering_angle = data.getSteeringAngle();
        double a = config.getA();
        double b = config.getWheelbase() - a;
        double half_track_width = config.getTrackWidth() / 2.0;
        double r_wheel = config.getWheelRadius();
        double v_wheel[4];
        data.getWheelVelocities(v_wheel);

        double slipAngle[4];
        double slipRatio[4];
        double vxw[4] = {
            vx - half_track_width * w_yaw,
            vx + half_track_width * w_yaw,
            vx - half_track_width * w_yaw,
            vx + half_track_width * w_yaw};
        double vyw[4] = {
            vy + a * w_yaw,
            vy + a * w_yaw,
            vy - b * w_yaw,
            vy - b * w_yaw};
        slipAngle[0] = atan2(vyw[0], vxw[0]) - steering_angle; // front left
        slipAngle[1] = atan2(vyw[1], vxw[1]) - steering_angle; // front right
        slipAngle[2] = atan2(vyw[2], vxw[2]);                  // rear left
        slipAngle[3] = atan2(vyw[3], vxw[3]);                  // rear right

        double Fx_wheel[4];
        double Fy_wheel[4];
        for (int i = 0; i < 4; ++i)
        {
            // calc net slip
            slipRatio[i] = v_wheel[i] * r_wheel / vxw[i] - 1; // front left
            double slipNet = sqrt(slipAngle[i] * slipAngle[i] + slipRatio[i] * slipRatio[i]);

            // calculate tire forces
            config.tire_config.calcTireForces(slipAngle[i], slipRatio[i], data.Fz_wheel[i], Fx_wheel[i], Fy_wheel[i]);
        }
    }
    void calcTireNormalLoads() // update Fz_wheel array
    {
        // get vehicle parameters
        double m = config.getMass();
        double g = 9.81; // gravity
        double a = config.getA();
        double b = config.getWheelbase() - a;
        double h = config.getZcg();
        double track_width = config.getTrackWidth();
        double wheelbase = config.getWheelbase();

        double Fz_front = m * g * b / (a + b) / 2.0;
        double Fz_rear = m * g * a / (a + b) / 2.0;

        // calc load transfer
        double dFz_x = h * m * data.ax / config.wheelbase;
        double dFz_y = h * m * data.ay / (config.track_width / 2.0);

        // set normal loads on each wheel
        double Fz_wheel[4] = {
            Fz_front - dFz_x / 2.0 - dFz_y / 2.0, // front left
            Fz_front - dFz_x / 2.0 + dFz_y / 2.0, // front right
            Fz_rear + dFz_x / 2.0 - dFz_y / 2.0,  // rear left
            Fz_rear + dFz_x / 2.0 + dFz_y / 2.0   // rear right
        };
        data.setWheelNormalLoads(Fz_wheel);
    }
    void calcNetForcesAndMoments() // update Fx, Fy, Mz
    {
        // calculate net forces and moments acting on the vehicle
        double Fx = 0.0;
        double Fy = 0.0;
        double Mz = 0.0;
        for (int i = 0; i < 2; i++)
        {
            Fx += data.Fx_wheel[i] * cos(data.steering_angle) - data.Fy_wheel[i] * sin(data.steering_angle);
            Fy += data.Fy_wheel[i] * sin(data.steering_angle) + data.Fx_wheel[i] * cos(data.steering_angle);
        }
        for (int i = 2; i < 4; i++)
        {
            Fx += data.Fx_wheel[i];
            Fy += data.Fy_wheel[i];
        }
        Mz += -data.Fx_wheel[0] * config.track_width / 2.0 + data.Fy_wheel[0] * config.a;
        Mz += -data.Fx_wheel[1] * config.track_width / 2.0 + data.Fy_wheel[1] * config.a;
        Mz += -data.Fx_wheel[2] * config.track_width / 2.0 - data.Fy_wheel[2] * config.a;
        Mz += -data.Fx_wheel[3] * config.track_width / 2.0 - data.Fy_wheel[3] * config.a;
        data.setBodyForcesAndMoments(Fx, Fy, Mz);
    }
    void calcAccelerations() // update resultant body accelerations ax, ay, a_yaw
    {
        double ax, ay, a_yaw, Fx, Fy, Mz;
        double mass = config.getMass();
        double Izz = config.getIzz();
        data.getBodyForcesAndMoments(Fx, Fy, Mz);
        data.getLinearAccelerations(ax, ay);
        data.getAngularAccelerations(a_yaw);

        // calculate resultant body accelerations based on net forces and moments
        ax = Fx / config.mass;
        ay = Fy / config.mass;
        a_yaw = Mz / config.Izz;

        data.setLinearAccelerations(ax, ay);
        data.setAngularAccelerations(a_yaw);
    }
    void calcNewState(double dt) // update vehicle state (X, Y, yaw, vx, vy, w_yaw)
    {
        double vx, vy, w_yaw;
        data.getLinearVelocities(vx, vy);
        data.getAngularVelocities(w_yaw);

        double ax, ay, a_yaw;
        data.getLinearAccelerations(ax, ay);
        data.getAngularAccelerations(a_yaw);

        double X, Y, yaw;
        data.getPosition(X, Y);
        data.getOrientation(yaw);

        vx += (ax - w_yaw * vy) * dt;
        vy += (ay + w_yaw * vx) * dt;
        w_yaw += a_yaw * dt;

        X += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
        Y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
        yaw += w_yaw * dt + 0.5 * a_yaw * dt * dt;

        double v_wheel[4];
        double a_wheel[4];
        data.getWheelVelocities(v_wheel);
        data.getWheelAccelerations(a_wheel);
        for (int i = 0; i < 4; ++i)
        {
            v_wheel[i] += a_wheel[i] * dt;
        }
        data.setWheelVelocities(v_wheel);
    }
    void stepSimulation() 
    {
        input.setInput(0, 1);
        calcWheelSlipsAndForces();
        calcTireNormalLoads();
        calcNetForcesAndMoments();
        calcAccelerations();
        double dt = 0.01; // time step
        calcNewState(dt);
    }
private:
    VehicleData data;
    VehicleConfig config;
    VehicleInput input;
};
