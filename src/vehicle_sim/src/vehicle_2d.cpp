#include "vehicle_2d.hpp"
#include <cmath> // for std::cos, std::sin, std::atan2, std::sqrt

VehicleInput::VehicleInput()
    : steering_input(0.0), throttle_input(0.0) {}

void VehicleInput::getSteeringInput(double &steering_input_) const
{
    steering_input_ = steering_input;
}
void VehicleInput::setSteeringInput(double steering_input_)
{
    if (steering_input_ < -1.0)
        steering_input = -1.0;
    else if (steering_input_ > 1.0)
        steering_input = 1.0;
    else
        steering_input = steering_input_;
}
void VehicleInput::getThrottleInput(double &throttle_input_) const
{
    throttle_input_ = throttle_input;
}
void VehicleInput::setThrottleInput(double throttle_input_)
{
    if (throttle_input_ < -1.0)
        throttle_input = -1.0;
    else if (throttle_input_ > 1.0)
        throttle_input = 1.0;
    else
        throttle_input = throttle_input_;
}
void VehicleInput::getBrakeInput(double &brake_input_) const
{
    brake_input_ = brake_input;
}
void VehicleInput::setBrakeInput(double brake_input_)
{
    if (brake_input_ < 0.0)
        brake_input = 0.0;
    else if (brake_input_ > 1.0)
        brake_input = 1.0;
    else
        brake_input = brake_input_;
}

VehicleData::VehicleData()
    : X(0.0), Y(0.0), yaw(0.0),
      vx(10.0), vy(0.0), w_yaw(0.0),
      ax(0.0), ay(0.0), a_yaw(0.0),
      w_wheel{10.0, 10.0, 10.0, 10.0},
      a_wheel{0.0, 0.0, 0.0, 0.0},
      Fx(0.0), Fy(0.0), Mz(0.0),
      Fx_wheel{0.0, 0.0, 0.0, 0.0},
      Fy_wheel{0.0, 0.0, 0.0, 0.0},
      Fz_wheel{0.0, 0.0, 0.0, 0.0},
      steering_angle(0.0), motor_torque(0.0),
      wheel_torques{0.0, 0.0, 0.0, 0.0} {}

// getters and setters for inputs
double VehicleData::getSteeringAngle() const { return steering_angle; }
void VehicleData::setSteeringAngle(const double steering_angle_) { steering_angle = steering_angle_; }
double VehicleData::getMotorTorque() const { return motor_torque; }
void VehicleData::setMotorTorque(const double motor_torque_) { motor_torque = motor_torque_; }
void VehicleData::getBrakeTorque(double (&brake_torque_)[4]) const
{
    brake_torque_[0] = brake_torque[0];
    brake_torque_[1] = brake_torque[1];
    brake_torque_[2] = brake_torque[2];
    brake_torque_[3] = brake_torque[3];
}
void VehicleData::setBrakeTorque(const double brake_torque_[4])
{
    brake_torque[0] = brake_torque_[0];
    brake_torque[1] = brake_torque_[1];
    brake_torque[2] = brake_torque_[2];
    brake_torque[3] = brake_torque_[3];
}

// getters and setters for position and orientation
void VehicleData::getPosition(double &x_, double &y_) const
{
    x_ = X;
    y_ = Y;
}
void VehicleData::setPosition(const double x_, const double y_)
{
    X = x_;
    Y = y_;
}
void VehicleData::getOrientation(double &yaw_) const { yaw_ = yaw; }
void VehicleData::setOrientation(const double yaw_) { yaw = yaw_; }

// getters and setters for velocities
void VehicleData::getLinearVelocities(double &vx_, double &vy_) const
{
    vx_ = vx;
    vy_ = vy;
}
void VehicleData::setLinearVelocities(const double vx_, const double vy_)
{
    vx = vx_;
    vy = vy_;
}
void VehicleData::getAngularVelocities(double &w_yaw_) const { w_yaw_ = w_yaw; }
void VehicleData::setAngularVelocities(const double w_yaw_) { w_yaw = w_yaw_; }

void VehicleData::getWheelVelocities(double &w_wheel_fl_, double &w_wheel_fr_, double &w_wheel_rl_, double &w_wheel_rr_) const
{
    w_wheel_fl_ = w_wheel[0];
    w_wheel_fr_ = w_wheel[1];
    w_wheel_rl_ = w_wheel[2];
    w_wheel_rr_ = w_wheel[3];
}
void VehicleData::setWheelVelocities(const double w_wheel_fl_, const double w_wheel_fr_, const double w_wheel_rl_, const double w_wheel_rr_)
{
    w_wheel[0] = w_wheel_fl_;
    w_wheel[1] = w_wheel_fr_;
    w_wheel[2] = w_wheel_rl_;
    w_wheel[3] = w_wheel_rr_;
}
void VehicleData::getWheelVelocities(double (&w_wheel_)[4]) const
{
    w_wheel_[0] = w_wheel[0];
    w_wheel_[1] = w_wheel[1];
    w_wheel_[2] = w_wheel[2];
    w_wheel_[3] = w_wheel[3];
}
void VehicleData::setWheelVelocities(const double w_wheel_[4])
{
    w_wheel[0] = w_wheel_[0];
    w_wheel[1] = w_wheel_[1];
    w_wheel[2] = w_wheel_[2];
    w_wheel[3] = w_wheel_[3];
}

// getters and setters for accelerations
void VehicleData::getLinearAccelerations(double &ax_, double &ay_) const
{
    ax_ = ax;
    ay_ = ay;
}
void VehicleData::setLinearAccelerations(const double ax_, const double ay_)
{
    ax = ax_;
    ay = ay_;
}
void VehicleData::getAngularAccelerations(double &a_yaw_) const { a_yaw_ = a_yaw; }
void VehicleData::setAngularAccelerations(const double a_yaw_) { a_yaw = a_yaw_; }
void VehicleData::getWheelAccelerations(double &a_fl_, double &a_fr_, double &a_rl_, double &a_rr_) const
{
    a_fl_ = a_wheel[0];
    a_fr_ = a_wheel[1];
    a_rl_ = a_wheel[2];
    a_rr_ = a_wheel[3];
}
void VehicleData::setWheelAccelerations(const double a_fl_, const double a_fr_, const double a_rl_, const double a_rr_)
{
    a_wheel[0] = a_fl_;
    a_wheel[1] = a_fr_;
    a_wheel[2] = a_rl_;
    a_wheel[3] = a_rr_;
}
void VehicleData::getWheelAccelerations(double (&a_wheel_)[4]) const
{
    a_wheel_[0] = a_wheel[0];
    a_wheel_[1] = a_wheel[1];
    a_wheel_[2] = a_wheel[2];
    a_wheel_[3] = a_wheel[3];
}
void VehicleData::setWheelAccelerations(const double a_wheel_[4])
{
    a_wheel[0] = a_wheel_[0];
    a_wheel[1] = a_wheel_[1];
    a_wheel[2] = a_wheel_[2];
    a_wheel[3] = a_wheel_[3];
}

// getters and setters for forces and moments
void VehicleData::getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const
{
    Fx_ = Fx;
    Fy_ = Fy;
    Mz_ = Mz;
}
void VehicleData::setBodyForcesAndMoments(const double Fx_, const double Fy_, const double Mz_)
{
    Fx = Fx_;
    Fy = Fy_;
    Mz = Mz_;
}
void VehicleData::getWheelNormalLoads(double (&Fz_wheel_)[4]) const
{
    Fz_wheel_[0] = Fz_wheel[0];
    Fz_wheel_[1] = Fz_wheel[1];
    Fz_wheel_[2] = Fz_wheel[2];
    Fz_wheel_[3] = Fz_wheel[3];
}
void VehicleData::setWheelNormalLoads(const double Fz_wheel_[4])
{
    Fz_wheel[0] = Fz_wheel_[0];
    Fz_wheel[1] = Fz_wheel_[1];
    Fz_wheel[2] = Fz_wheel_[2];
    Fz_wheel[3] = Fz_wheel_[3];
}
void VehicleData::getWheelForces(double (&Fx_wheel_)[4], double (&Fy_wheel_)[4]) const
{
    Fx_wheel_[0] = Fx_wheel[0];
    Fx_wheel_[1] = Fx_wheel[1];
    Fx_wheel_[2] = Fx_wheel[2];
    Fx_wheel_[3] = Fx_wheel[3];
    Fy_wheel_[0] = Fy_wheel[0];
    Fy_wheel_[1] = Fy_wheel[1];
    Fy_wheel_[2] = Fy_wheel[2];
    Fy_wheel_[3] = Fy_wheel[3];
}
void VehicleData::setWheelForces(const double Fx_wheel_[4], const double Fy_wheel_[4])
{
    Fx_wheel[0] = Fx_wheel_[0];
    Fx_wheel[1] = Fx_wheel_[1];
    Fx_wheel[2] = Fx_wheel_[2];
    Fx_wheel[3] = Fx_wheel_[3];
    Fy_wheel[0] = Fy_wheel_[0];
    Fy_wheel[1] = Fy_wheel_[1];
    Fy_wheel[2] = Fy_wheel_[2];
    Fy_wheel[3] = Fy_wheel_[3];
}
void VehicleData::getWheelTorques(double (&wheel_torques_)[4]) const
{
    wheel_torques_[0] = wheel_torques[0];
    wheel_torques_[1] = wheel_torques[1];
    wheel_torques_[2] = wheel_torques[2];
    wheel_torques_[3] = wheel_torques[3];
}
void VehicleData::setWheelTorques(const double (&wheel_torques_)[4])
{
    wheel_torques[0] = wheel_torques_[0];
    wheel_torques[1] = wheel_torques_[1];
    wheel_torques[2] = wheel_torques_[2];
    wheel_torques[3] = wheel_torques_[3];
}

TireConfig::TireConfig()
    : B(0.7), C(1.5), D(1.0), E(0.8), f(100.0) {} //
TireConfig::TireConfig(double B_, double C_, double D_, double E_, double f_)
    : B(B_), C(C_), D(D_), E(E_), f(f_) {}

// tire forces
void TireConfig::setTireConfig(const double B_, const double C_, const double D_, const double E_, const double f_)
{
    B = B_;
    C = C_;
    D = D_;
    E = E_;
    f = f_;
}
void TireConfig::getTireConfig(double &B_, double &C_, double &D_, double &E_, double &f_) const
{
    B_ = B;
    C_ = C;
    D_ = D;
    E_ = E;
    f_ = f;
}

void TireConfig::calcTireForces(const double slip_angle, const double slip_ratio, const double Fz_wheel, double &Fx_wheel, double &Fy_wheel)
{
    // slip_angle [deg]
    // slip_ratio [ratio]
    double slipNet = std::sqrt(slip_angle * slip_angle + slip_ratio * slip_ratio * f * f);
    if (slipNet < 1e-6)
    {
        Fx_wheel = 0.0;
        Fy_wheel = 0.0;
        return;
    }

    // pacejka tire formula
    double Fnet = D * std::sin(C * std::atan(B * slipNet - E * (B * slipNet - std::atan(B * slipNet))));

    // calculate forces in each direction
    double F_partial = Fz_wheel * Fnet / slipNet;
    Fx_wheel = F_partial * (slip_ratio * f);
    Fy_wheel = -F_partial * slip_angle;
}

VehicleConfig::VehicleConfig()
    : wheelbase(2.57), track_width(1.79), steer_max(30.0 * M_PI / 180.0),
      mass(1250.0), Izz(1800.0), z_cg(0.6), a(1.24), r_wheel(0.3125),
      I_wheel(10), Tmax(250.0), Tnegmax(250.0), Pmax(152e3), Pnegmax(100e3),
      brake_Tmax(2e4), brake_bias(0.55),
      diff_damping(10.0), tire_config() {} // Toyota 86 (https://en.wikipedia.org/wiki/Toyota_86)
VehicleConfig::VehicleConfig(double wheelbase_, double track_width_, double steer_max_,
                             double mass_, double Izz_,
                             double z_cg_, double a_, double r_wheel_, double I_wheel_,
                             double Tmax_, double Tnegmax_, double Pmax_, double Pnegmax_,
                             double brake_Tmax_, double brake_bias_,
                             double diff_damping_, TireConfig tire_config_)
    : wheelbase(wheelbase_), track_width(track_width_), steer_max(steer_max_),
      mass(mass_), Izz(Izz_), z_cg(z_cg_), a(a_), r_wheel(r_wheel_),
      I_wheel(I_wheel_), Tmax(Tmax_), diff_damping(diff_damping_),
      tire_config(tire_config_) {}

double VehicleConfig::getMass() const { return mass; }
double VehicleConfig::getIzz() const { return Izz; }
double VehicleConfig::getWheelbase() const { return wheelbase; }
double VehicleConfig::getTrackWidth() const { return track_width; }
double VehicleConfig::getSteerMax() const { return steer_max; }
double VehicleConfig::getZcg() const { return z_cg; }
double VehicleConfig::getA() const { return a; }
double VehicleConfig::getWheelRadius() const { return r_wheel; }
double VehicleConfig::getTmax() const { return Tmax; }
double VehicleConfig::getTnegmax() const { return Tnegmax; }
double VehicleConfig::getPmax() const { return Pmax; }
double VehicleConfig::getPnegmax() const { return Pnegmax; }
double VehicleConfig::getBrakeTmax() const { return brake_Tmax; }
double VehicleConfig::getBrakeBias() const { return brake_bias; }
double VehicleConfig::getDiffDamping() const { return diff_damping; }
double VehicleConfig::getI_wheel(void) const { return I_wheel; }
TireConfig VehicleConfig::getTireConfig() const { return tire_config; }

Vehicle::Vehicle() : data(VehicleData()), config(VehicleConfig()), input(VehicleInput()) {}
Vehicle::Vehicle(const VehicleConfig &config_)
    : data(VehicleData()), config(config_), input(VehicleInput()) {}
Vehicle::Vehicle(const VehicleData &data_, const VehicleConfig &config_, const VehicleInput &input_)
    : data(data_), config(config_), input(input_) {}

void Vehicle::calcMotorTorque()
{
    if (input.throttle_input < 0.0)
    {
        double T_Pmax = data.w_wheel[2] + data.w_wheel[3];
        data.setMotorTorque(input.throttle_input * std::abs(config.getTnegmax()));
        return;
    }
    data.setMotorTorque(input.throttle_input * config.getTmax());
}

void Vehicle::calcBrakeTorque()
{
    double brakeTorque[4];
    double brake_bias = config.getBrakeBias();
    double brake_Tmax = config.getBrakeTmax();
    double halfBrakeTorqueFront = input.brake_input * brake_Tmax * brake_bias / 2.0;
    double halfBrakeTorqueRear = input.brake_input * brake_Tmax * (1.0 - brake_bias) / 2.0;
    double w_wheel[4];
    data.getWheelVelocities(w_wheel);
    double sign[4];
    sign[0] = copysign(1.0, w_wheel[0]) * std::min(1.0, std::abs(w_wheel[0]) * 10);
    sign[1] = copysign(1.0, w_wheel[1]) * std::min(1.0, std::abs(w_wheel[1]) * 10);
    sign[2] = copysign(1.0, w_wheel[2]) * std::min(1.0, std::abs(w_wheel[2]) * 10);
    sign[3] = copysign(1.0, w_wheel[3]) * std::min(1.0, std::abs(w_wheel[3]) * 10);

    brakeTorque[0] = sign[0] * halfBrakeTorqueFront;
    brakeTorque[1] = sign[1] * halfBrakeTorqueFront;
    brakeTorque[2] = sign[2] * halfBrakeTorqueRear;
    brakeTorque[3] = sign[3] * halfBrakeTorqueRear;
    data.setBrakeTorque(brakeTorque);
}

void Vehicle::calcSteeringAngle()
{
    data.setSteeringAngle(input.steering_input * config.getSteerMax());
}

void Vehicle::calcTractionTorquesRWD()
{
    double w_wheel[4];
    data.getWheelVelocities(w_wheel);
    double brake_torque[4];
    data.getBrakeTorque(brake_torque);
    const double diff_damping = config.getDiffDamping();
    const double motor_torque = data.getMotorTorque();

    const double dw_wheel_front = w_wheel[0] - w_wheel[1]; // left - right
    const double dw_wheel_rear = w_wheel[2] - w_wheel[3];
    const double damping_rear = diff_damping * dw_wheel_rear;

    // Calculate wheel torques with viscous damping
    double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
    wheel_torques[2] = motor_torque / 2.0 - damping_rear;
    wheel_torques[3] = motor_torque / 2.0 + damping_rear;

    // subtract brake torque from wheel torques
    for (int i = 0; i < 4; ++i)
    {
        wheel_torques[i] -= brake_torque[i];
    }
    data.setWheelTorques(wheel_torques);
}

void Vehicle::calcTractionTorquesAWD()
{
    double w_wheel[4];
    data.getWheelVelocities(w_wheel);
    const double diff_damping = config.getDiffDamping();
    const double motor_torque = data.getMotorTorque();
    const double dw_wheel_front = w_wheel[0] - w_wheel[1]; // left - right
    const double dw_wheel_rear = w_wheel[2] - w_wheel[3];
    const double damping_front = diff_damping * dw_wheel_front;
    const double damping_rear = diff_damping * dw_wheel_rear;

    // Calculate wheel torques with viscous damping
    double wheel_torques[4];
    wheel_torques[0] = motor_torque / 4.0 - damping_front;
    wheel_torques[1] = motor_torque / 4.0 + damping_front;
    wheel_torques[2] = motor_torque / 4.0 - damping_rear;
    wheel_torques[3] = motor_torque / 4.0 + damping_rear;
    data.setWheelTorques(wheel_torques);
}

void Vehicle::calcTireNormalLoads() // update Fz_wheel array
{
    // get vehicle parameters
    const double g = 9.81;
    const double m = config.getMass();
    const double a = config.getA();
    const double b = config.getWheelbase() - a;
    const double h = config.getZcg();
    const double wheelbase = config.getWheelbase();
    const double track_width = config.getTrackWidth();
    double ax, ay;
    data.getLinearAccelerations(ax, ay);

    const double Fz_part = m * g / wheelbase / 2.0;
    const double Fz_front = Fz_part * b;
    const double Fz_rear = Fz_part * a;

    // calc load transfer
    const double dFz_x = (h * m * ax / wheelbase) / 2.0;
    const double dFz_y = (h * m * ay / track_width) / 2.0;

    // calc and set normal loads on each wheel
    double Fz_wheel[4] = {
        Fz_front - dFz_x - dFz_y, // front left
        Fz_front - dFz_x + dFz_y, // front right
        Fz_rear + dFz_x - dFz_y,  // rear left
        Fz_rear + dFz_x + dFz_y   // rear right
    };

    // check for negative normal loads
    int neg_count = 0;
    double Fz_neg = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        if (Fz_wheel[i] < 0.0)
        {
            neg_count++;
            Fz_neg += Fz_wheel[i];
            Fz_wheel[i] = 0.0;
        }
    }
    // if neg count, distribute loads evenly
    if (neg_count > 0)
    {
        for (int i = 0; i < 4; ++i)
        {
            if (Fz_wheel[i] > 0.0)
            {
                Fz_wheel[i] += Fz_neg / (4 - neg_count);
            }
        }
    }

    data.setWheelNormalLoads(Fz_wheel);
}

void Vehicle::calcWheelSlipsAndForces()
{
    // get vehicle states
    const double steering_angle = data.getSteeringAngle();
    const double a = config.getA();
    const double b = config.getWheelbase() - a;
    const double half_track_width = config.getTrackWidth() / 2.0;
    const double r_wheel = config.getWheelRadius();

    double w_wheel[4];
    double Fz_wheel[4];
    double vx, vy, w_yaw;
    data.getWheelVelocities(w_wheel);
    data.getWheelNormalLoads(Fz_wheel);
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);

    // calculate velocities at wheel centers
    const double vx_yaw = half_track_width * w_yaw;
    double vxw[4] = {
        vx - vx_yaw,  // fl
        vx + vx_yaw,  // fr
        vx - vx_yaw,  // rl
        vx + vx_yaw}; // rr
    double vyw[4] = {
        vy + a * w_yaw,
        vy + a * w_yaw,
        vy - b * w_yaw,
        vy - b * w_yaw};

    // resolve x and y velocities in wheel frame for wheels that steer
    for (int i = 0; i < 2; ++i)
    {
        const double vxw_temp = vxw[i];
        vxw[i] = vxw_temp * std::cos(steering_angle) + vyw[i] * std::sin(steering_angle);
        vyw[i] = vyw[i] * std::cos(steering_angle) - vxw_temp * std::sin(steering_angle);
    }

    // calculate slip angles
    double slip_angle[4];
    double slip_ratio[4];
    slip_angle[0] = std::atan2(vyw[0], vxw[0]); // front left
    slip_angle[1] = std::atan2(vyw[1], vxw[1]); // front right
    slip_angle[2] = std::atan2(vyw[2], vxw[2]); // rear left
    slip_angle[3] = std::atan2(vyw[3], vxw[3]); // rear right

    double Fx_wheel[4];
    double Fy_wheel[4];
    const double slip_threshold = 0.1; // threshold for slip ratio
    for (int i = 0; i < 4; ++i)
    {
        // calc slip ratios
        double denominator = std::max(std::max(std::abs(w_wheel[i] * r_wheel), std::abs(vxw[i])), slip_threshold);
        slip_ratio[i] = (w_wheel[i] * r_wheel - vxw[i]) / denominator;

        // calculate tire forces
        config.getTireConfig().calcTireForces(slip_angle[i] * 180.0 / M_PI, slip_ratio[i], Fz_wheel[i], Fx_wheel[i], Fy_wheel[i]);
    }
    data.setWheelForces(Fx_wheel, Fy_wheel);
}

void Vehicle::calcNetForcesAndMoments() // update Fx, Fy, Mz
{
    // calculate net forces and moments acting on the vehicle
    const double steering_angle = data.getSteeringAngle();
    const double a = config.getA();
    const double track_width = config.getTrackWidth();
    const double wheelbase = config.getWheelbase();
    const double b = wheelbase - a;
    const double half_track_width = track_width / 2.0;
    double Fx_body[4];
    double Fy_body[4];
    double Fx_wheel[4];
    double Fy_wheel[4];
    data.getWheelForces(Fx_wheel, Fy_wheel);

    double Fx = 0.0;
    double Fy = 0.0;
    double Mz = 0.0;

    // resolve forces in vehicle frame for wheels that steer
    for (int i = 0; i < 2; ++i)
    {
        double Fx_temp = Fx_wheel[i];
        Fx_body[i] = Fx_temp * std::cos(steering_angle) - Fy_wheel[i] * std::sin(steering_angle);
        Fy_body[i] = Fx_temp * std::sin(steering_angle) + Fy_wheel[i] * std::cos(steering_angle);
        Fx += Fx_body[i];
        Fy += Fy_body[i];
    }
    // resolve forces in vehicle frame for wheels that do not steer
    for (int i = 2; i < 4; ++i)
    {
        Fx_body[i] = Fx_wheel[i];
        Fy_body[i] = Fy_wheel[i];
        Fx += Fx_body[i];
        Fy += Fy_body[i];
    }
    // calculate moments about the center of mass
    Mz += -Fx_body[0] * half_track_width + Fy_body[0] * a; // front left
    Mz += Fx_body[1] * half_track_width + Fy_body[1] * a;  // front right
    Mz += -Fx_body[2] * half_track_width - Fy_body[2] * b; // rear left
    Mz += Fx_body[3] * half_track_width - Fy_body[3] * b;  // rear right

    data.setBodyForcesAndMoments(Fx, Fy, Mz);
}

void Vehicle::calcBodyAccelerations() // update resultant body accelerations ax, ay, a_yaw
{
    // calculate resultant body accelerations based on net forces and moments
    double ax, ay, a_yaw;
    const double mass = config.getMass();
    const double Izz = config.getIzz();
    double Fx, Fy, Mz;
    data.getBodyForcesAndMoments(Fx, Fy, Mz);

    double vx, vy, w_yaw;
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);
    // double vehicle_slip_angle = std::atan2(vy, vx);

    ax = Fx / mass;
    ay = Fy / mass;
    a_yaw = Mz / Izz;

    data.setLinearAccelerations(ax, ay);
    data.setAngularAccelerations(a_yaw);
}

void Vehicle::calcWheelAccelerations()
{
    double a_wheel[4];
    double I_wheel = config.getI_wheel();
    double wheel_radius = config.getWheelRadius();

    double wheel_torques[4];
    double Fx_wheel[4];
    double Fy_wheel[4];
    data.getWheelTorques(wheel_torques);
    data.getWheelForces(Fx_wheel, Fy_wheel);

    for (int i = 0; i < 4; ++i)
    {
        a_wheel[i] = (wheel_torques[i] - Fx_wheel[i] * wheel_radius) / I_wheel;
    }
    data.setWheelAccelerations(a_wheel);
}

void Vehicle::calcNewState(double dt) // update vehicle state (X, Y, yaw, vx, vy, w_yaw)
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

    // calculate new velocities
    vx += (ax + w_yaw * vy) * dt;
    vy += (ay - w_yaw * vx) * dt;
    w_yaw += a_yaw * dt;

    // calculate new position and orientation
    double sin_yaw = std::sin(yaw);
    double cos_yaw = std::cos(yaw);
    X += (vx * cos_yaw - vy * sin_yaw) * dt;
    Y += (vx * sin_yaw + vy * cos_yaw) * dt;
    yaw += w_yaw * dt;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));

    // calculate new wheel velocities
    double w_wheel[4];
    double a_wheel[4];
    data.getWheelVelocities(w_wheel);
    data.getWheelAccelerations(a_wheel);
    for (int i = 0; i < 4; ++i)
    {
        w_wheel[i] += a_wheel[i] * dt;
    }

    // set
    data.setPosition(X, Y);
    data.setOrientation(yaw);
    data.setLinearVelocities(vx, vy);
    data.setAngularVelocities(w_yaw);
    data.setWheelVelocities(w_wheel);
}

bool Vehicle::stepSimulation(double dt, double steering_input, double throttle_input, double brake_input)
{
    input.setSteeringInput(steering_input);
    input.setThrottleInput(throttle_input);
    input.setBrakeInput(brake_input);
    calcSteeringAngle();
    calcMotorTorque();
    calcBrakeTorque();
    calcTractionTorquesRWD();
    calcTireNormalLoads();
    calcWheelSlipsAndForces();
    calcNetForcesAndMoments();
    calcBodyAccelerations();
    calcWheelAccelerations();
    calcNewState(dt);

    return true;
}
VehicleData Vehicle::getVehicleData() const { return data; }
VehicleConfig Vehicle::getVehicleConfig() const { return config; }
VehicleInput Vehicle::getVehicleInput() const { return input; }
