#include "Vehicle2D_ST.hpp"
#include <cmath> // for std::cos, std::sin, std::atan2, std::sqrt

Vehicle2D_ST::Vehicle2D_ST() : data(Vehicle2D_STData()), config(Vehicle2D_STConfig()), input(VehicleInput()) {}
Vehicle2D_ST::Vehicle2D_ST(const Vehicle2D_STConfig &config_)
    : data(Vehicle2D_STData()), config(config_), input(VehicleInput()) {}
Vehicle2D_ST::Vehicle2D_ST(const Vehicle2D_STData &data_, const Vehicle2D_STConfig &config_, const VehicleInput &input_)
    : data(data_), config(config_), input(input_) {}

void Vehicle2D_ST::calcMotorTorque()
{
    const double Tmax = config.getTmax();
    const double Tnegmax = config.getTnegmax();
    const double Tzero = config.getTzero();
    const double motor_speed = data.w_wheel[1] / 2.0 * config.getGearRatio();

    // power limit
    const double one_over_abs_motor_speed = 1 / std::abs(motor_speed);
    const double T_Pmax = config.getPmax() * one_over_abs_motor_speed;
    const double T_Pnegmax = config.getPnegmax() * one_over_abs_motor_speed;

    // handle negative throttle
    double motor_torque = 0.0;
    if (std::signbit(input.throttle_input) == std::signbit(motor_speed))
    {
        motor_torque = input.throttle_input * (std::min(Tmax, T_Pmax) + Tzero) - std::copysign(Tzero, motor_speed);
    }
    else
    {
        motor_torque = input.throttle_input * (std::min(Tnegmax, T_Pnegmax) + Tzero) - std::copysign(Tzero, motor_speed);
    }
    data.setMotorTorque(motor_torque);
}

void Vehicle2D_ST::calcBrakeTorque()
{
    double brakeTorque[2];
    double brake_bias = config.getBrakeBias();
    double brake_Tmax = config.getBrakeTmax();
    double brake_torque_front = input.brake_input * brake_Tmax * brake_bias;
    double brake_torque_rear = input.brake_input * brake_Tmax * (1.0 - brake_bias);
    double w_wheel[2];
    data.getWheelVelocities(w_wheel);
    double sign[2];
    sign[0] = copysign(1.0, w_wheel[0]) * std::min(1.0, std::abs(w_wheel[0]) * 10);
    sign[1] = copysign(1.0, w_wheel[1]) * std::min(1.0, std::abs(w_wheel[1]) * 10);

    brakeTorque[0] = sign[0] * brake_torque_front;
    brakeTorque[1] = sign[1] * brake_torque_rear;
    data.setBrakeTorque(brakeTorque);
}

void Vehicle2D_ST::calcSteeringAngle()
{
    data.setSteeringAngle(input.steering_input * config.getSteerMax());
}

void Vehicle2D_ST::calcTractionTorquesRWD()
{
    // get data
    double w_wheel[2];
    data.getWheelVelocities(w_wheel);
    double brake_torque[2];
    data.getBrakeTorque(brake_torque);
    const double motor_torque = data.getMotorTorque();
    double gear_ratio = config.getGearRatio();

    // calculate wheel torques with viscous damping
    double wheel_torques[2] = {0.0, 0.0};
    wheel_torques[1] = motor_torque * gear_ratio;

    // subtract brake torque from wheel torques
    for (int i = 0; i < 2; ++i)
    {
        wheel_torques[i] -= brake_torque[i];
    }
    data.setWheelTorques(wheel_torques);
}

void Vehicle2D_ST::calcTractionTorques()
{

    if (config.getDrivetrainType() == RWD)
    {
        calcTractionTorquesRWD();
    }
    else if (config.getDrivetrainType() == FWD)
    {
        calcTractionTorquesFWD();
    }
    else if (config.getDrivetrainType() == AWD)
    {
        calcTractionTorquesAWD();
    }
}

void Vehicle2D_ST::calcTractionTorquesFWD()
{
    // get data
    double w_wheel[2];
    data.getWheelVelocities(w_wheel);
    double brake_torque[2];
    data.getBrakeTorque(brake_torque);
    const double motor_torque = data.getMotorTorque();
    double gear_ratio = config.getGearRatio();

    // calculate wheel torques with viscous damping
    double wheel_torques[2] = {0.0, 0.0};
    wheel_torques[0] = motor_torque * gear_ratio;

    // subtract brake torque from wheel torques
    for (int i = 0; i < 2; ++i)
    {
        wheel_torques[i] -= brake_torque[i];
    }
    data.setWheelTorques(wheel_torques);
}

void Vehicle2D_ST::calcTractionTorquesAWD()
{
    // get data
    double w_wheel[2];
    data.getWheelVelocities(w_wheel);
    double brake_torque[2];
    data.getBrakeTorque(brake_torque);
    const double motor_torque = data.getMotorTorque();
    double gear_ratio = config.getGearRatio();

    // calculate common torques
    const double net_wheel_torque = motor_torque * gear_ratio / 4.0;
    const double damping_torque_fr = 0.0 * // TODO diffdamping
                                     (w_wheel[0] - w_wheel[1]) / 2.0 * 0.0; // damping between front and rear wheels

    // Calculate wheel torques with viscous damping
    double wheel_torques[2] = {0.0, 0.0};
    wheel_torques[0] = net_wheel_torque - damping_torque_fr;
    wheel_torques[1] = net_wheel_torque + damping_torque_fr;

    // subtract brake torque from wheel torques
    for (int i = 0; i < 2; ++i)
    {
        wheel_torques[i] -= brake_torque[i];
    }
    data.setWheelTorques(wheel_torques);
}

void Vehicle2D_ST::calcTireNormalLoads() // update Fz_wheel array
{
    // get vehicle parameters
    const double g = 9.81;
    const double m = config.getMass();
    const double a = config.getA();
    const double b = config.getWheelbase() - a;
    const double h = config.getZcg();
    const double wheelbase = config.getWheelbase();
    double ax, ay;
    data.getLinearAccelerations(ax, ay);

    const double Fz_part = m * g / wheelbase;
    const double Fz_front = Fz_part * b;
    const double Fz_rear = Fz_part * a;

    // calc load transfer
    const double dFz_x = (h * m * ax / wheelbase) / 2.0;

    // calc and set normal loads on each wheel
    double Fz_wheel[2] = {
        Fz_front - dFz_x, // front
        Fz_rear + dFz_x,  // rear
    };

    // check for negative normal loads
    int neg_count = 0;
    double Fz_neg = 0.0;
    for (int i = 0; i < 2; ++i)
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
        for (int i = 0; i < 2; ++i)
        {
            if (Fz_wheel[i] > 0.0)
            {
                Fz_wheel[i] += Fz_neg / (2 - neg_count);
            }
        }
    }

    data.setWheelNormalLoads(Fz_wheel);
}

void Vehicle2D_ST::calcWheelSlipsAndForces()
{
    // get vehicle states
    const double steering_angle = data.getSteeringAngle();
    const double a = config.getA();
    const double b = config.getWheelbase() - a;
    const double r_wheel = config.getWheelRadius();

    double w_wheel[2];
    double Fz_wheel[2];
    double vx, vy, w_yaw;
    data.getWheelVelocities(w_wheel);
    data.getWheelNormalLoads(Fz_wheel);
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);

    // calculate velocities at wheel centers
    double vxw[2] = {
        vx,  // front
        vx}; // rear
    double vyw[2] = {
        vy + a * w_yaw,
        vy - b * w_yaw};

    // resolve x and y velocities in wheel frame for wheels that steer
    const double vxw_temp = vxw[0];
    vxw[0] = vxw_temp * std::cos(steering_angle) + vyw[0] * std::sin(steering_angle);
    vyw[0] = vyw[0] * std::cos(steering_angle) - vxw_temp * std::sin(steering_angle);

    // calculate slip angles
    double slip_angle[2];
    double slip_ratio[2];
    slip_angle[0] = std::atan2(vyw[0], vxw[0]); // front
    slip_angle[1] = std::atan2(vyw[1], vxw[1]); // rear

    // calculate tire forces
    double Fx_wheel[2];
    double Fy_wheel[2];
    const double slip_threshold = 0.5; // threshold for slip ratio
    for (int i = 0; i < 2; ++i)
    {
        // calc slip ratios
        double denominator = std::max(std::max(std::abs(w_wheel[i] * r_wheel), std::abs(vxw[i])), slip_threshold);
        slip_ratio[i] = (w_wheel[i] * r_wheel - vxw[i]) / denominator;

        // calculate tire forces
        config.getTireConfig().calcTireForces(RAD2DEG(slip_angle[i]), slip_ratio[i], w_wheel[i], Fz_wheel[i], Fx_wheel[i], Fy_wheel[i]);
    }
    data.setWheelForces(Fx_wheel, Fy_wheel);
}

void Vehicle2D_ST::calcAerodynamicForces(double (&F)[3], double (&M)[3])
{
    double vx, vy, w_yaw;
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);

    F[0] = -0.5 * config.getCDx() * config.getRho() * config.getFrontalArea() * vx * vx;
    F[1] = 0.0;
    F[2] = 0.0;
    M[0] = 0.0;
    M[1] = 0.0;
    M[2] = -0.5 * config.getCMz() * config.getRho() * config.getFrontalArea() * w_yaw * w_yaw;
}

void Vehicle2D_ST::calcNetForcesAndMoments() // update Fx, Fy, Mz
{
    // calculate net forces and moments acting on the vehicle
    const double steering_angle = data.getSteeringAngle();
    const double a = config.getA();
    const double wheelbase = config.getWheelbase();
    const double b = wheelbase - a;
    double Fx_body[2];
    double Fy_body[2];
    double Fx_wheel[2];
    double Fy_wheel[2];
    data.getWheelForces(Fx_wheel, Fy_wheel);

    // aerodynamic forces
    double F_aero[3];
    double M_aero[3];
    calcAerodynamicForces(F_aero, M_aero);

    double Fx = F_aero[0];
    double Fy = F_aero[1];
    double Mz = M_aero[2];

    // resolve forces in vehicle frame for wheels that steer
    double Fx_temp = Fx_wheel[0];
    Fx_body[0] = Fx_temp * std::cos(steering_angle) - Fy_wheel[0] * std::sin(steering_angle);
    Fy_body[0] = Fx_temp * std::sin(steering_angle) + Fy_wheel[0] * std::cos(steering_angle);
    Fx += Fx_body[0];
    Fy += Fy_body[0];

    // resolve forces in vehicle frame for wheels that do not steer
    Fx_body[1] = Fx_wheel[1];
    Fy_body[1] = Fy_wheel[1];
    Fx += Fx_body[1];
    Fy += Fy_body[1];

    // calculate moments about the center of mass
    Mz += Fy_body[0] * a; // front
    Mz += - Fy_body[1] * b; // rear

    data.setBodyForcesAndMoments(Fx, Fy, Mz);
}

void Vehicle2D_ST::calcBodyAccelerations() // update resultant body accelerations ax, ay, a_yaw
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

void Vehicle2D_ST::calcWheelAccelerations()
{
    double a_wheel[2];
    double I_wheel = config.getI_wheel();
    double wheel_radius = config.getWheelRadius();

    double wheel_torques[2];
    double Fx_wheel[2];
    double Fy_wheel[2];
    data.getWheelTorques(wheel_torques);
    data.getWheelForces(Fx_wheel, Fy_wheel);

    for (int i = 0; i < 2; ++i)
    {
        a_wheel[i] = (wheel_torques[i] - Fx_wheel[i] * wheel_radius) / I_wheel;
    }

    data.setWheelAccelerations(a_wheel);
}

void Vehicle2D_ST::calcNewState(double dt) // update vehicle state (X, Y, yaw, vx, vy, w_yaw)
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
    double w_wheel[2];
    double a_wheel[2];
    double p_wheel[2];
    data.getWheelVelocities(w_wheel);
    data.getWheelAccelerations(a_wheel);
    data.getWheelRotations(p_wheel);
    for (int i = 0; i < 2; ++i)
    {
        w_wheel[i] += a_wheel[i] * dt;
    }
    for (int i = 0; i < 2; ++i)
    {
        p_wheel[i] += w_wheel[i] * dt + 0.5 * a_wheel[i] * dt * dt;
    }

    // set
    data.setPosition(X, Y);
    data.setOrientation(yaw);
    data.setLinearVelocities(vx, vy);
    data.setAngularVelocities(w_yaw);
    data.setWheelVelocities(w_wheel);
    data.setWheelRotations(p_wheel);
}

bool Vehicle2D_ST::stepSimulation(double dt, double steering_input, double throttle_input, double brake_input)
{
    input.setSteeringInput(steering_input);
    input.setThrottleInput(throttle_input);
    input.setBrakeInput(brake_input);
    calcSteeringAngle();
    calcMotorTorque();
    calcBrakeTorque();
    calcTractionTorques();
    calcTireNormalLoads();
    calcWheelSlipsAndForces();
    calcNetForcesAndMoments();
    calcBodyAccelerations();
    calcWheelAccelerations();
    calcNewState(dt);

    return true;
}
Vehicle2D_STData &Vehicle2D_ST::getVehicle2D_STData() { return data; }
Vehicle2D_STConfig &Vehicle2D_ST::getVehicle2D_STConfig() { return config; }
VehicleInput &Vehicle2D_ST::getVehicleInput() { return input; }
