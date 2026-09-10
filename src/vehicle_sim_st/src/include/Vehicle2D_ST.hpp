#pragma once

#include "TireConfig.hpp"
#include "Vehicle2D_STData.hpp"
#include "Vehicle2D_STConfig.hpp"
#include "VehicleInput.hpp"
#include "Vehicle2D_STCSVWriter.hpp"
#include <cmath> // for std::cos, std::sin, std::atan2, std::sqrt

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class Vehicle2D_ST
{
public:
    Vehicle2D_ST();
    Vehicle2D_ST(const Vehicle2D_STConfig &config_);
    Vehicle2D_ST(const Vehicle2D_STData &data_, const Vehicle2D_STConfig &config_, const VehicleInput &input_);

    void calcMotorTorque();
    void calcBrakeTorque();
    void calcSteeringAngle();
    void calcTractionTorques();
    void calcTractionTorquesFWD();
    void calcTractionTorquesRWD();
    void calcTractionTorquesAWD();
    void calcWheelSlipsAndForces();
    void calcTireNormalLoads();
    void calcAerodynamicForces(double (&F)[3], double (&M)[3]);
    void calcNetForcesAndMoments();
    void calcBodyAccelerations();
    void calcWheelAccelerations();
    void calcNewState(double dt);
    bool stepSimulation(double dt, double steering_input, double throttle_input, double brake_input);
    Vehicle2D_STData &getVehicle2D_STData();
    Vehicle2D_STConfig &getVehicle2D_STConfig();
    VehicleInput &getVehicleInput();

private:
    Vehicle2D_STData data;
    Vehicle2D_STConfig config;
    VehicleInput input;
};
