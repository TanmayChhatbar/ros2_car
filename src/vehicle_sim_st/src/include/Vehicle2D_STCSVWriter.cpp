#include "Vehicle2D_ST.hpp"

#include <iostream>
#include <fstream>

// cSV header and data formatting
void writeCSVHeader(std::ofstream &file)
{
     file << "time,X,Y,yaw,vx,vy,w_yaw,ax,ay,a_yaw,"
          << "w_wheel_rearont,w_wheel_rear,"
          << "Fx_wheel_rearont,Fx_wheel_rear,"
          << "Fy_wheel_rearont,Fy_wheel_rear,"
          << "steering,motor_torque" << std::endl;
}

void writeCSVData(std::ofstream &file, double time, const Vehicle2D_STData &data,
                  const VehicleInput &input)
{
     double X, Y, yaw;
     double vx, vy, w_yaw;
     double ax, ay, a_yaw;
     double w_wheel[2];
     double Fx_wheel[2], Fy_wheel[2];
     double steering_angle = data.getSteeringAngle();
     double motor_torque = data.getMotorTorque();

     data.getPosition(X, Y);
     data.getOrientation(yaw);
     data.getLinearVelocities(vx, vy);
     data.getAngularVelocities(w_yaw);
     data.getLinearAccelerations(ax, ay);
     data.getAngularAccelerations(a_yaw);
     data.getWheelVelocities(w_wheel);
     data.getWheelForces(Fx_wheel, Fy_wheel);

     file << time << ","
          << X << "," << Y << "," << yaw << ","
          << vx << "," << vy << "," << w_yaw << ","
          << ax << "," << ay << "," << a_yaw << ","
          << w_wheel[0] << "," << w_wheel[1] << ","
          << Fx_wheel[0] << "," << Fx_wheel[1] << ","
          << Fy_wheel[0] << "," << Fy_wheel[1] << ","
          << steering_angle << "," << motor_torque << std::endl;
}

void exportVehicle2D_STConfig(Vehicle2D_STConfig &Vehicle2D_STConfig, std::string filename)
{
     std::ofstream config_file(filename);
     if (!config_file.is_open())
     {
          std::cerr << "Failed to open file: vehicle_config.csv" << std::endl;
          return;
     }

     config_file << "wheelbase_m, steer_max_rad, mass_kg, Izz_kgm2, z_cg_m, a_m, r_wheel_m, I_wheel_kgm2, Tmax_Nm,"
                 << "tire_B, tire_C, tire_D, tire_E, tire_f" << std::endl;
     double B, C, D, E, f;
     Vehicle2D_STConfig.getTireConfig().getTireParams(B, C, D, E, f);

     config_file << Vehicle2D_STConfig.getWheelbase() << ","
                 << Vehicle2D_STConfig.getSteerMax() << ","
                 << Vehicle2D_STConfig.getMass() << ","
                 << Vehicle2D_STConfig.getIzz() << ","
                 << Vehicle2D_STConfig.getZcg() << ","
                 << Vehicle2D_STConfig.getA() << ","
                 << Vehicle2D_STConfig.getWheelRadius() << ","
                 << Vehicle2D_STConfig.getI_wheel() << ","
                 << Vehicle2D_STConfig.getTmax() << ","
                 << B << ","
                 << C << ","
                 << D << ","
                 << E << ","
                 << f << std::endl;

     config_file.close();
}
