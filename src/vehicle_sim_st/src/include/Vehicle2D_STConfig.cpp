#include "Vehicle2D_STConfig.hpp"
#include "TireConfig.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include "json.hpp"

Vehicle2D_STConfig::Vehicle2D_STConfig()
    : wheelbase(1.79), steer_max(30.0 * M_PI / 180.0),
      mass(1250.0), Izz(1800.0), z_cg(0.6), a(1.24), r_wheel(0.3125), I_wheel(2.0),
      Tmax(205), Tnegmax(250.0), Pmax(152e3), Pnegmax(100e3), gear_ratio(3.626 * 4.1),
      brake_Tmax(2e4), brake_bias(0.55),
      drivetrain_type(RWD),
      tire_config(),
      CDx(0.3), CMz(0.0), Af(1.775 * (1.425 - 0.1)), rho(1.225) {} // Toyota 86 (https://en.wikipedia.org/wiki/Toyota_86)
Vehicle2D_STConfig::Vehicle2D_STConfig(double wheelbase_, double steer_max_,
                                 double mass_, double Izz_, double z_cg_, double a_, double r_wheel_, double I_wheel_,
                                 double Tmax_, double Tzero_, double Tnegmax_, double Pmax_, double Pnegmax_, double gear_ratio_,
                                 double brake_Tmax_, double brake_bias_,
                                 DrivetrainType_E drivetrain_type_, 
                                 TireConfig tire_config_,
                                 double CDx_, double CMz_, double Af_, double rho_)
    : wheelbase(wheelbase_), steer_max(steer_max_),
      mass(mass_), Izz(Izz_), z_cg(z_cg_), a(a_), r_wheel(r_wheel_), I_wheel(I_wheel_),
      Tmax(Tmax_), Tzero(Tzero_), Tnegmax(Tnegmax_), Pmax(Pmax_), Pnegmax(Pnegmax_), gear_ratio(gear_ratio_),
      brake_Tmax(brake_Tmax_), brake_bias(brake_bias_),
      drivetrain_type(drivetrain_type_),
      tire_config(tire_config_),
      CDx(CDx_), CMz(CMz_), Af(Af_), rho(rho_) {}

double Vehicle2D_STConfig::getMass() const { return mass; }
double Vehicle2D_STConfig::getIzz() const { return Izz; }
double Vehicle2D_STConfig::getWheelbase() const { return wheelbase; }
double Vehicle2D_STConfig::getSteerMax() const { return steer_max; }
double Vehicle2D_STConfig::getZcg() const { return z_cg; }
double Vehicle2D_STConfig::getA() const { return a; }
double Vehicle2D_STConfig::getWheelRadius() const { return r_wheel; }
double Vehicle2D_STConfig::getTmax() const { return Tmax; }
double Vehicle2D_STConfig::getTnegmax() const { return Tnegmax; }
double Vehicle2D_STConfig::getTzero() const { return Tzero; }
double Vehicle2D_STConfig::getPmax() const { return Pmax; }
double Vehicle2D_STConfig::getPnegmax() const { return Pnegmax; }
double Vehicle2D_STConfig::getBrakeTmax() const { return brake_Tmax; }
double Vehicle2D_STConfig::getBrakeBias() const { return brake_bias; }
DrivetrainType_E Vehicle2D_STConfig::getDrivetrainType() const { return drivetrain_type; }
double Vehicle2D_STConfig::getI_wheel(void) const { return I_wheel; }
double Vehicle2D_STConfig::getGearRatio(void) const { return gear_ratio; }
double Vehicle2D_STConfig::getCDx(void) const { return CDx; }
double Vehicle2D_STConfig::getCMz(void) const { return CMz; }
double Vehicle2D_STConfig::getFrontalArea(void) const { return Af; }
double Vehicle2D_STConfig::getRho(void) const { return rho; }
TireConfig &Vehicle2D_STConfig::getTireConfig() { return tire_config; }

Vehicle2D_STConfig Vehicle2D_STConfig::loadFromFile(const std::string& filename) {
  try {
      // read JSON file
      std::ifstream file(filename);
      if (!file.is_open()) {
          std::cerr << "Error: Could not open file " << filename << std::endl;
          return Vehicle2D_STConfig(); // return default
      }
      
      nlohmann::json j;
      file >> j;
      
      const auto& vehicle = j;
      
      TireConfig tire_config;
      if (vehicle.contains("tire_config")) {
          const auto& tc = vehicle["tire_config"];
          tire_config = TireConfig(
              tc.value("B", 0.7),
              tc.value("C", 1.5),
              tc.value("D", 1.0),
              tc.value("E", 0.8),
              tc.value("f", 100.0),
              tc.value("coeff_rr", 0.03),
              tc.value("coeff_stiction", 0.008)
          );
      }
      
      // Create and return the Vehicle2D_STConfig
      return Vehicle2D_STConfig(
          vehicle.value("wheelbase", 2.57),
          vehicle.value("steer_max", 30.0 * M_PI / 180.0),
          vehicle.value("mass", 1250.0),
          vehicle.value("Izz", 1800.0),
          vehicle.value("z_cg", 0.6),
          vehicle.value("a", 1.24),
          vehicle.value("r_wheel", 0.3125),
          vehicle.value("I_wheel", 2.0) * 2.0,
          vehicle.value("Tmax", 205.0),
          vehicle.value("Tzero", 30.0),
          vehicle.value("Tnegmax", 250.0),
          vehicle.value("Pmax", 152e3),
          vehicle.value("Pnegmax", 100e3),
          vehicle.value("gear_ratio", 3.626 * 4.1),
          vehicle.value("brake_Tmax", 2e4),
          vehicle.value("brake_bias", 0.55),
          vehicle.value("drivetrain_type", RWD),
          tire_config,
          vehicle.value("CDx", 0.3),
          vehicle.value("CMz", 0.0),
          vehicle.value("Af", 1.775 * (1.425 - 0.1)),
          vehicle.value("rho", 1.225)
      );
  }
  catch (const std::exception& e) {
      std::cerr << "Error loading vehicle config: " << e.what() << std::endl;
      return Vehicle2D_STConfig(); // Return default config on error
  }
}
