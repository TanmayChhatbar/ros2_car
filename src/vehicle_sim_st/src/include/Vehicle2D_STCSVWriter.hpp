#pragma once

#include "Vehicle2D_STData.hpp"
#include "Vehicle2D_STConfig.hpp"
#include "VehicleInput.hpp"
#include <iostream>
#include <fstream>

// cSV header and data formatting
void writeCSVHeader(std::ofstream &file);

void writeCSVData(std::ofstream &file, double time, const Vehicle2D_STData &data,
                  const VehicleInput &input);

void exportVehicle2D_STConfig(Vehicle2D_STConfig &Vehicle2D_STConfig, std::string filename);
