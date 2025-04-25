#pragma once

#include <iostream>
#include <fstream>

// cSV header and data formatting
void writeCSVHeader(std::ofstream &file);

void writeCSVData(std::ofstream &file, double time, const VehicleData &data,
                  const VehicleInput &input);

void exportVehicleConfig(const VehicleConfig &VehicleConfig);
