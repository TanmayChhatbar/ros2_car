#include "vehicle_2d.hpp"
#include "vehicle_2d_csvWriter.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

int main()
{
    // default configuration
    Vehicle vehicle = Vehicle();

    // prepare output CSV file
    std::string filename = "vehicle_sim_test.csv";
    std::ofstream output_file(filename);

    if (!output_file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 1;
    }
    writeCSVHeader(output_file);

    // run simulation
    double current_time = 0.0;
    int step_count = 0;

    std::cout << "Running test..." << std::endl;
    double throttle_input = 0.0; // [-1 to 1]
    double steering_input = 0.0; // [-1 to 1]
    double dt = 0.001;           // [s]
    double tmax = 20.0;          // [s]
    double freq_steer = 0.5;     // frequency of steering oscillation

    for (int step = 0; step < (int)(tmax / dt); step++)
    {
        // calculate inputs based on current time
        steering_input = std::sin(freq_steer * current_time / 2 / M_PI);
        throttle_input = 1.0; // constant throttle

        // set inputs and step simulation
        vehicle.stepSimulation(dt, steering_input, throttle_input);

        // output data every 10 steps (reduces file size)
        if (step_count % 10 == 0)
        {
            writeCSVData(output_file, current_time, vehicle.getVehicleData(), vehicle.getVehicleInput());
        }

        current_time += dt;
        step_count++;
    }

    output_file.close();
    std::cout << "Finished test. Data saved to " << filename << std::endl;

    return 0;
}
