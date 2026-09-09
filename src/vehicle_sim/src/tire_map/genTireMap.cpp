#include "genTireMap.hpp"

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("../configs/toyota_86.json");
    TireConfig &tire_config = config.getTireConfig();

    // open file
    const std::string filename = "./tire_map.csv";
    std::ofstream file;
    file = std::ofstream(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    // write header
    file << "slip_angle [rad],slip_ratio [-],Fx [N],Fy [N],Fz [N],w [rad/s]\n";

    // generate tire map
    double slip_angles_lb = - M_PI / 2.0;
    double slip_angles_ub = M_PI / 2.0;
    double slip_angles_step = 0.01;
    double slip_ratios_lb = -1.0;
    double slip_ratios_ub = 1.0;
    double slip_ratios_step = 0.01;
    std::vector<double> slip_angles = {slip_angles_lb, slip_angles_step, slip_angles_ub};
    std::vector<double> slip_ratios = {slip_ratios_lb, slip_ratios_step, slip_ratios_ub};

    double vx = 5.0; // [m/s] vehicle speed
    double w_wheel = vx / config.getWheelRadius(); // [rad/s] wheel speed
    double Fz_wheel = 1.0; // [N] normal force

    std::vector<std::vector<double>> slips = combinations({slip_angles, slip_ratios});

    for (uint i = 0; i < slips.size(); ++i)
    {
        const double slip_angle = slips[i][0];
        const double slip_ratio = slips[i][1];

        // calculate tire forces
        double Fx_wheel, Fy_wheel;

        tire_config.calcTireForces(slip_angle, slip_ratio, w_wheel, Fz_wheel, Fx_wheel, Fy_wheel);

        // write to file
        file << slip_angle << "," << slip_ratio << "," << Fx_wheel << "," << Fy_wheel << "," << Fz_wheel << "," << w_wheel << "\n";
    }
}
