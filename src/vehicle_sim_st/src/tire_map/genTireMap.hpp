#include "Vehicle2D.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <chrono>
#include <nlopt.hpp>
#include "CSVWriter.hpp"
#include "CSVReader.hpp"

std::vector<std::vector<double>> combinations(const std::vector<std::vector<double>> &values)
{
    // values is a vector of vectors, where each inner vector contains lower bound, step size, upper bound for that dimension
    std::vector<std::vector<double>> trial_points;
    std::vector<double> lb_current;
    std::vector<double> ub_current;
    std::vector<double> step_size;
    for (uint i = 0; i < values.size(); ++i)
    {
        lb_current.push_back(values[i][0]);
        step_size.push_back(values[i][1]);
        ub_current.push_back(values[i][2]);
    }

    // generate all combinations of trial points
    std::vector<double> trial_point_cur = lb_current;
    trial_points.push_back(trial_point_cur);
    while (true)
    {
        trial_point_cur[0] += step_size[0];
        for (size_t i = 0; i < (lb_current.size() - 1); ++i)
        {
            if (trial_point_cur[i] > ub_current[i])
            {
                trial_point_cur[i] = lb_current[i];
                trial_point_cur[i + 1] += step_size[i + 1];
            }
        }
        if (trial_point_cur.back() > ub_current.back())
        {
            break; // all combinations generated
        }
        trial_points.push_back(trial_point_cur);
    }
    return trial_points;
}
