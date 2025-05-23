#pragma once

#include <vector>
#include <functional>
#include <iostream>
#include <cmath>

class OptSolver
{
public:
    OptSolver();
    OptSolver(std::function<double(std::vector<double>)> f_);
    bool solve();
    double getScore();
    std::vector<double> getSolution();
    void setFunction(std::function<double(std::vector<double>)> f_);
    void setGuess(std::vector<double> x0_);

private:
    void calcGradients();
    bool calcNextBestTrial();
    double score = 1.0e10;
    double threshold = 1e-2;
    bool converged = false;
    std::function<double(std::vector<double>)> f; // optimization function
    std::vector<double> x_trial;                  // trial point
    std::vector<double> lb;                       // lower bounds
    std::vector<double> ub;                       // upper bounds
    std::vector<double> gradients;                // gradients
    std::vector<double> x_opt;                    // optimal point
};
