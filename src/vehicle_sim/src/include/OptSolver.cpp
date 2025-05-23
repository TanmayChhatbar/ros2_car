#include <OptSolver.hpp>

#define FORWARD_GRADIENT_ONLY 1
#define USE_NORMALIZED_GRADIENTS 0

OptSolver::OptSolver()
{
    OptSolver(nullptr);
}

OptSolver::OptSolver(std::function<double(std::vector<double>)> f_)
{
    f = f_;
    score = 0.0;
    x_opt.clear();
    x_trial.clear();
    gradients.clear();
}

void OptSolver::setFunction(std::function<double(std::vector<double>)> f_)
{
    f = f_;
}

void OptSolver::calcGradients()
{
    // Numerical gradient calculation (finite differences)
    const double h = 1e-6;
    gradients.resize(x_trial.size());
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        std::vector<double> x_forward = x_trial;
        x_forward[i] += h;
        double f_forward = f(x_forward);

#if (FORWARD_GRADIENT_ONLY)
        gradients[i] = (f_forward - f(x_trial)) / h;
#else
        std::vector<double> x_backward = x_trial;
        x_backward[i] -= h;
        double f_backward = f(x_backward);
        gradients[i] = (f_forward - f_backward) / (2 * h);
#endif
    }
}

bool OptSolver::calcNextBestTrial()
{
    // calculate total slope
#if USE_NORMALIZED_GRADIENTS
    double norm = 0.0;
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        norm += gradients[i] * gradients[i];
    }
    norm = std::sqrt(norm);

    // if gradients very small, near a local optimum
    if (norm < threshold)
    {
        return true;
    }

    // normalize gradients
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        gradients[i] /= norm;
    }
#else
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        gradients[i] = gradients[i] / 10.0;
    }
#endif

    // calculate next best trial point
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        x_trial[i] -= gradients[i];
    }
    return false;
}

bool OptSolver::solve()
{
    if (f == nullptr)
    {
        std::cerr << "Error: Objective function not set." << std::endl;
        return 0.0;
    }

    // optimize
    bool converged = false;
    uint n_iter = 0;
    while (!converged && n_iter < 1000)
    {
        // calculate score
        calcGradients();

        converged = calcNextBestTrial();
        double new_score = f(x_trial);
        if (new_score < score)
        {
            score = new_score;
            x_opt = x_trial;
        }

        n_iter++;
        if (n_iter % 10 == 0)
        {
            std::cout << n_iter << "\t";
            std::cout << score << "\t";
            for (size_t i = 0; i < x_trial.size() - 1; ++i)
            {
                std::cout << x_trial[i] << "\t";
            }
            std::cout << x_trial[x_trial.size() - 1] << std::endl;
        }
    }
    return converged;
}

std::vector<double> OptSolver::getSolution(void)
{
    return x_opt;
}

double OptSolver::getScore()
{
    return score;
}

void OptSolver::setGuess(std::vector<double> x0_)
{
    x_trial = x0_;
    x_opt = x0_;
    score = f(x_trial);
}
