#include <OptSolver.hpp>

#define FORWARD_GRADIENT_ONLY 1
#define USE_NORMALIZED_GRADIENTS 1
#define USE_RK4 1

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
#if USE_RK4
    // using RK4
    // k1
    std::vector<double> x_tmp = x_trial;
    std::vector<double> k1 = Jacobian(x_tmp);
    gradients.resize(x_trial.size());

    // k2
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        x_tmp[i] = x_trial[i] + h * k1[i] / 2.0;
    }
    std::vector<double> k2 = Jacobian(x_tmp);

    // k3
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        x_tmp[i] = x_trial[i] + h * k2[i] / 2.0;
    }

    std::vector<double> k3 = Jacobian(x_tmp);

    // k4
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        x_tmp[i] = x_trial[i] + h * k3[i];
    }
    std::vector<double> k4 = Jacobian(x_tmp);

    // calculate gradients
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        gradients[i] = (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
    }
#else
    gradients = Jacobian(x_trial);
#endif
}

std::vector<double> OptSolver::Jacobian(const std::vector<double> x_trial)
{
    double f_curr = f(x_trial);
    std::vector<double> x_forward = x_trial;
    std::vector<double> J(x_trial.size());
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        const double orig = x_forward[i];
        x_forward[i] += h;
        J[i] = (f(x_forward) - f_curr) / h;
        x_forward[i] = orig; // restore original value
    }
    return J;
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

    // limit norm to avoid too large step size
    if (norm < 100.0)
    {
        norm = 100.0;
    }

    // normalize gradients
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        gradients[i] *= step_size / norm;
    }
#else
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        gradients[i] *= step_size;
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
        return false;
    }

    // optimize
    bool converged = false;
    uint n_iter = 0;
    while (!converged && n_iter < max_iters)
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
        if ((n_iter) % 1 == 0 || n_iter == 1)
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
