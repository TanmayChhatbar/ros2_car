#include <OptSolver.hpp>

double f(std::vector<double> x)
{
    return x[0] * x[0] * 480.0 + (x[1] - 100.0) * (x[1] - 10.0) + 500.0;
}

int main()
{
    OptSolver solver(f);
    std::vector<double> x0 = {1.0, 2.0};

    solver.setGuess(x0);
    if (!solver.solve())
    {
        std::cerr << "Solver did not converge." << std::endl;
        return 1;
    }

    std::vector<double> x_opt = solver.getSolution();
    std::cout << "Optimal solution: "
              << x_opt[0] << ", " << x_opt[1] << std::endl;
    std::cout << "Objective function value: "
              << solver.getScore() << std::endl;
}
