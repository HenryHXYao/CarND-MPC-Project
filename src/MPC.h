#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
using std::vector;

struct Solution
{
vector<double> x;
vector<double> y;
double delta;
double a;
};

class MPC
{
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    Solution Solve(const Eigen::VectorXd &state,
                              const Eigen::VectorXd &coeffs);
};

#endif // MPC_H
