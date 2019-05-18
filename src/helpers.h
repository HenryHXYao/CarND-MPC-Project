#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <math.h>
using Eigen::VectorXd;
using std::string;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double derivativeEval(const VectorXd &coeffs, double x)
{
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); ++i)
  {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j)
  {
    for (int i = 0; i < order; ++i)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}

void transformToVehicle(double x, double y, double psi, std::vector<double> &x_vals, std::vector<double> &y_vals)
{
  for (int i = 0; i < x_vals.size(); ++i)
  {
    double x1 = x_vals[i] - x;
    double y1 = y_vals[i] - y;
    x_vals[i] = cos(psi) * x1 + sin(psi) * y1;
    y_vals[i] = -sin(psi) * x1 + cos(psi) * y1;
  }
}

#endif // HELPERS_H