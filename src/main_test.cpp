#include <vector>
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "matplotlibcpp.h"
#include "MPC.h"

namespace plt = matplotlibcpp;

using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

int main() {
  MPC mpc;
  int iters = 50;

  VectorXd ptsx(2);
  VectorXd ptsy(2);
  ptsx << -100, 100;
  ptsy << -1, -1;

  /**
   * TODO: fit a polynomial to the above x and y coordinates
   */
  auto coeffs =  polyfit(ptsx, ptsy, 1) ;

  // NOTE: free feel to play around with these
  double x = -1;
  double y = 10;
  double psi = 0;
  double v = 10;
  /**
   * TODO: calculate the cross track error
   */
  double cte = polyeval(coeffs, x) - y ;
  /**
   * TODO: calculate the orientation error
   */
  double epsi = psi - atan(coeffs[1]);

  VectorXd state(6);
  state << x, y, psi, v, cte, epsi;

  vector<double> x_vals = {state[0]};
  vector<double> y_vals = {state[1]};
  vector<double> psi_vals = {state[2]};
  vector<double> v_vals = {state[3]};
  vector<double> cte_vals = {state[4]};
  vector<double> epsi_vals = {state[5]};
  vector<double> delta_vals = {};
  vector<double> a_vals = {};

  for (size_t i = 0; i < iters; ++i) {
    cout << "Iteration " << i << endl;

    auto solution = mpc.Solve(state, coeffs);

    x_vals.push_back(solution.x[1]);
    y_vals.push_back(solution.y[1]);
    psi_vals.push_back(solution.psi[1]);
    v_vals.push_back(solution.v[1]);
    cte_vals.push_back(solution.cte[1]);
    epsi_vals.push_back(solution.epsi[1]);

    delta_vals.push_back(solution.delta[0]);
    a_vals.push_back(solution.a[0]);

    state << solution.x[1], solution.y[1], solution.psi[1], solution.v[1], solution.cte[1], solution.epsi[1];
    cout << "x = " << solution.x[1] << endl;
    cout << "y = " << solution.y[1] << endl;
    cout << "psi = " << solution.psi[1] << endl;
    cout << "v = " << solution.v[1] << endl;
    cout << "cte = " << solution.cte[1] << endl;
    cout << "epsi = " << solution.epsi[1] << endl;
    cout << "delta = " << solution.delta[0] << endl;
    cout << "a = " << solution.a[0] << endl;
    cout << endl;
  }

  // Plot values
  // NOTE: feel free to play around with this.
  // It's useful for debugging!
  plt::subplot(3, 1, 1);
  plt::title("CTE");
  plt::plot(cte_vals);
  plt::subplot(3, 1, 2);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  plt::subplot(3, 1, 3);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::show();
}