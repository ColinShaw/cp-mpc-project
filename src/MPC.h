#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  double steering_angle;
  double throttle;
  vector<double> mpc_x;
  vector<double> mpc_y;

  MPC();
  virtual ~MPC();
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
