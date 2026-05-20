#pragma once

#include <Eigen/Dense>

static constexpr int CartesianTDPADoF = 3;

class CartesianTDPALeader {
 public:
  using MatrixNd = Eigen::Matrix<double, CartesianTDPADoF, CartesianTDPADoF>;

  void init();

  void energyObserver(
      const double v[CartesianTDPADoF],
      const double f[CartesianTDPADoF],
      double deltaT);

  void energyController(
      double force[CartesianTDPADoF],
      const double velocity[CartesianTDPADoF],
      double E_F_in_delayed,
      double deltaT);

  double getInputPowerFlow() const { return P_L_in; }
  double getOutputPowerFlow() const { return P_L_out; }
  double getInputEnergyFlow() const { return E_L_in; }
  double getOutputEnergyFlow() const { return E_L_out; }
  double getDissipatedEnergyFlow() const { return E_diss; }
  double getAlpha() const { return alpha; }

  double alpha = 0.0;

  double E_L_in = 0.0;
  double E_L_out = 0.0;

  double P_L_in = 0.0;
  double P_L_out = 0.0;

  double E_F_in_delayed = 0.0;
  double E_diss = 0.0;

  double v_L_old[CartesianTDPADoF] = {0.0, 0.0, 0.0};
};

class CartesianTDPAFollower {
 public:
  using MatrixNd = Eigen::Matrix<double, CartesianTDPADoF, CartesianTDPADoF>;

  void init();

  // Kept for compatibility with existing controller code.
  // No Lambda scaling is used in the split Cartesian TDPA.
  void setCartesianMassMatrix(const MatrixNd& Lambda);

  void energyObserver(
      const double v[CartesianTDPADoF],
      const double f[CartesianTDPADoF],
      double deltaT);

  void energyController(
      double velocity[CartesianTDPADoF],
      const double force[CartesianTDPADoF],
      double E_L_in_delayed,
      double deltaT);

  double getInputPowerFlow() const { return P_F_in; }
  double getOutputPowerFlow() const { return P_F_out; }
  double getInputEnergyFlow() const { return E_F_in; }
  double getOutputEnergyFlow() const { return E_F_out; }
  double getDissipatedEnergyFlow() const { return E_diss; }
  double getBeta() const { return beta; }

  double beta = 0.0;

  double E_F_in = 0.0;
  double E_F_out = 0.0;

  double P_F_in = 0.0;
  double P_F_out = 0.0;

  double E_L_in_delayed = 0.0;
  double E_diss = 0.0;

  double f_F_old[CartesianTDPADoF] = {0.0, 0.0, 0.0};
};