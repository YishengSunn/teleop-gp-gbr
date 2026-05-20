#pragma once

#define DoF 7

class TDPA_Leader {
 public:
  void init();

  void energyObserver(double v[DoF], double f[DoF], double deltaT);

  void energyController(double force[DoF],
                        double velocity[DoF],
                        double E_F_in_delayed,
                        double deltaT);

  double getInputPowerFlow() { return P_L_in; }
  double getOutputPowerFlow() { return P_L_out; }
  double getInputEnergyFlow() { return E_L_in; }
  double getOutputEnergyFlow() { return E_L_out; }
  double getDissipatedEnergyFlow() { return E_diss; }
  double getAlpha() { return alpha; }

  double alpha = 0.0;

  double E_L_in = 0.0;
  double E_L_out = 0.0;

  double P_L_in = 0.0;
  double P_L_out = 0.0;

  double E_F_in_delayed = 0.0;
  double E_diss = 0.0;

  double v_L_old[DoF] = {0.0};
};

class TDPA_Follower {
 public:
  void init();

  void energyObserver(double v[DoF], double f[DoF], double deltaT);

  void energyController(double velocity[DoF],
                        double force[DoF],
                        double E_L_in_delayed,
                        double deltaT);

  double getInputPowerFlow() { return P_F_in; }
  double getOutputPowerFlow() { return P_F_out; }
  double getInputEnergyFlow() { return E_F_in; }
  double getOutputEnergyFlow() { return E_F_out; }
  double getDissipatedEnergyFlow() { return E_diss; }
  double getBeta() { return beta; }

  double beta = 0.0;

  double E_F_in = 0.0;
  double E_F_out = 0.0;

  double P_F_in = 0.0;
  double P_F_out = 0.0;

  double E_L_in_delayed = 0.0;
  double E_diss = 0.0;

  double f_F_old[DoF] = {0.0};
};