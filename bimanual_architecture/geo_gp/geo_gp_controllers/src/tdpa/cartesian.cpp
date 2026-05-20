#include "geo_gp_controllers/tdpa/cartesian.h"

#include <cmath>

namespace {

inline bool isFinite(double x) {
  return std::isfinite(x);
}

inline void innerProduct(
    const double a[CartesianTDPADoF],
    const double b[CartesianTDPADoF],
    double& dest) {
  dest = 0.0;
  for (int i = 0; i < CartesianTDPADoF; ++i) {
    dest += a[i] * b[i];
  }
}

inline void resetSamples(double* x, int size) {
  for (int i = 0; i < size; ++i) {
    x[i] = 0.0;
  }
}

inline double squaredNorm(const double x[CartesianTDPADoF]) {
  double out = 0.0;
  for (int i = 0; i < CartesianTDPADoF; ++i) {
    out += x[i] * x[i];
  }
  return out;
}

}  // namespace

void CartesianTDPALeader::init() {
  resetSamples(&alpha, 1);

  resetSamples(&E_L_in, 1);
  resetSamples(&E_L_out, 1);

  resetSamples(&P_L_in, 1);
  resetSamples(&P_L_out, 1);

  resetSamples(&E_F_in_delayed, 1);
  resetSamples(&E_diss, 1);

  resetSamples(v_L_old, CartesianTDPADoF);
}

void CartesianTDPALeader::energyObserver(
    const double v[CartesianTDPADoF],
    const double f[CartesianTDPADoF],
    double deltaT) {
  if (!isFinite(deltaT) || deltaT <= 0.0) {
    P_L_in = 0.0;
    P_L_out = 0.0;
    return;
  }

  double power = 0.0;
  innerProduct(v, f, power);

  if (!isFinite(power)) {
    P_L_in = 0.0;
    P_L_out = 0.0;
    return;
  }

  if (power > 0.0) {
    P_L_in = power;
    P_L_out = 0.0;
  } else {
    P_L_in = 0.0;
    P_L_out = -power;
  }

  E_L_in += P_L_in * deltaT;
  E_L_out += P_L_out * deltaT;
}

void CartesianTDPALeader::energyController(
    double force[CartesianTDPADoF],
    const double velocity[CartesianTDPADoF],
    double E_F_in_delayed_in,
    double deltaT) {
  alpha = 0.0;

  if (!isFinite(deltaT) || deltaT <= 0.0) {
    return;
  }

  E_F_in_delayed = E_F_in_delayed_in;

  const double E_pc = E_F_in_delayed - E_L_out + E_diss;
  if (!isFinite(E_pc) || E_pc >= 0.0) {
    return;
  }

  const double velocity_length = squaredNorm(velocity);
  if (!isFinite(velocity_length) || velocity_length <= 1e-6) {
    return;
  }

  alpha = -E_pc / (deltaT * velocity_length);

  double dissipated_power = 0.0;
  for (int i = 0; i < CartesianTDPADoF; ++i) {
    const double f_pc_i = alpha * velocity[i];
    force[i] += f_pc_i;
    dissipated_power += velocity[i] * f_pc_i;
  }

  E_diss += deltaT * dissipated_power;
}

void CartesianTDPAFollower::init() {
  resetSamples(&beta, 1);

  resetSamples(&E_F_in, 1);
  resetSamples(&E_F_out, 1);

  resetSamples(&P_F_in, 1);
  resetSamples(&P_F_out, 1);

  resetSamples(&E_L_in_delayed, 1);
  resetSamples(&E_diss, 1);

  resetSamples(f_F_old, CartesianTDPADoF);
}

void CartesianTDPAFollower::energyObserver(
    const double v[CartesianTDPADoF],
    const double f[CartesianTDPADoF],
    double deltaT) {
  if (!isFinite(deltaT) || deltaT <= 0.0) {
    P_F_in = 0.0;
    P_F_out = 0.0;
    return;
  }

  double power = 0.0;
  innerProduct(v, f, power);

  if (!isFinite(power)) {
    P_F_in = 0.0;
    P_F_out = 0.0;
    return;
  }

  if (power > 0.0) {
    P_F_in = power;
    P_F_out = 0.0;
  } else {
    P_F_in = 0.0;
    P_F_out = -power;
  }

  E_F_in += P_F_in * deltaT;
  E_F_out += P_F_out * deltaT;
}

void CartesianTDPAFollower::energyController(
    double velocity[CartesianTDPADoF],
    const double force[CartesianTDPADoF],
    double E_L_in_delayed_in,
    double deltaT) {
  beta = 0.0;

  if (!isFinite(deltaT) || deltaT <= 0.0) {
    return;
  }

  E_L_in_delayed = E_L_in_delayed_in;

  const double E_pc = E_L_in_delayed - E_F_out + E_diss;
  if (!isFinite(E_pc) || E_pc >= 0.0) {
    return;
  }

  const double force_length = squaredNorm(force);
  if (!isFinite(force_length) || force_length <= 1e-6) {
    return;
  }

  beta = -E_pc / (deltaT * force_length);

  double dissipated_power = 0.0;
  for (int i = 0; i < CartesianTDPADoF; ++i) {
    const double v_pc_i = beta * force[i];
    velocity[i] += v_pc_i;
    dissipated_power += force[i] * v_pc_i;
  }

  E_diss += deltaT * dissipated_power;
}