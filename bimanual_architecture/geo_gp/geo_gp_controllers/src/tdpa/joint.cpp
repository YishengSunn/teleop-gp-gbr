#include "geo_gp_controllers/tdpa/joint.h"

#include <cmath>

namespace {

constexpr double kMinNorm = 1.0e-6;

inline bool isFinite(double x) {
  return std::isfinite(x);
}

inline void innerProduct(const double a[DoF], const double b[DoF], double& dest) {
  dest = 0.0;
  for (int i = 0; i < DoF; ++i) {
    dest += a[i] * b[i];
  }
}

inline void resetSamples(double* x, int size) {
  for (int i = 0; i < size; ++i) {
    x[i] = 0.0;
  }
}

inline double squaredNorm(const double x[DoF]) {
  double out = 0.0;
  for (int i = 0; i < DoF; ++i) {
    out += x[i] * x[i];
  }
  return out;
}

}  // namespace

void TDPA_Leader::init() {
  resetSamples(&alpha, 1);

  resetSamples(&E_L_in, 1);
  resetSamples(&E_L_out, 1);

  resetSamples(&P_L_in, 1);
  resetSamples(&P_L_out, 1);

  resetSamples(&E_F_in_delayed, 1);
  resetSamples(&E_diss, 1);

  resetSamples(v_L_old, DoF);
}

void TDPA_Leader::energyObserver(double v[DoF], double f[DoF], double deltaT) {
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

void TDPA_Leader::energyController(double force[DoF],
                                   double velocity[DoF],
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
  if (!isFinite(velocity_length) || velocity_length <= kMinNorm) {
    return;
  }

  alpha = -E_pc / (deltaT * velocity_length);

  double dissipated_power = 0.0;
  for (int i = 0; i < DoF; ++i) {
    const double tau_pc_i = alpha * velocity[i];
    force[i] += tau_pc_i;
    dissipated_power += velocity[i] * tau_pc_i;
  }

  E_diss += deltaT * dissipated_power;
}

void TDPA_Follower::init() {
  resetSamples(&beta, 1);

  resetSamples(&E_F_in, 1);
  resetSamples(&E_F_out, 1);

  resetSamples(&P_F_in, 1);
  resetSamples(&P_F_out, 1);

  resetSamples(&E_L_in_delayed, 1);
  resetSamples(&E_diss, 1);

  resetSamples(f_F_old, DoF);
}

void TDPA_Follower::energyObserver(double v[DoF], double f[DoF], double deltaT) {
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

void TDPA_Follower::energyController(double velocity[DoF],
                                     double force[DoF],
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
  if (!isFinite(force_length) || force_length <= kMinNorm) {
    return;
  }

  beta = -E_pc / (deltaT * force_length);

  double dissipated_power = 0.0;
  for (int i = 0; i < DoF; ++i) {
    const double dq_pc_i = beta * force[i];
    velocity[i] += dq_pc_i;
    dissipated_power += force[i] * dq_pc_i;
  }

  E_diss += deltaT * dissipated_power;
}