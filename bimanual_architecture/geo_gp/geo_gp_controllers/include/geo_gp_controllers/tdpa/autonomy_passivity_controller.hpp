#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace geo_gp_controllers {

struct EnergyTankParameters {
  double initial_energy{2.0};
  double max_energy{5.0};
  double recharge_efficiency{0.8};
  double power_epsilon{1e-9};
  double velocity_epsilon{1e-6};
  double wrench_max_abs{100.0};
};

class EnergyTank {
 public:
  explicit EnergyTank(const EnergyTankParameters& parameters = EnergyTankParameters());
  void configure(const EnergyTankParameters& parameters);
  void reset();
  double energy() const;

  template <typename VectorType>
  VectorType update(const VectorType& f_autonomy,
                    const VectorType& velocity,
                    double dt,
                    double* lambda_out = nullptr,
                    double* power_out = nullptr) {
    VectorType bounded = f_autonomy;
    const double limit = std::max(0.0, finiteOrZero(parameters_.wrench_max_abs));
    for (int i = 0; i < bounded.size(); ++i) {
      bounded(i) = clampValue(finiteOrZero(bounded(i)), -limit, limit);
    }
    const double safe_dt = (std::isfinite(dt) && dt > 0.0) ? dt : 0.0;
    const double speed = velocity.allFinite() ? velocity.norm() : 0.0;
    const double power =
        (safe_dt > 0.0 && speed > parameters_.velocity_epsilon && velocity.allFinite())
            ? bounded.dot(velocity)
            : 0.0;
    double lambda = 1.0;
    if (power > parameters_.power_epsilon && safe_dt > 0.0) {
      const double demand = power * safe_dt;
      lambda = clampValue(energy_ / (demand + parameters_.power_epsilon), 0.0, 1.0);
      bounded *= lambda;
      energy_ -= lambda * demand;
    } else if (power < -parameters_.power_epsilon && safe_dt > 0.0) {
      const double eta = clampValue(finiteOrZero(parameters_.recharge_efficiency), 0.0, 1.0);
      energy_ += eta * (-power) * safe_dt;
    }
    energy_ =
        clampValue(finiteOrZero(energy_), 0.0, std::max(0.0, finiteOrZero(parameters_.max_energy)));
    if (lambda_out)
      *lambda_out = lambda;
    if (power_out)
      *power_out = power;
    return bounded;
  }

 private:
  static double finiteOrZero(double value);
  static double clampValue(double value, double low, double high);
  EnergyTankParameters parameters_;
  double energy_{0.0};
};

struct AutonomyPassivityParameters {
  EnergyTankParameters tank{};
  double alpha_time_constant{0.05};
  double alpha_rate_limit{2.0};
};

struct AutonomyPassivityOutput {
  Eigen::Matrix<double, 6, 1> wrench_autonomy_safe{Eigen::Matrix<double, 6, 1>::Zero()};
  double alpha{0.0};
  double lambda{1.0};
  double power{0.0};
  double tank_energy{0.0};
};

class AutonomyPassivityController {
 public:
  explicit AutonomyPassivityController(
      const AutonomyPassivityParameters& parameters = AutonomyPassivityParameters());
  void configure(const AutonomyPassivityParameters& parameters);
  void reset();
  double updateAlpha(double alpha_requested, double dt);
  AutonomyPassivityOutput update(const Eigen::Matrix<double, 6, 1>& wrench_leader,
                                 const Eigen::Matrix<double, 6, 1>& wrench_total,
                                 const Eigen::Matrix<double, 6, 1>& velocity,
                                 double dt);
  double alpha() const;
  double tankEnergy() const;

 private:
  AutonomyPassivityParameters parameters_;
  EnergyTank tank_;
  double alpha_{0.0};
};

}  // namespace geo_gp_controllers
