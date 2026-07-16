#include "geo_gp_controllers/tdpa/autonomy_passivity_controller.hpp"

#include <algorithm>
#include <cmath>

namespace geo_gp_controllers {

EnergyTank::EnergyTank(const EnergyTankParameters& parameters) : parameters_(parameters) {
  reset();
}
void EnergyTank::configure(const EnergyTankParameters& parameters) {
  parameters_ = parameters;
  reset();
}
void EnergyTank::reset() {
  energy_ = clampValue(finiteOrZero(parameters_.initial_energy), 0.0,
                       std::max(0.0, finiteOrZero(parameters_.max_energy)));
}
double EnergyTank::energy() const {
  return energy_;
}
double EnergyTank::finiteOrZero(double value) {
  return std::isfinite(value) ? value : 0.0;
}
double EnergyTank::clampValue(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

AutonomyPassivityController::AutonomyPassivityController(
    const AutonomyPassivityParameters& parameters)
    : parameters_(parameters), tank_(parameters.tank) {}
void AutonomyPassivityController::configure(const AutonomyPassivityParameters& parameters) {
  parameters_ = parameters;
  tank_.configure(parameters.tank);
  alpha_ = 0.0;
}
void AutonomyPassivityController::reset() {
  tank_.reset();
  alpha_ = 0.0;
}
double AutonomyPassivityController::updateAlpha(double alpha_requested, double dt) {
  const double requested =
      std::max(0.0, std::min(std::isfinite(alpha_requested) ? alpha_requested : 0.0, 1.0));
  const double safe_dt = (std::isfinite(dt) && dt > 0.0) ? dt : 0.0;
  double target = requested;
  if (parameters_.alpha_time_constant > 0.0 && safe_dt > 0.0) {
    const double beta = safe_dt / (parameters_.alpha_time_constant + safe_dt);
    target = alpha_ + beta * (requested - alpha_);
  }
  if (parameters_.alpha_rate_limit > 0.0 && safe_dt > 0.0) {
    const double delta = parameters_.alpha_rate_limit * safe_dt;
    target = std::max(alpha_ - delta, std::min(target, alpha_ + delta));
  }
  alpha_ = std::max(0.0, std::min(target, 1.0));
  return alpha_;
}
AutonomyPassivityOutput AutonomyPassivityController::update(
    const Eigen::Matrix<double, 6, 1>& wrench_leader,
    const Eigen::Matrix<double, 6, 1>& wrench_total,
    const Eigen::Matrix<double, 6, 1>& velocity,
    double dt) {
  AutonomyPassivityOutput output;
  output.alpha = alpha_;
  const Eigen::Matrix<double, 6, 1> f_autonomy = wrench_total - wrench_leader;
  output.wrench_autonomy_safe =
      tank_.update(f_autonomy, velocity, dt, &output.lambda, &output.power);
  output.tank_energy = tank_.energy();
  return output;
}
double AutonomyPassivityController::alpha() const {
  return alpha_;
}
double AutonomyPassivityController::tankEnergy() const {
  return tank_.energy();
}

}  // namespace geo_gp_controllers
