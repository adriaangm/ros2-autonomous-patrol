#ifndef PATROL_MISSION__BATTERY_MODEL_HPP_
#define PATROL_MISSION__BATTERY_MODEL_HPP_

#include <algorithm>
#include <stdexcept>

namespace patrol_mission
{

/// Configuration of the simulated battery. All levels are percentages [0, 100].
struct BatteryConfig
{
  double initial_level{100.0};    ///< Level at startup [%]
  double drain_rate_moving{0.5};  ///< Discharge while navigating [%/s]
  double drain_rate_idle{0.05};   ///< Discharge while stopped [%/s]
  double charge_rate{5.0};        ///< Charge while docked [%/s]
  double low_threshold{25.0};     ///< At or below this level the robot must return to dock [%]
  double resume_threshold{95.0};  ///< At or above this level the robot may resume patrolling [%]
};

/// Simple, ROS-agnostic battery model. Kept free of ROS types so it can be unit tested in isolation.
class BatteryModel
{
public:
  BatteryModel()
  : BatteryModel(BatteryConfig{}) {}

  explicit BatteryModel(const BatteryConfig & config)
  : config_(config), level_(config.initial_level)
  {
    validate(config_);
  }

  void drain(double dt_s, bool moving)
  {
    if (dt_s <= 0.0) {
      return;
    }
    const double rate = moving ? config_.drain_rate_moving : config_.drain_rate_idle;
    level_ = std::clamp(level_ - rate * dt_s, 0.0, 100.0);
  }

  void charge(double dt_s)
  {
    if (dt_s <= 0.0) {
      return;
    }
    level_ = std::clamp(level_ + config_.charge_rate * dt_s, 0.0, 100.0);
  }

  double level() const {return level_;}
  bool isLow() const {return level_ <= config_.low_threshold;}
  bool isCharged() const {return level_ >= config_.resume_threshold;}
  const BatteryConfig & config() const {return config_;}

private:
  static void validate(const BatteryConfig & c)
  {
    const auto in_range = [](double v) {return v >= 0.0 && v <= 100.0;};
    if (!in_range(c.initial_level) || !in_range(c.low_threshold) || !in_range(c.resume_threshold)) {
      throw std::invalid_argument("Battery levels and thresholds must be within [0, 100]");
    }
    if (c.low_threshold >= c.resume_threshold) {
      throw std::invalid_argument("Battery low_threshold must be lower than resume_threshold");
    }
    if (c.drain_rate_moving < 0.0 || c.drain_rate_idle < 0.0 || c.charge_rate <= 0.0) {
      throw std::invalid_argument(
              "Battery drain rates must be non-negative and charge_rate must be positive");
    }
  }

  BatteryConfig config_;
  double level_;
};

}  // namespace patrol_mission

#endif  // PATROL_MISSION__BATTERY_MODEL_HPP_
