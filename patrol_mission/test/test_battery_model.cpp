#include <gtest/gtest.h>

#include <stdexcept>

#include "patrol_mission/battery_model.hpp"

using patrol_mission::BatteryConfig;
using patrol_mission::BatteryModel;

namespace
{

BatteryConfig makeConfig()
{
  BatteryConfig config;
  config.initial_level = 50.0;
  config.drain_rate_moving = 1.0;
  config.drain_rate_idle = 0.1;
  config.charge_rate = 10.0;
  config.low_threshold = 20.0;
  config.resume_threshold = 90.0;
  return config;
}

}  // namespace

TEST(BatteryModel, DrainsFasterWhenMoving)
{
  BatteryModel moving(makeConfig());
  BatteryModel idle(makeConfig());
  moving.drain(10.0, true);
  idle.drain(10.0, false);
  EXPECT_DOUBLE_EQ(moving.level(), 40.0);
  EXPECT_DOUBLE_EQ(idle.level(), 49.0);
}

TEST(BatteryModel, NeverDropsBelowZero)
{
  BatteryModel battery(makeConfig());
  battery.drain(1000.0, true);
  EXPECT_DOUBLE_EQ(battery.level(), 0.0);
}

TEST(BatteryModel, NeverChargesAboveHundred)
{
  BatteryModel battery(makeConfig());
  battery.charge(1000.0);
  EXPECT_DOUBLE_EQ(battery.level(), 100.0);
}

TEST(BatteryModel, IgnoresNonPositiveTimeSteps)
{
  BatteryModel battery(makeConfig());
  battery.drain(-5.0, true);
  battery.charge(0.0);
  EXPECT_DOUBLE_EQ(battery.level(), 50.0);
}

TEST(BatteryModel, ReportsThresholds)
{
  BatteryModel battery(makeConfig());
  EXPECT_FALSE(battery.isLow());
  EXPECT_FALSE(battery.isCharged());

  battery.drain(30.0, true);  // 50 -> 20
  EXPECT_TRUE(battery.isLow());

  battery.charge(7.0);  // 20 -> 90
  EXPECT_TRUE(battery.isCharged());
}

TEST(BatteryModel, RejectsInvalidConfiguration)
{
  BatteryConfig inverted = makeConfig();
  inverted.low_threshold = 95.0;
  EXPECT_THROW(BatteryModel{inverted}, std::invalid_argument);

  BatteryConfig out_of_range = makeConfig();
  out_of_range.initial_level = 150.0;
  EXPECT_THROW(BatteryModel{out_of_range}, std::invalid_argument);

  BatteryConfig no_charge = makeConfig();
  no_charge.charge_rate = 0.0;
  EXPECT_THROW(BatteryModel{no_charge}, std::invalid_argument);
}
