#include "patrol_mission/patrol_manager.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace patrol_mission
{

using namespace std::chrono_literals;

std::string toString(MissionState state)
{
  switch (state) {
    case MissionState::kWaitingForNav2:
      return "WAITING_FOR_NAV2";
    case MissionState::kIdle:
      return "IDLE";
    case MissionState::kPatrolling:
      return "PATROLLING";
    case MissionState::kReturningToDock:
      return "RETURNING_TO_DOCK";
    case MissionState::kCharging:
      return "CHARGING";
    case MissionState::kFinished:
      return "FINISHED";
  }
  return "UNKNOWN";
}

PatrolManager::PatrolManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("patrol_manager", options)
{
  loadParameters();

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Transient local: late subscribers (e.g. a dashboard) immediately receive the current state.
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "patrol/state", rclcpp::QoS(1).reliable().transient_local());
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("patrol/battery_state", 10);

  start_srv_ = create_service<Trigger>(
    "patrol/start",
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
      handleStart(*response);
    });
  stop_srv_ = create_service<Trigger>(
    "patrol/stop",
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
      handleStop(*response);
    });

  const rclcpp::Time now_time = now();
  next_attempt_time_ = now_time;
  last_tick_time_ = now_time;

  timer_ = create_wall_timer(100ms, [this]() {onTick();});

  publishState();
  RCLCPP_INFO(
    get_logger(), "Loaded %zu waypoints in frame '%s'. Dock at (%.2f, %.2f). Autostart: %s",
    waypoints_.size(), goal_frame_.c_str(), dock_.x, dock_.y, autostart_ ? "yes" : "no");
}

// ---------------------------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------------------------

void PatrolManager::loadParameters()
{
  autostart_ = declare_parameter<bool>("autostart", true);
  loop_patrol_ = declare_parameter<bool>("loop_patrol", true);
  max_loops_ = static_cast<int>(declare_parameter<int64_t>("max_loops", 0));
  max_retries_ = static_cast<int>(declare_parameter<int64_t>("max_retries", 2));
  retry_delay_s_ = declare_parameter<double>("retry_delay", 3.0);
  goal_frame_ = declare_parameter<std::string>("goal_frame", "map");

  if (max_loops_ < 0 || max_retries_ < 0 || retry_delay_s_ < 0.0) {
    throw std::invalid_argument("max_loops, max_retries and retry_delay must be non-negative");
  }

  const auto names = declare_parameter<std::vector<std::string>>(
    "waypoint_names", std::vector<std::string>{});
  if (names.empty()) {
    throw std::invalid_argument("Parameter 'waypoint_names' must contain at least one waypoint");
  }

  waypoints_.clear();
  waypoints_.reserve(names.size());
  for (const auto & name : names) {
    waypoints_.push_back(loadPose("waypoints." + name, name));
  }
  dock_ = loadPose("dock_pose", "dock");

  BatteryConfig config;
  config.initial_level = declare_parameter<double>("battery.initial_level", config.initial_level);
  config.drain_rate_moving =
    declare_parameter<double>("battery.drain_rate_moving", config.drain_rate_moving);
  config.drain_rate_idle =
    declare_parameter<double>("battery.drain_rate_idle", config.drain_rate_idle);
  config.charge_rate = declare_parameter<double>("battery.charge_rate", config.charge_rate);
  config.low_threshold = declare_parameter<double>("battery.low_threshold", config.low_threshold);
  config.resume_threshold =
    declare_parameter<double>("battery.resume_threshold", config.resume_threshold);
  battery_ = BatteryModel(config);  // Throws std::invalid_argument on inconsistent values
}

Waypoint PatrolManager::loadPose(const std::string & parameter_name, const std::string & label)
{
  const auto values = declare_parameter<std::vector<double>>(parameter_name, std::vector<double>{});
  if (values.size() != 3U) {
    throw std::invalid_argument(
            "Parameter '" + parameter_name + "' must be [x, y, yaw] (got " +
            std::to_string(values.size()) + " values)");
  }
  return Waypoint{label, values[0], values[1], values[2]};
}

// ---------------------------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------------------------

void PatrolManager::onTick()
{
  const rclcpp::Time now_time = now();

  double dt_s = 0.0;
  if (has_last_tick_) {
    dt_s = (now_time - last_tick_time_).seconds();
  }
  last_tick_time_ = now_time;
  has_last_tick_ = true;
  if (dt_s < 0.0 || dt_s > 1.0) {
    dt_s = 0.0;  // Simulation clock just started, was reset or was paused
  }

  if (state_ == MissionState::kCharging) {
    battery_.charge(dt_s);
  } else {
    battery_.drain(dt_s, goal_active_);
  }
  publishBattery(now_time);

  switch (state_) {
    case MissionState::kWaitingForNav2:
      if (nav_client_->action_server_is_ready()) {
        RCLCPP_INFO(get_logger(), "Nav2 action server 'navigate_to_pose' is available");
        next_attempt_time_ = now_time;
        transitionTo(autostart_ ? MissionState::kPatrolling : MissionState::kIdle);
      }
      break;

    case MissionState::kPatrolling:
      if (battery_.isLow()) {
        RCLCPP_WARN(
          get_logger(), "Battery low (%.1f%%): interrupting patrol at '%s', returning to dock",
          battery_.level(), waypoints_[current_index_].name.c_str());
        // The dock goal preempts the active goal inside bt_navigator; the old result is ignored.
        invalidateActiveGoal();
        retries_ = 0;
        next_attempt_time_ = now_time;
        transitionTo(MissionState::kReturningToDock);
        break;
      }
      if (!goal_active_ && now_time >= next_attempt_time_) {
        sendGoal(waypoints_[current_index_]);
      }
      break;

    case MissionState::kReturningToDock:
      if (!goal_active_ && now_time >= next_attempt_time_) {
        sendGoal(dock_);
      }
      break;

    case MissionState::kCharging:
      if (battery_.isCharged()) {
        RCLCPP_INFO(
          get_logger(), "Battery charged (%.1f%%): resuming patrol at '%s'",
          battery_.level(), waypoints_[current_index_].name.c_str());
        retries_ = 0;
        next_attempt_time_ = now_time;
        transitionTo(MissionState::kPatrolling);
      }
      break;

    case MissionState::kIdle:
    case MissionState::kFinished:
      break;
  }
}

void PatrolManager::transitionTo(MissionState next)
{
  if (next != state_) {
    RCLCPP_INFO(
      get_logger(), "State: %s -> %s", toString(state_).c_str(), toString(next).c_str());
  }
  state_ = next;
  publishState();
}

void PatrolManager::advanceWaypoint()
{
  ++current_index_;
  if (current_index_ < waypoints_.size()) {
    return;
  }

  current_index_ = 0;
  ++completed_loops_;
  RCLCPP_INFO(get_logger(), "Patrol loop %d completed", completed_loops_);

  const bool loop_limit_reached = max_loops_ > 0 && completed_loops_ >= max_loops_;
  if (!loop_patrol_ || loop_limit_reached) {
    transitionTo(MissionState::kFinished);
  }
}

void PatrolManager::invalidateActiveGoal()
{
  ++goal_seq_;
  goal_active_ = false;
  distance_remaining_ = 0.0;
}

// ---------------------------------------------------------------------------------------------
// Nav2 action client
// ---------------------------------------------------------------------------------------------

void PatrolManager::sendGoal(const Waypoint & target)
{
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = goal_frame_;
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = target.x;
  goal.pose.pose.position.y = target.y;
  goal.pose.pose.orientation.z = std::sin(target.yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(target.yaw / 2.0);

  const std::uint64_t seq = ++goal_seq_;
  goal_active_ = true;
  distance_remaining_ = 0.0;

  RCLCPP_INFO(
    get_logger(), "Sending goal '%s' -> (x=%.2f, y=%.2f, yaw=%.2f)",
    target.name.c_str(), target.x, target.y, target.yaw);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

  options.goal_response_callback =
    [this, seq](const GoalHandle::SharedPtr & goal_handle) {
      if (seq != goal_seq_) {
        return;  // Stale goal
      }
      if (!goal_handle) {
        onGoalRejected();
      }
    };

  options.feedback_callback =
    [this, seq](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      if (seq != goal_seq_) {
        return;
      }
      distance_remaining_ = feedback->distance_remaining;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "[%s] distance remaining: %.2f m | battery: %.1f%%",
        toString(state_).c_str(), distance_remaining_, battery_.level());
    };

  options.result_callback =
    [this, seq](const GoalHandle::WrappedResult & result) {
      if (seq != goal_seq_) {
        return;
      }
      goal_active_ = false;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          onGoalSucceeded();
          break;
        case rclcpp_action::ResultCode::ABORTED:
          onGoalFailed("aborted by Nav2");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          onGoalFailed("canceled externally");
          break;
        default:
          onGoalFailed("unknown result code");
          break;
      }
    };

  nav_client_->async_send_goal(goal, options);
}

void PatrolManager::onGoalRejected()
{
  // Usually means Nav2 is still activating: not the waypoint's fault, so it does not count as a retry.
  goal_active_ = false;
  RCLCPP_WARN(
    get_logger(), "Goal rejected by Nav2 (is the navigation stack active?). Retrying in %.1f s",
    retry_delay_s_);
  next_attempt_time_ = now() + rclcpp::Duration::from_seconds(retry_delay_s_);
}

void PatrolManager::onGoalSucceeded()
{
  retries_ = 0;

  if (state_ == MissionState::kReturningToDock) {
    RCLCPP_INFO(get_logger(), "Docked at charging station (battery %.1f%%)", battery_.level());
    transitionTo(MissionState::kCharging);
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Reached waypoint '%s' (%zu/%zu) | battery %.1f%%",
    waypoints_[current_index_].name.c_str(), current_index_ + 1, waypoints_.size(),
    battery_.level());
  advanceWaypoint();
  next_attempt_time_ = now();
}

void PatrolManager::onGoalFailed(const std::string & reason)
{
  const bool docking = state_ == MissionState::kReturningToDock;
  const Waypoint & target = docking ? dock_ : waypoints_[current_index_];

  ++retries_;
  if (retries_ <= max_retries_) {
    RCLCPP_WARN(
      get_logger(), "Goal '%s' failed (%s). Retry %d/%d in %.1f s",
      target.name.c_str(), reason.c_str(), retries_, max_retries_, retry_delay_s_);
    next_attempt_time_ = now() + rclcpp::Duration::from_seconds(retry_delay_s_);
    return;
  }

  retries_ = 0;

  if (docking) {
    RCLCPP_ERROR(
      get_logger(), "Dock unreachable after %d retries (%s). Will keep trying",
      max_retries_, reason.c_str());
    next_attempt_time_ = now() + rclcpp::Duration::from_seconds(retry_delay_s_);
    return;
  }

  RCLCPP_ERROR(
    get_logger(), "Skipping waypoint '%s' after %d retries (%s)",
    target.name.c_str(), max_retries_, reason.c_str());
  advanceWaypoint();
  next_attempt_time_ = now();
}

// ---------------------------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------------------------

void PatrolManager::publishState()
{
  std_msgs::msg::String msg;
  msg.data = toString(state_);
  state_pub_->publish(msg);
}

void PatrolManager::publishBattery(const rclcpp::Time & stamp)
{
  constexpr float kUnknown = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  msg.voltage = kUnknown;
  msg.temperature = kUnknown;
  msg.current = kUnknown;
  msg.charge = kUnknown;
  msg.capacity = kUnknown;
  msg.design_capacity = kUnknown;
  msg.percentage = static_cast<float>(battery_.level() / 100.0);
  msg.power_supply_status = state_ == MissionState::kCharging ?
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.present = true;
  battery_pub_->publish(msg);
}

// ---------------------------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------------------------

void PatrolManager::handleStart(Trigger::Response & response)
{
  switch (state_) {
    case MissionState::kIdle:
    case MissionState::kFinished:
      if (state_ == MissionState::kFinished) {
        current_index_ = 0;
        completed_loops_ = 0;
      }
      retries_ = 0;
      next_attempt_time_ = now();
      transitionTo(MissionState::kPatrolling);
      response.success = true;
      response.message = "Patrol started";
      break;

    case MissionState::kWaitingForNav2:
      autostart_ = true;
      response.success = true;
      response.message = "Patrol will start as soon as Nav2 is available";
      break;

    default:
      response.success = false;
      response.message = "Patrol already running (state: " + toString(state_) + ")";
      break;
  }
}

void PatrolManager::handleStop(Trigger::Response & response)
{
  switch (state_) {
    case MissionState::kWaitingForNav2:
      autostart_ = false;
      response.success = true;
      response.message = "Autostart disabled: the patrol will not start automatically";
      break;

    case MissionState::kIdle:
    case MissionState::kFinished:
      response.success = false;
      response.message = "Patrol is not running";
      break;

    default:
      if (goal_active_) {
        nav_client_->async_cancel_all_goals();
      }
      invalidateActiveGoal();
      transitionTo(MissionState::kIdle);
      response.success = true;
      response.message = "Patrol stopped";
      break;
  }
}

}  // namespace patrol_mission

RCLCPP_COMPONENTS_REGISTER_NODE(patrol_mission::PatrolManager)
