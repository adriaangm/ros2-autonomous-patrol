#ifndef PATROL_MISSION__PATROL_MANAGER_HPP_
#define PATROL_MISSION__PATROL_MANAGER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "patrol_mission/battery_model.hpp"

namespace patrol_mission
{

/// Named 2D pose in the goal frame.
struct Waypoint
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum class MissionState
{
  kWaitingForNav2,
  kIdle,
  kPatrolling,
  kReturningToDock,
  kCharging,
  kFinished
};

std::string toString(MissionState state);

/// Mission layer on top of Nav2: patrols a list of waypoints, retries or skips failed goals and
/// returns to the charging dock when the simulated battery runs low.
class PatrolManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Trigger = std_srvs::srv::Trigger;

  explicit PatrolManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Configuration
  void loadParameters();
  Waypoint loadPose(const std::string & parameter_name, const std::string & label);

  // State machine
  void onTick();
  void transitionTo(MissionState next);
  void advanceWaypoint();
  void invalidateActiveGoal();

  // Nav2 action client
  void sendGoal(const Waypoint & target);
  void onGoalRejected();
  void onGoalSucceeded();
  void onGoalFailed(const std::string & reason);

  // Outputs
  void publishState();
  void publishBattery(const rclcpp::Time & stamp);

  // Services
  void handleStart(Trigger::Response & response);
  void handleStop(Trigger::Response & response);

  // Parameters
  bool autostart_{true};
  bool loop_patrol_{true};
  int max_loops_{0};
  int max_retries_{2};
  double retry_delay_s_{3.0};
  std::string goal_frame_{"map"};
  std::vector<Waypoint> waypoints_;
  Waypoint dock_;
  BatteryModel battery_;

  // Runtime state
  MissionState state_{MissionState::kWaitingForNav2};
  std::size_t current_index_{0};
  int retries_{0};
  int completed_loops_{0};
  bool goal_active_{false};
  std::uint64_t goal_seq_{0};
  double distance_remaining_{0.0};
  rclcpp::Time next_attempt_time_;
  rclcpp::Time last_tick_time_;
  bool has_last_tick_{false};

  // ROS interfaces
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace patrol_mission

#endif  // PATROL_MISSION__PATROL_MANAGER_HPP_
