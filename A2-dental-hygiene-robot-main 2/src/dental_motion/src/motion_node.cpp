// Increment 3: MoveIt-backed TreatTarget action for guarded positioning demos.
//
// Important scope boundary: this does NOT clean a tooth. The UR5e model has no
// calibrated brush TCP, jaw collision mesh, force control or tool-state feedback.
// The node plans (and, with two explicit opt-ins, can execute on VERIFIED FAKE
// HARDWARE ONLY) a pre-approach -> hover -> retract sequence. It NEVER returns
// success=true until genuine treatment is implemented in a later increment.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "dental_interfaces/action/treat_target.hpp"
#include "dental_interfaces/msg/dental_finding.hpp"

namespace {
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using Finding = dental_interfaces::msg::DentalFinding;
using Point = geometry_msgs::msg::Point;
using Normal = geometry_msgs::msg::Vector3;

bool finitePoint(const Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool finiteNormal(const Normal & n)
{
  return std::isfinite(n.x) && std::isfinite(n.y) && std::isfinite(n.z);
}

double distance(const Point & a, const Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

Point offsetPoint(const Point & p, const Normal & unit_normal, double offset_m)
{
  Point result = p;
  result.x += unit_normal.x * offset_m;
  result.y += unit_normal.y * offset_m;
  result.z += unit_normal.z * offset_m;
  return result;
}
}  // namespace

class MotionNode final : public rclcpp::Node {
 public:
  using Action = dental_interfaces::action::TreatTarget;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  // Robot model parameters arrive from the launch file as overrides. With
  // automatic declaration enabled, ordinary parameters may already exist.
  template <typename T>
  T parameterOr(const std::string & name, const T & fallback)
  {
    T value{};
    return get_parameter(name, value) ? value : declare_parameter<T>(name, fallback);
  }

  MotionNode() : Node("motion", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    robot_mode_ = parameterOr<std::string>("robot_mode", "fake");
    planning_group_ = parameterOr<std::string>("planning_group", "ur_manipulator");
    pose_link_ = parameterOr<std::string>("pose_link", "tool0");
    base_frame_ = parameterOr<std::string>("robot_base_frame", "base_link");
    min_confidence_ = parameterOr<double>("min_confidence", 0.75);
    max_target_age_sec_ = parameterOr<double>("max_target_age_sec", 15.0);
    preapproach_distance_m_ = parameterOr<double>("preapproach_distance_m", 0.10);
    hover_distance_m_ = parameterOr<double>("hover_distance_m", 0.06);
    max_first_move_m_ = parameterOr<double>("max_first_move_m", 0.10);
    planning_time_sec_ = parameterOr<double>("planning_time_sec", 10.0);
    // Restricted to fake-hardware diagnostics. Defaults retain the original
    // conservative 5% limits until a deliberate override is supplied.
    fake_velocity_scaling_ = parameterOr<double>("fake_velocity_scaling", 0.05);
    fake_acceleration_scaling_ = parameterOr<double>("fake_acceleration_scaling", 0.05);
    // For this short, fake-only positioning demo, do not accept solutions that
    // move to a distant IK branch or take large joint-space detours.
    max_goal_joint_delta_rad_ = parameterOr<double>("max_goal_joint_delta_rad", 1.0);
    max_segment_joint_travel_rad_ = parameterOr<double>("max_segment_joint_travel_rad", 4.0);
    max_single_joint_travel_rad_ = parameterOr<double>("max_single_joint_travel_rad", 1.25);
    // Diagnostic budget for all three planned trajectories together. A short
    // positioning demo should not queue minutes of motion at 5% speed.
    max_total_planned_duration_sec_ = parameterOr<double>(
        "max_total_planned_duration_sec", 70.0);
    execute_fake_motion_ = parameterOr<bool>("execute_fake_motion", false);
    confirm_fake_hardware_ = parameterOr<bool>("confirm_fake_hardware", false);

    if ((robot_mode_ != "fake" && robot_mode_ != "real") ||
        planning_group_.empty() || pose_link_.empty() || base_frame_.empty() ||
        !std::isfinite(min_confidence_) || min_confidence_ < 0.0 || min_confidence_ > 1.0 ||
        !std::isfinite(max_target_age_sec_) || max_target_age_sec_ <= 0.0 ||
        !std::isfinite(preapproach_distance_m_) ||
        !std::isfinite(hover_distance_m_) ||
        hover_distance_m_ < 0.03 || preapproach_distance_m_ <= hover_distance_m_ ||
        preapproach_distance_m_ > 0.20 ||
        !std::isfinite(max_first_move_m_) || max_first_move_m_ <= 0.0 ||
        max_first_move_m_ > 0.15 ||
        !std::isfinite(planning_time_sec_) || planning_time_sec_ <= 0.0 ||
        !std::isfinite(fake_velocity_scaling_) ||
        fake_velocity_scaling_ < 0.01 || fake_velocity_scaling_ > 0.20 ||
        !std::isfinite(fake_acceleration_scaling_) ||
        fake_acceleration_scaling_ < 0.01 || fake_acceleration_scaling_ > 0.20 ||
        !std::isfinite(max_goal_joint_delta_rad_) ||
        max_goal_joint_delta_rad_ <= 0.0 || max_goal_joint_delta_rad_ > 1.5 ||
        !std::isfinite(max_segment_joint_travel_rad_) ||
        max_segment_joint_travel_rad_ <= 0.0 || max_segment_joint_travel_rad_ > 6.0 ||
        !std::isfinite(max_single_joint_travel_rad_) ||
        max_single_joint_travel_rad_ <= 0.0 || max_single_joint_travel_rad_ > 1.5 ||
        !std::isfinite(max_total_planned_duration_sec_) ||
        max_total_planned_duration_sec_ <= 0.0 ||
        max_total_planned_duration_sec_ > 120.0) {
      throw std::invalid_argument("Invalid motion parameters: check frames, mode and safe distance ranges");
    }
    if (execute_fake_motion_ && (robot_mode_ != "fake" || !confirm_fake_hardware_)) {
      throw std::invalid_argument(
          "Execution requires robot_mode=fake AND confirm_fake_hardware=true");
    }

    server_ = rclcpp_action::create_server<Action>(
        this, "/dental/motion/treat_target",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
          return handleGoal(*goal);
        },
        [this](const std::shared_ptr<GoalHandle> goal_handle) {
          return handleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandle> goal_handle) {
          handleAccepted(goal_handle);
        });

    RCLCPP_WARN(get_logger(),
        "TreatTarget positioning demo ready: robot_mode=%s, execute_fake_motion=%s, "
        "fake speed scale=%.2f, fake acceleration scale=%.2f. "
        "NO TOOL CONTACT OR CLEANING. Workcell scene and tool geometry must be separately calibrated/loaded.",
        robot_mode_.c_str(), execute_fake_motion_ ? "true" : "false",
        fake_velocity_scaling_, fake_acceleration_scaling_);
  }

  ~MotionNode() override
  {
    cancellation_requested_.store(true);
    // Do not destroy the MoveIt client while an execution worker still uses it.
    // During normal shutdown, request the trajectory to stop first.
    stopActiveMotion();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void requestStop()
  {
    cancellation_requested_.store(true);
    stopActiveMotion();
  }

 private:
  rclcpp_action::GoalResponse handleGoal(const Action::Goal & goal)
  {
    if (robot_mode_ != "fake") {
      RCLCPP_ERROR(get_logger(), "Rejected: real robot operation is not implemented/safe in Increment 3");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal.tool_id != "brush") {
      RCLCPP_WARN(get_logger(),
          "Rejected tool '%s': Increment 3 accepts brush as an identifier only; no tool is actuated",
          goal.tool_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto & target = goal.target;
    const auto & f = target.finding;
    if (target.header.frame_id != base_frame_ ||
        target.header.stamp.sec <= 0 || target.header.stamp.nanosec >= 1000000000u ||
        !finitePoint(f.position) || !finiteNormal(f.surface_normal) ||
        !std::isfinite(f.confidence) || f.confidence < min_confidence_ || f.confidence > 1.0 ||
        (f.issue_type != Finding::ISSUE_PLAQUE && f.issue_type != Finding::ISSUE_STAIN)) {
      RCLCPP_WARN(get_logger(), "Rejected invalid/unsupported target (expected measured %s-frame plaque/stain)",
                  base_frame_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto & n = f.surface_normal;
    const double norm = std::hypot(std::hypot(n.x, n.y), n.z);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.10) {
      RCLCPP_WARN(get_logger(), "Rejected target with non-unit surface normal (length %.4f)", norm);
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto acquisition_time = rclcpp::Time(target.header.stamp, get_clock()->get_clock_type());
    const double age = (get_clock()->now() - acquisition_time).seconds();
    if (!std::isfinite(age) || age > max_target_age_sec_ || age < -1.0) {
      RCLCPP_WARN(get_logger(), "Rejected stale/future target (age %.2f seconds)", age);
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejected concurrent treatment goal; motion worker is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancellation_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> &)
  {
    cancellation_requested_.store(true);
    RCLCPP_WARN(get_logger(), "Cancel requested. Stopping active trajectory if executing.");
    stopActiveMotion();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    // At most one active worker. A finished joinable thread may be reclaimed here.
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle] {executeGoal(goal_handle);});
  }

  void stopActiveMotion()
  {
    std::shared_ptr<MoveGroup> group;
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (executing_trajectory_) {
        group = active_group_;
      }
    }
    if (group) {
      group->stop();
    }
  }

  void publishPhase(const std::shared_ptr<GoalHandle> & handle, const std::string & phase)
  {
    if (!rclcpp::ok() || !handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->phase = phase;
    handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "TreatTarget phase: %s", phase.c_str());
  }

  bool interrupted(const std::shared_ptr<GoalHandle> & handle) const
  {
    return cancellation_requested_.load() || handle->is_canceling() || !rclcpp::ok();
  }

  void finishIncomplete(const std::shared_ptr<GoalHandle> & handle, const std::string & reason)
  {
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());

    // A terminal ROS action result is the completion signal to callers.
    // Release internal resources BEFORE publishing it. Otherwise a supervisor
    // can receive the result, request its next pass immediately, and have that
    // valid next goal rejected because busy_ is still true.
    //
    // The previous worker thread may still be unwinding its local variables.
    // handleAccepted() joins that thread before starting the next worker.
    // The mutex prevents the cancellation callback from racing this release.
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      executing_trajectory_ = false;
      active_group_.reset();
      busy_.store(false, std::memory_order_release);
    }

    if (!handle->is_active() || !rclcpp::ok()) {
      return;
    }
    auto result = std::make_shared<Action::Result>();
    result->success = false;
    result->message = reason;
    if (handle->is_canceling()) {
      handle->canceled(result);
    } else {
      handle->abort(result);
    }
  }

  // Update the predicted start state to the final waypoint of a planned segment.
  // This allows all three segments to be independently collision-checked even
  // when plan-only is selected (no fake movement is performed during planning).
  static void advanceState(moveit::core::RobotState & state, const MoveGroup::Plan & plan)
  {
    const auto & jt = plan.trajectory_.joint_trajectory;
    if (jt.joint_names.empty() || jt.points.empty() ||
        jt.points.back().positions.size() != jt.joint_names.size()) {
      throw std::runtime_error("Planner returned invalid/empty joint trajectory");
    }
    state.setVariablePositions(jt.joint_names, jt.points.back().positions);
    state.update();
  }

  void executeGoal(const std::shared_ptr<GoalHandle> & handle)
  {
    // All exit paths, including the exception handler, call finishIncomplete().
    // It releases busy_ before emitting the terminal action result. Do not
    // release busy_ again when this thread returns: a new pass may be active.

    try {
      if (interrupted(handle)) {
        finishIncomplete(handle, "Goal canceled before planning");
        return;
      }
      publishPhase(handle, "VALIDATING_CURRENT_STATE");
      auto group = std::make_shared<MoveGroup>(shared_from_this(), planning_group_);
      {
        std::lock_guard<std::mutex> lock(active_mutex_);
        active_group_ = group;
      }
      // MoveIt may plan in "world" while our target manager publishes in
      // "base_link". Frame names need not match: both must be valid links in
      // the SAME loaded robot model, and poses must be expressed consistently.
      // Never relabel coordinates from one frame as though they were another.
      RCLCPP_INFO(get_logger(), "MoveIt planning frame='%s', target frame='%s'",
                  group->getPlanningFrame().c_str(), base_frame_.c_str());
      if (!group->setEndEffectorLink(pose_link_)) {
        finishIncomplete(handle, "MoveIt rejected configured pose_link");
        return;
      }
      group->setPoseReferenceFrame(base_frame_);
      group->setPlanningTime(planning_time_sec_);
      group->setNumPlanningAttempts(5);
      group->setMaxVelocityScalingFactor(fake_velocity_scaling_);
      group->setMaxAccelerationScalingFactor(fake_acceleration_scaling_);
      group->setGoalPositionTolerance(0.005);
      group->setGoalOrientationTolerance(0.05);
      group->allowReplanning(false);

      auto start = group->getCurrentState(10.0);
      if (!start) {
        finishIncomplete(handle, "No current UR5e joint state; verify ROS_DOMAIN_ID and /joint_states");
        return;
      }
      // getCurrentPose() is normally reported in MoveIt's model/planning
      // frame, NOT necessarily in base_link. Instead compute tool0 relative
      // to base_link using FK from the SAME fresh joint state. For transforms
      // T_model_base and T_model_tool:
      //     T_base_tool = inverse(T_model_base) * T_model_tool.
      // This removes the model/world transform without guessing its offset.
      const auto & model = start->getRobotModel();
      if (!model->hasLinkModel(base_frame_) || !model->hasLinkModel(pose_link_)) {
        finishIncomplete(handle, "Missing base/tool link in MoveIt robot model");
        return;
      }
      start->updateLinkTransforms();
      const Eigen::Isometry3d base_T_tool =
          start->getGlobalLinkTransform(base_frame_).inverse() *
          start->getGlobalLinkTransform(pose_link_);
      const Eigen::Quaterniond orientation(base_T_tool.rotation());
      if (!base_T_tool.matrix().allFinite() ||
          !orientation.coeffs().allFinite() ||
          std::abs(orientation.norm() - 1.0) > 0.01) {
        finishIncomplete(handle, "Invalid current tool pose from MoveIt FK");
        return;
      }
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose.header.frame_id = base_frame_;
      current_pose.header.stamp = now();
      current_pose.pose.position.x = base_T_tool.translation().x();
      current_pose.pose.position.y = base_T_tool.translation().y();
      current_pose.pose.position.z = base_T_tool.translation().z();
      current_pose.pose.orientation.x = orientation.x();
      current_pose.pose.orientation.y = orientation.y();
      current_pose.pose.orientation.z = orientation.z();
      current_pose.pose.orientation.w = orientation.w();
      RCLCPP_INFO(get_logger(),
          "Current %s pose in %s: (%.4f, %.4f, %.4f)m",
          pose_link_.c_str(), base_frame_.c_str(),
          current_pose.pose.position.x, current_pose.pose.position.y,
          current_pose.pose.position.z);
      const auto & finding = handle->get_goal()->target.finding;
      Normal n = finding.surface_normal;
      const double magnitude = std::hypot(std::hypot(n.x, n.y), n.z);
      n.x /= magnitude; n.y /= magnitude; n.z /= magnitude;

      // Currently moves tool0 (not a calibrated brush tip). Preserve the current
      // tool0 orientation: surface normal alone cannot define a safe brush attitude.
      geometry_msgs::msg::PoseStamped pre = current_pose;
      geometry_msgs::msg::PoseStamped hover = current_pose;
      pre.pose.position = offsetPoint(finding.position, n, preapproach_distance_m_);
      hover.pose.position = offsetPoint(finding.position, n, hover_distance_m_);
      pre.header.stamp = now();
      hover.header.stamp = pre.header.stamp;
      if (distance(current_pose.pose.position, pre.pose.position) > max_first_move_m_) {
        finishIncomplete(handle, "Pre-approach exceeds bounded first-motion distance. Reposition in RViz first.");
        return;
      }
      // If the first move is tiny, MoveIt may still produce an empty trajectory;
      // this is treated as a failed demo rather than a misleading success.
      // Pre-approach and hover are short Cartesian displacements. The generic
      // setPoseTarget() constraint lets OMPL choose any equivalent IK branch,
      // including multi-revolution joint changes near a singularity.
      // Instead solve each target from its preceding predicted joint state.
      // MoveIt still plans/collision-checks the joint-space trajectory.
      const auto * joint_group = model->getJointModelGroup(planning_group_);
      if (!joint_group) {
        finishIncomplete(handle, "MoveIt robot model does not contain the configured planning group");
        return;
      }
      const std::array<geometry_msgs::msg::PoseStamped, 3> poses{pre, hover, pre};
      const std::array<std::string, 3> names{"PREAPPROACH", "HOVER", "RETRACT"};
      std::array<MoveGroup::Plan, 3> plans;
      std::array<double, 3> nominal_durations_sec{};
      double total_nominal_duration_sec = 0.0;
      auto predicted = std::make_shared<moveit::core::RobotState>(*start);
      for (size_t i = 0; i < plans.size(); ++i) {
        if (interrupted(handle)) {
          finishIncomplete(handle, "Goal canceled while planning");
          return;
        }
        publishPhase(handle, "PLANNING_" + names[i]);
        group->setStartState(*predicted);
        if (i == 2) {
          // Retract to the EXACT pre-approach joint configuration from plan 0.
          // A second Cartesian pose goal can select a different IK branch,
          // even when the requested tool pose is identical. The previously
          // planned hover path guarantees a geometric return path exists in
          // the unchanged scene, though the planner must still find one.
          const auto & pre_traj = plans[0].trajectory_.joint_trajectory;
          if (pre_traj.joint_names.empty() || pre_traj.points.empty() ||
              !group->setJointValueTarget(
                  pre_traj.joint_names, pre_traj.points.back().positions)) {
            finishIncomplete(handle, "Invalid pre-approach joint target for retract");
            return;
          }
        } else {
          // RobotState::setFromIK() expects a pose in the MODEL frame, not
          // base_link. Transform with the FK of the same predicted state.
          predicted->updateLinkTransforms();
          Eigen::Isometry3d base_T_goal = Eigen::Isometry3d::Identity();
          base_T_goal.translation() = Eigen::Vector3d(
              poses[i].pose.position.x,
              poses[i].pose.position.y,
              poses[i].pose.position.z);
          const auto & qmsg = poses[i].pose.orientation;
          Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
          if (!q.coeffs().allFinite() || q.norm() < 1e-6) {
            finishIncomplete(handle, "Invalid tool orientation in target pose");
            return;
          }
          q.normalize();
          base_T_goal.linear() = q.toRotationMatrix();
          const Eigen::Isometry3d model_T_goal =
              predicted->getGlobalLinkTransform(base_frame_) * base_T_goal;

          // Seed from the previous plan end, and impose explicit local joint
          // consistency limits. A failure is preferable to wrapping 2*pi.
          auto ik_state = std::make_shared<moveit::core::RobotState>(*predicted);
          const std::vector<double> consistency_limits(
              joint_group->getVariableCount(), max_goal_joint_delta_rad_);
          if (!ik_state->setFromIK(
                  joint_group, model_T_goal, pose_link_,
                  consistency_limits, 0.25)) {
            finishIncomplete(handle,
                "No local IK for " + names[i] +
                " within joint consistency limits. Reposition fake robot to a bent-elbow posture.");
            return;
          }
          ik_state->update();
          std::vector<double> before, after;
          predicted->copyJointGroupPositions(joint_group, before);
          ik_state->copyJointGroupPositions(joint_group, after);
          if (before.size() != after.size() || after.size() != joint_group->getVariableCount()) {
            finishIncomplete(handle, "Unexpected joint count returned by local IK");
            return;
          }
          bool too_far = false;
          for (size_t j = 0; j < after.size(); ++j) {
            if (!std::isfinite(after[j]) ||
                std::abs(after[j] - before[j]) > max_goal_joint_delta_rad_) {
              too_far = true;
              break;
            }
          }
          if (too_far || !group->setJointValueTarget(*ik_state)) {
            finishIncomplete(handle, "IK goal is not a valid nearby joint configuration");
            return;
          }
        }
        const auto code = group->plan(plans[i]);
        group->clearPoseTargets();
        if (code != moveit::core::MoveItErrorCode::SUCCESS ||
            plans[i].trajectory_.joint_trajectory.points.empty()) {
          RCLCPP_ERROR(get_logger(),
              "Planning %s failed: MoveIt result=%d (see move_group logs for details)",
              names[i].c_str(), code.val);
          finishIncomplete(handle, "MoveIt failed to plan " + names[i] + " segment");
          return;
        }
        // Report the actual time parameterisation; the retract joint-space
        // path may be much longer than the visible 40 mm TCP displacement.
        const auto & trajectory = plans[i].trajectory_.joint_trajectory;
        const auto & end_time = trajectory.points.back().time_from_start;
        const double nominal_seconds = static_cast<double>(end_time.sec) +
            static_cast<double>(end_time.nanosec) * 1e-9;
        if (!std::isfinite(nominal_seconds) || nominal_seconds <= 0.0) {
          finishIncomplete(handle, "Invalid time-parameterised " + names[i] + " trajectory");
          return;
        }
        // The default time-parameterizer resamples at ~0.1 s intervals.
        // Waypoint count alone does NOT indicate whether the joint-space
        // path is unnecessarily long. Measure the actual joint travel.
        const auto joint_count = trajectory.joint_names.size();
        if (joint_count == 0 || trajectory.points.front().positions.size() != joint_count) {
          finishIncomplete(handle, "Trajectory has inconsistent joint dimensions");
          return;
        }
        std::vector<double> travel_per_joint(joint_count, 0.0);
        double net_joint_travel = 0.0;
        for (size_t j = 0; j < joint_count; ++j) {
          const double first = trajectory.points.front().positions[j];
          const double last = trajectory.points.back().positions[j];
          if (!std::isfinite(first) || !std::isfinite(last)) {
            finishIncomplete(handle, "Non-finite joint state in planned trajectory");
            return;
          }
          net_joint_travel += std::abs(last - first);
        }
        for (size_t k = 1; k < trajectory.points.size(); ++k) {
          const auto & prev = trajectory.points[k - 1].positions;
          const auto & next = trajectory.points[k].positions;
          if (prev.size() != joint_count || next.size() != joint_count) {
            finishIncomplete(handle, "Inconsistent waypoint dimensions in planned trajectory");
            return;
          }
          for (size_t j = 0; j < joint_count; ++j) {
            if (!std::isfinite(prev[j]) || !std::isfinite(next[j])) {
              finishIncomplete(handle, "Non-finite waypoint in planned trajectory");
              return;
            }
            travel_per_joint[j] += std::abs(next[j] - prev[j]);
          }
        }
        double joint_path_travel = 0.0;
        for (const double travelled : travel_per_joint) {
          joint_path_travel += travelled;
        }
        nominal_durations_sec[i] = nominal_seconds;
        total_nominal_duration_sec += nominal_seconds;
        RCLCPP_INFO(get_logger(),
            "Planned %s: %zu waypoints, nominal duration %.2fs (total %.2fs), "
            "joint L1 path=%.3frad, net=%.3frad, detour ratio=%.2f",
            names[i].c_str(), trajectory.points.size(),
            nominal_seconds, total_nominal_duration_sec,
            joint_path_travel, net_joint_travel,
            joint_path_travel / std::max(net_joint_travel, 1e-6));
        bool excessive_joint_travel = joint_path_travel > max_segment_joint_travel_rad_;
        for (size_t j = 0; j < joint_count; ++j) {
          RCLCPP_INFO(get_logger(), "  %s total travel: %.3f rad",
                      trajectory.joint_names[j].c_str(), travel_per_joint[j]);
          if (travel_per_joint[j] > max_single_joint_travel_rad_) {
            excessive_joint_travel = true;
          }
        }
        if (excessive_joint_travel) {
          finishIncomplete(handle,
              "Rejected " + names[i] + " trajectory: excessive joint-space travel. "
              "Avoid IK branch switching/revolutions; reposition fake robot if necessary.");
          return;
        }
        if (total_nominal_duration_sec > max_total_planned_duration_sec_) {
          finishIncomplete(handle, "Planned motion exceeds diagnostic time budget; inspect segment durations before executing");
          return;
        }
        advanceState(*predicted, plans[i]);
        publishPhase(handle, "PLANNED_" + names[i]);
      }
      if (interrupted(handle)) {
        finishIncomplete(handle, "Goal canceled before execution");
        return;
      }
      if (!execute_fake_motion_) {
        finishIncomplete(handle,
            "All three segments planned. PLAN-ONLY: no motion and no cleaning performed (expected result.success=false).");
        return;
      }

      // External operator MUST have verified the controller uses fake hardware.
      // robot_mode/confirm_fake_hardware parameters are NOT hardware attestation.
      for (size_t i = 0; i < plans.size(); ++i) {
        if (interrupted(handle)) {
          finishIncomplete(handle, "Goal canceled before segment " + names[i]);
          return;
        }
        RCLCPP_INFO(get_logger(), "Executing %s; expected trajectory duration %.2fs",
                    names[i].c_str(), nominal_durations_sec[i]);
        publishPhase(handle, "EXECUTING_" + names[i]);
        {
          std::lock_guard<std::mutex> lock(active_mutex_);
          executing_trajectory_ = true;
        }
        const auto code = group->execute(plans[i]);  // blocking until MoveIt returns
        RCLCPP_INFO(get_logger(), "MoveIt execute(%s) returned code=%d",
                    names[i].c_str(), code.val);
        {
          std::lock_guard<std::mutex> lock(active_mutex_);
          executing_trajectory_ = false;
        }
        if (interrupted(handle)) {
          finishIncomplete(handle, "Goal canceled during motion; inspect robot state before retrying");
          return;
        }
        if (code != moveit::core::MoveItErrorCode::SUCCESS) {
          finishIncomplete(handle,
              "MoveIt execution failed in " + names[i] + ". No automatic recovery motion attempted.");
          return;
        }
        publishPhase(handle, "EXECUTED_" + names[i]);
      }
      finishIncomplete(handle,
          "Fake-hardware pre-approach/hover/retract completed. NO cleaning or tool contact performed; TreatTarget remains incomplete.");
    } catch (const std::exception & e) {
      finishIncomplete(handle, std::string("Motion exception: ") + e.what());
    }
  }

  std::string robot_mode_, planning_group_, pose_link_, base_frame_;
  double fake_velocity_scaling_, fake_acceleration_scaling_;
  double max_goal_joint_delta_rad_, max_segment_joint_travel_rad_, max_single_joint_travel_rad_;
  double min_confidence_, max_target_age_sec_, preapproach_distance_m_;
  double hover_distance_m_, max_first_move_m_, planning_time_sec_;
  double max_total_planned_duration_sec_;
  bool execute_fake_motion_{false};
  bool confirm_fake_hardware_{false};
  std::atomic<bool> busy_{false};
  std::atomic<bool> cancellation_requested_{false};
  std::mutex active_mutex_;
  std::shared_ptr<MoveGroup> active_group_;
  bool executing_trajectory_{false};
  std::thread worker_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MotionNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    node->requestStop();
    executor.remove_node(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motion"), "Motion node failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
