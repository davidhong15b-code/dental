// Increment 1: guarded C++ MoveIt planning and fake-hardware execution check.
//
// This is deliberately separate from the TreatTarget action server: reaching a
// pose is not evidence that the dental surface has been cleaned.
//
// Requires an already-running UR5e driver, move_group, and a live /joint_states.
// Set execute:=false to inspect the plan; only fake hardware may execute.

#include <cmath>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace {

// NodeOptions automatically declares CLI parameter overrides so that MoveIt
// can receive robot_description* parameters. Never redeclare an existing one.
template <typename T>
T parameterOr(const rclcpp::Node::SharedPtr & node,
              const std::string & name, const T & default_value)
{
  T value{};
  return node->get_parameter(name, value) ? value : default_value;
}

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

int run(const rclcpp::Node::SharedPtr & node)
{
  const auto logger = node->get_logger();
  const auto mode = parameterOr<std::string>(node, "robot_mode", "fake");
  const auto group_name = parameterOr<std::string>(node, "planning_group", "ur_manipulator");
  const auto pose_link = parameterOr<std::string>(node, "pose_link", "tool0");
  const bool execute = parameterOr<bool>(node, "execute", false);
  const double dx = parameterOr<double>(node, "dx", 0.0);
  const double dy = parameterOr<double>(node, "dy", 0.0);
  const double dz = parameterOr<double>(node, "dz", 0.025);

  // Do not allow a misspelled mode (or a real robot) to bypass the safety gate.
  if (mode != "fake") {
    RCLCPP_ERROR(logger, "Only robot_mode:=fake is permitted by this smoke test (received '%s').",
                 mode.c_str());
    return EXIT_FAILURE;
  }
  if (group_name.empty() || pose_link.empty()) {
    RCLCPP_ERROR(logger, "planning_group and pose_link must be non-empty.");
    return EXIT_FAILURE;
  }
  // This is a *bounded incremental move*, not a general-purpose motion node.
  const double displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!std::isfinite(displacement) || displacement < 1e-6 || displacement > 0.03) {
    RCLCPP_ERROR(logger, "Requested displacement must be finite and between 0 and 0.03 m.");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(logger, "Planning group=%s, link=%s, delta=(%.4f, %.4f, %.4f) m, execute=%s",
              group_name.c_str(), pose_link.c_str(), dx, dy, dz,
              execute ? "true" : "false");

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface group(node, group_name);
  const auto planning_frame = group.getPlanningFrame();
  if (planning_frame.empty()) {
    RCLCPP_ERROR(logger, "MoveIt returned an empty planning frame.");
    return EXIT_FAILURE;
  }
  if (!group.setEndEffectorLink(pose_link)) {
    RCLCPP_ERROR(logger, "Link '%s' is not a valid end-effector link for '%s'.",
                 pose_link.c_str(), group_name.c_str());
    return EXIT_FAILURE;
  }

  group.setPlanningTime(10.0);
  group.setNumPlanningAttempts(5);
  group.setMaxVelocityScalingFactor(0.05);
  group.setMaxAccelerationScalingFactor(0.05);
  group.setGoalPositionTolerance(0.005);
  group.setGoalOrientationTolerance(0.05);

  // getCurrentState() requires observed joint states; never invent a home pose.
  if (!group.getCurrentState(10.0)) {
    RCLCPP_ERROR(logger, "No current robot state within 10 s. Check /joint_states and TF.");
    return EXIT_FAILURE;
  }
  auto target = group.getCurrentPose(pose_link);
  if (target.header.frame_id.empty() || !finitePose(target.pose)) {
    RCLCPP_ERROR(logger, "Current %s pose is unavailable or invalid.", pose_link.c_str());
    return EXIT_FAILURE;
  }
  if (target.header.frame_id != planning_frame) {
    RCLCPP_ERROR(logger, "Current pose frame '%s' differs from planning frame '%s'.",
                 target.header.frame_id.c_str(), planning_frame.c_str());
    return EXIT_FAILURE;
  }

  target.pose.position.x += dx;
  target.pose.position.y += dy;
  target.pose.position.z += dz;
  group.setPoseReferenceFrame(planning_frame);
  group.setStartStateToCurrentState();
  if (!group.setPoseTarget(target, pose_link)) {
    RCLCPP_ERROR(logger, "MoveIt rejected the requested pose target.");
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(logger, "Planning from current pose to %s-frame position (%.4f, %.4f, %.4f) m",
              planning_frame.c_str(), target.pose.position.x,
              target.pose.position.y, target.pose.position.z);
  MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
      plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(logger, "MoveIt failed to produce a nonempty joint trajectory.");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(logger, "Plan succeeded: %zu trajectory points.",
              plan.trajectory_.joint_trajectory.points.size());

  if (!execute) {
    RCLCPP_INFO(logger, "Planning-only test complete. Inspect RViz before execute:=true.");
    return EXIT_SUCCESS;
  }

  // This tool has no dental-object collision model or calibrated tool TCP.
  // Only allow fake hardware to exercise trajectory-controller execution.
  const auto result = group.execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "MoveIt/controller failed to execute the fake-hardware trajectory.");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(logger, "Fake-hardware trajectory execution succeeded.");
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = EXIT_FAILURE;
  auto node = std::make_shared<rclcpp::Node>(
      "motion_smoke_test",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  try {
    status = run(node);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt smoke test failed: %s", ex.what());
  }

  executor.cancel();
  executor_thread.join();
  executor.remove_node(node);
  rclcpp::shutdown();
  return status;
}
