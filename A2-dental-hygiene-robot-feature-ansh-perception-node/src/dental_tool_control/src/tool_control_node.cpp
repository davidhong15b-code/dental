#include <memory>
#include <string>
#include <stdexcept>

#include "dental_interfaces/action/change_tool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ToolControlNode : public rclcpp::Node {
 public:
  using Action = dental_interfaces::action::ChangeTool;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  ToolControlNode() : Node("tool_control") {
    const auto mode = declare_parameter<std::string>("robot_mode", "fake");
    if (mode != "fake" && mode != "real") {
      throw std::invalid_argument("robot_mode must be fake or real");
    }
    RCLCPP_INFO(get_logger(), "Robot mode: %s", mode.c_str());
    // Tool mounting and TCP calibration must be implemented before accepting goals.

    // Safety: accept no goals until the real implementation is present.
    server_ = rclcpp_action::create_server<Action>(
        this, "/dental/tools/change_tool",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const Action::Goal>) {
          RCLCPP_WARN(get_logger(), "ChangeTool rejected: implementation not installed");
          return rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle>) {});
    RCLCPP_INFO(get_logger(), "tool_control interface scaffold started");
  }

 private:
  rclcpp_action::Server<Action>::SharedPtr server_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolControlNode>());
  rclcpp::shutdown();
  return 0;
}
