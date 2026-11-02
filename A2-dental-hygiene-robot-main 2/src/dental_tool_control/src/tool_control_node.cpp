// ChangeTool action server for the custom end effector.
//
// Each tool_id maps to one servo angle. On a goal the node sends a one-line
// command to the Arduino over USB serial and waits for its acknowledgement:
//   PC      -> Arduino : "S <angle>\n"    move servo to <angle> degrees (0..180)
//   Arduino -> PC      : "OK <angle>\n"   servo commanded and settle delay finished
//                        "ERR <reason>\n" command rejected
// success=true is only returned after the matching "OK". With robot_mode=fake no
// serial port is opened and the servo move is simulated with a fixed delay.
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "dental_interfaces/action/change_tool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace {
using Steady = std::chrono::steady_clock;

bool baudToSpeed(int baud, speed_t & speed) {
  switch (baud) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
    default: return false;
  }
}
}  // namespace

class ToolControlNode final : public rclcpp::Node {
 public:
  using Action = dental_interfaces::action::ChangeTool;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  ToolControlNode() : Node("tool_control") {
    mode_ = declare_parameter<std::string>("robot_mode", "fake");
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud_rate", 9600);
    ack_timeout_sec_ = declare_parameter<double>("ack_timeout_sec", 3.0);
    fake_move_sec_ = declare_parameter<double>("fake_move_sec", 1.0);
    tool_ids_ = declare_parameter<std::vector<std::string>>(
        "tool_ids", std::vector<std::string>{"brush"});
    tool_angles_ = declare_parameter<std::vector<int64_t>>(
        "tool_angles", std::vector<int64_t>{90});

    speed_t speed{};
    const std::set<std::string> unique_ids(tool_ids_.begin(), tool_ids_.end());
    bool angles_ok = true;
    for (const auto angle : tool_angles_) {
      angles_ok = angles_ok && angle >= 0 && angle <= 180;
    }
    if ((mode_ != "fake" && mode_ != "real") || port_.empty() ||
        !baudToSpeed(baud_, speed) ||
        !std::isfinite(ack_timeout_sec_) || ack_timeout_sec_ <= 0.0 || ack_timeout_sec_ > 30.0 ||
        !std::isfinite(fake_move_sec_) || fake_move_sec_ < 0.0 || fake_move_sec_ > 10.0 ||
        tool_ids_.empty() || tool_ids_.size() != tool_angles_.size() ||
        unique_ids.size() != tool_ids_.size() || unique_ids.count("") > 0 || !angles_ok) {
      throw std::invalid_argument(
          "Invalid tool_control parameters: check mode, baud rate, timeouts and "
          "matching unique tool_ids / tool_angles (0..180 deg)");
    }

    // Latched so late subscribers still see which tool is active.
    current_pub_ = create_publisher<std_msgs::msg::String>(
        "/dental/tools/current", rclcpp::QoS(1).reliable().transient_local());

    if (mode_ == "real") {
      openSerial(speed);
    }

    server_ = rclcpp_action::create_server<Action>(
        this, "/dental/tools/change_tool",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
          return handleGoal(*goal);
        },
        [](const std::shared_ptr<GoalHandle>) {
          // A servo command already sent cannot be meaningfully undone halfway.
          return rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<GoalHandle> h) {
          if (worker_.joinable()) worker_.join();
          worker_ = std::thread([this, h]() {execute(h);});
        });

    RCLCPP_INFO(get_logger(), "tool_control ready: robot_mode=%s, %zu tool(s)%s",
                mode_.c_str(), tool_ids_.size(),
                mode_ == "fake" ? ", servo moves are SIMULATED" : "");
  }

  ~ToolControlNode() override {
    stopping_.store(true);
    if (worker_.joinable()) worker_.join();
    if (fd_ >= 0) close(fd_);
  }

 private:
  int angleFor(const std::string & tool_id) const {
    for (size_t i = 0; i < tool_ids_.size(); ++i) {
      if (tool_ids_[i] == tool_id) return static_cast<int>(tool_angles_[i]);
    }
    return -1;
  }

  rclcpp_action::GoalResponse handleGoal(const Action::Goal & goal) {
    if (angleFor(goal.tool_id) < 0) {
      RCLCPP_WARN(get_logger(), "Reject ChangeTool: unknown tool '%s'", goal.tool_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (mode_ == "real" && fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Reject ChangeTool: serial port %s is not open", port_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Reject ChangeTool: already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void phase(const std::shared_ptr<GoalHandle> & h, const std::string & text) {
    auto fb = std::make_shared<Action::Feedback>();
    fb->phase = text;
    h->publish_feedback(fb);
    RCLCPP_INFO(get_logger(), "[TOOL] %s", text.c_str());
  }

  void finish(const std::shared_ptr<GoalHandle> & h, bool success, const std::string & msg) {
    auto result = std::make_shared<Action::Result>();
    result->success = success;
    result->message = msg;
    if (success) {
      RCLCPP_INFO(get_logger(), "[TOOL] SUCCESS: %s", msg.c_str());
      h->succeed(result);
    } else {
      RCLCPP_ERROR(get_logger(), "[TOOL] FAILED: %s", msg.c_str());
      h->abort(result);
    }
    busy_.store(false);
  }

  void execute(const std::shared_ptr<GoalHandle> h) {
    const auto tool_id = h->get_goal()->tool_id;
    const int angle = angleFor(tool_id);
    phase(h, "SENDING tool=" + tool_id + " angle=" + std::to_string(angle));

    std::string error;
    if (mode_ == "fake") {
      const auto deadline = Steady::now() + std::chrono::duration<double>(fake_move_sec_);
      while (Steady::now() < deadline && !stopping_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (stopping_.load()) error = "node shutting down";
    } else {
      phase(h, "WAITING_ACK");
      sendAndWaitAck(angle, error);
    }
    if (!error.empty()) {
      finish(h, false, "Tool change to '" + tool_id + "' failed: " + error);
      return;
    }

    std_msgs::msg::String current;
    current.data = tool_id;
    current_pub_->publish(current);
    phase(h, "DONE");
    finish(h, true, "Servo at " + std::to_string(angle) + " deg for tool '" + tool_id + "'");
  }

  void openSerial(speed_t speed) {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s: %s (goals will be rejected)",
                   port_.c_str(), std::strerror(errno));
      return;
    }
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr on %s failed: %s", port_.c_str(), std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;   // read() returns after at most VTIME even with no data
    tty.c_cc[VTIME] = 1;  // 0.1 s
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr on %s failed: %s", port_.c_str(), std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return;
    }
    // Opening the port resets most Arduinos; wait for the bootloader to hand over.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    tcflush(fd_, TCIOFLUSH);
    RCLCPP_INFO(get_logger(), "Serial open: %s @ %d baud", port_.c_str(), baud_);
  }

  // Returns true only when the Arduino acknowledges this exact angle.
  bool sendAndWaitAck(int angle, std::string & error) {
    tcflush(fd_, TCIFLUSH);  // drop any stale replies from an earlier timed-out command
    const std::string cmd = "S " + std::to_string(angle) + "\n";
    if (write(fd_, cmd.data(), cmd.size()) != static_cast<ssize_t>(cmd.size())) {
      error = "serial write failed: " + std::string(std::strerror(errno));
      return false;
    }
    const std::string expected_ack = "OK " + std::to_string(angle);
    const auto deadline = Steady::now() + std::chrono::duration<double>(ack_timeout_sec_);
    std::string line;
    char c = 0;
    while (Steady::now() < deadline && !stopping_.load()) {
      const ssize_t n = read(fd_, &c, 1);
      if (n < 0 && errno != EINTR && errno != EAGAIN) {
        error = "serial read failed: " + std::string(std::strerror(errno));
        return false;
      }
      if (n != 1 || c == '\r') continue;
      if (c != '\n') {
        if (line.size() < 64) line += c;
        continue;
      }
      if (line == expected_ack) return true;
      if (line.rfind("ERR", 0) == 0) {
        error = "Arduino replied '" + line + "'";
        return false;
      }
      line.clear();  // ignore unrelated output
    }
    error = stopping_.load() ? "node shutting down" : "no acknowledgement from Arduino within timeout";
    return false;
  }

  std::string mode_;
  std::string port_;
  int baud_{9600};
  double ack_timeout_sec_{3.0};
  double fake_move_sec_{1.0};
  std::vector<std::string> tool_ids_;
  std::vector<int64_t> tool_angles_;
  int fd_{-1};
  std::atomic<bool> busy_{false};
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_pub_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolControlNode>());
  rclcpp::shutdown();
  return 0;
}
