// Closed-loop inspection -> target selection -> fake UR5e positioning -> reinspection.
// The perception action and findings topic are the only source of detections.
// The positioning-only TreatTarget action never counts as physical treatment.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/time.hpp"
#include "dental_interfaces/action/inspect_teeth.hpp"
#include "dental_interfaces/action/run_cycle.hpp"
#include "dental_interfaces/action/treat_target.hpp"
#include "dental_interfaces/msg/dental_target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
namespace {
using Inspect = dental_interfaces::action::InspectTeeth;
using Run = dental_interfaces::action::RunCycle;
using Motion = dental_interfaces::action::TreatTarget;
using Target = dental_interfaces::msg::DentalTarget;
using FindingArray = dental_interfaces::msg::DentalFindingArray;
using Steady = std::chrono::steady_clock;

bool validStamp(const builtin_interfaces::msg::Time & t) {
  return t.sec > 0 && t.nanosec < 1000000000u;
}
bool sameStamp(const builtin_interfaces::msg::Time & a,
               const builtin_interfaces::msg::Time & b) {
  return a.sec == b.sec && a.nanosec == b.nanosec;
}
} // namespace

class Supervisor final : public rclcpp::Node {
 public:
  using ServerHandle = rclcpp_action::ServerGoalHandle<Run>;
  Supervisor() : Node("supervisor") {
    max_passes_ = declare_parameter<int>("max_positioning_passes", 2);
    timeout_inspection_ = declare_parameter<double>("inspection_timeout_sec", 15.0);
    timeout_target_ = declare_parameter<double>("target_timeout_sec", 3.0);
    timeout_motion_ = declare_parameter<double>("motion_timeout_sec", 100.0);
    max_joint_age_ = declare_parameter<double>("max_joint_state_age_sec", 2.0);
    max_scan_age_ = declare_parameter<double>("max_scan_age_sec", 10.0);
    require_scene_ = declare_parameter<bool>("require_workcell_ready", false);
    if (max_passes_ < 1 || max_passes_ > 10 || timeout_inspection_ <= 0.0 ||
        timeout_target_ <= 0.0 || timeout_motion_ <= 0.0 || max_joint_age_ <= 0.0 ||
        max_scan_age_ <= 0.0) {
      throw std::invalid_argument("Invalid supervisor bounds");
    }
    inspect_client_ = rclcpp_action::create_client<Inspect>(this, "/dental/perception/inspect");
    motion_client_ = rclcpp_action::create_client<Motion>(this, "/dental/motion/treat_target");
    selected_sub_ = create_subscription<Target>(
        "/dental/targets/selected", rclcpp::QoS(10),
        [this](Target::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          recent_targets_.push_back(*msg);
          if (recent_targets_.size() > 20) recent_targets_.pop_front();
        });
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (msg->name.size() >= 6 && msg->name.size() == msg->position.size()) {
            last_joint_state_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                Steady::now().time_since_epoch()).count());
          }
        });
    scene_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/dental/workcell/ready", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          scene_ready_.store(msg->data);
        });
    server_ = rclcpp_action::create_server<Run>(
        this, "/dental/supervisor/run_cycle",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Run::Goal>) {
          bool expected = false;
          if (!busy_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(get_logger(), "Reject RunCycle: already running");
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<ServerHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerHandle> h) {
          if (worker_.joinable()) worker_.join();
          worker_ = std::thread([this, h]() {run(h);});
        });
    RCLCPP_INFO(get_logger(), "Supervisor ready: max positioning passes=%d; ", max_passes_);
    RCLCPP_WARN(get_logger(),
      "PASSIVE BRUSH MVP: no cleaning actuation; only verified motion execution is counted as positioning");
  }
  ~Supervisor() override {
    stopping_.store(true);
    if (worker_.joinable()) worker_.join();
  }

 private:
  bool cancelled(const std::shared_ptr<ServerHandle> & h) const {
    return stopping_.load() || !rclcpp::ok() || h->is_canceling();
  }
  bool jointsFresh() const {
    const auto last = last_joint_state_ns_.load();
    if (last <= 0) return false;
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        Steady::now().time_since_epoch()).count();
    return now_ns >= last && static_cast<double>(now_ns - last) * 1e-9 <= max_joint_age_;
  }
  template<class Future>
  bool wait(Future & future, double seconds, const std::shared_ptr<ServerHandle> & h,
            bool require_joints = false) {
    const auto deadline = Steady::now() + std::chrono::duration<double>(seconds);
    while (Steady::now() < deadline && !cancelled(h)) {
      if (require_joints && !jointsFresh()) return false;
      if (future.wait_for(100ms) == std::future_status::ready) return true;
    }
    return false;
  }
  void phase(const std::shared_ptr<ServerHandle> & h, const std::string & text) {
    auto fb = std::make_shared<Run::Feedback>();
    fb->phase = text;
    fb->treated_count = 0;  // passive brush = NO treatment proof
    h->publish_feedback(fb);
    RCLCPP_INFO(get_logger(), "[SUPERVISOR] %s", text.c_str());
  }
  void finish(const std::shared_ptr<ServerHandle> & h, bool success,
              const std::string & msg) {
    auto result = std::make_shared<Run::Result>();
    result->success = success;
    result->treated_count = 0;
    result->message = msg;
    RCLCPP_INFO(get_logger(), "[SUPERVISOR] FINAL: %s", msg.c_str());
    // Publishing the terminal action result lets clients immediately request
    // another cycle. Make the server available before that completion signal,
    // consistent with the motion action server's lifecycle contract.
    busy_.store(false, std::memory_order_release);
    if (h->is_canceling()) h->canceled(result);
    else if (success) h->succeed(result);
    else h->abort(result);
  }
  bool scanFresh(const FindingArray & a) const {
    if (a.header.frame_id.empty() || !validStamp(a.header.stamp)) return false;
    const double age = (now() - rclcpp::Time(a.header.stamp, get_clock()->get_clock_type())).seconds();
    if (!std::isfinite(age) || age < -1.0 || age > max_scan_age_) return false;
    for (const auto & f : a.findings) {
      if (!std::isfinite(f.confidence) || f.confidence < 0.0f || f.confidence > 1.0f) return false;
    }
    return true;
  }
  bool matchingTarget(const Target & t, const FindingArray & findings) const {
    if (!sameStamp(t.header.stamp, findings.header.stamp) || t.header.frame_id != "base_link")
      return false;
    for (const auto & f : findings.findings) {
      if (f.id == t.finding.id && f.issue_type == t.finding.issue_type &&
          f.confidence >= 0.75f &&
          std::abs(f.confidence - t.finding.confidence) < 1e-5f)
        return true;
    }
    return false;
  }
  bool awaitTarget(const FindingArray & findings, Target & out,
                   const std::shared_ptr<ServerHandle> & h) {
    const auto deadline = Steady::now() + std::chrono::duration<double>(timeout_target_);
    while (Steady::now() < deadline && !cancelled(h)) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (auto it = recent_targets_.rbegin(); it != recent_targets_.rend(); ++it) {
          if (matchingTarget(*it, findings)) {out = *it; return true;}
        }
      }
      std::this_thread::sleep_for(40ms);
    }
    return false;
  }
  void run(const std::shared_ptr<ServerHandle> & h) {
    try { executeCycle(h); }
    catch (const std::exception & e) { finish(h, false, std::string("Supervisor exception: ") + e.what()); }
  }
  void executeCycle(const std::shared_ptr<ServerHandle> & h) {
    if (require_scene_ && !scene_ready_.load()) {
      finish(h, false, "Measured MoveIt workcell scene not confirmed; reject motion cycle"); return;
    }
    if (!jointsFresh()) {
      finish(h, false, "No fresh UR5e joint states; cannot start a supervised cycle");
      return;
    }
    int positions_done = 0;
    // A final inspection follows the last pass; no finding may be inferred from silence.
    for (int inspection = 0; inspection <= max_passes_; ++inspection) {
      if (cancelled(h)) { finish(h, false, "Operator cancelled cycle"); return; }
      phase(h, "INSPECTING pass=" + std::to_string(inspection + 1));
      if (!inspect_client_->wait_for_action_server(2s)) {
        finish(h, false, "InspectTeeth action unavailable (perception must implement it)"); return;
      }
      Inspect::Goal ig;
      ig.region_id = h->get_goal()->region_id;
      auto send = inspect_client_->async_send_goal(ig);
      if (!wait(send, 3.0, h)) {finish(h, false, "Inspection goal was not accepted in time"); return;}
      auto inspect_gh = send.get();
      if (!inspect_gh) {finish(h, false, "Perception rejected InspectTeeth"); return;}
      auto inspect_result = inspect_client_->async_get_result(inspect_gh);
      if (!wait(inspect_result, timeout_inspection_, h)) {
        inspect_client_->async_cancel_goal(inspect_gh);
        finish(h, false, "Inspection timed out or cycle canceled"); return;
      }
      const auto inspected = inspect_result.get();
      if (inspected.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !inspected.result || !inspected.result->success ||
          !scanFresh(inspected.result->findings)) {
        finish(h, false, "Inspection unsuccessful, stale or invalid (no motion commanded)"); return;
      }
      const auto & findings = inspected.result->findings;
      phase(h, "SCAN_COMPLETE findings=" + std::to_string(findings.findings.size()));
      if (findings.findings.empty()) {
        if (positions_done == 0) finish(h, true, "Inspection complete: no targets found; zero treatments required");
        else finish(h, false,
          "Scan returned no targets after positioning, but passive brush did not clean; "
          "requires independent cleaning verification (positioning demo complete)");
        return;
      }
      if (inspection == max_passes_) {
        finish(h, false, "Residual findings remain after " + std::to_string(positions_done) +
               " positioning passes; stopped at configured pass limit");
        return;
      }
      phase(h, "AWAITING_TF_SELECTED_TARGET");
      Target target;
      if (!awaitTarget(findings, target, h)) {
        finish(h, false, "No matching base_link target from target_manager; check camera TF, confidence and timestamps"); return;
      }
      if (require_scene_ && !scene_ready_.load()) {
        finish(h, false, "Workcell scene readiness lost before motion"); return;
      }
      if (!jointsFresh()) {finish(h, false, "Joint-state watchdog failed before motion"); return;}
      if (!motion_client_->wait_for_action_server(2s)) {
        finish(h, false, "TreatTarget action unavailable"); return;
      }
      Motion::Goal mg;
      mg.target = target;
      mg.tool_id = "brush";  // passive pre-mounted tool, NO tool change action
      phase(h, "MOTION_REQUEST target_id=" + std::to_string(target.finding.id));
      auto executed_retract = std::make_shared<std::atomic<bool>>(false);
      rclcpp_action::Client<Motion>::SendGoalOptions options;
      options.feedback_callback = [this, h, executed_retract](
          rclcpp_action::ClientGoalHandle<Motion>::SharedPtr,
          const std::shared_ptr<const Motion::Feedback> fb) {
            if (fb->phase == "EXECUTED_RETRACT") executed_retract->store(true);
            phase(h, "MOTION_" + fb->phase);
          };
      auto motion_send = motion_client_->async_send_goal(mg, options);
      if (!wait(motion_send, 3.0, h, true)) {
        finish(h, false, "Motion goal rejected/timed out or joint state lost"); return;
      }
      auto motion_gh = motion_send.get();
      if (!motion_gh) {finish(h, false, "Motion server rejected the transformed target"); return;}
      auto motion_result = motion_client_->async_get_result(motion_gh);
      if (!wait(motion_result, timeout_motion_, h, true)) {
        motion_client_->async_cancel_goal(motion_gh);
        finish(h, false, "Motion result timed out, cancelled, or lost joint states; stop requested"); return;
      }
      const auto completed = motion_result.get();
      for (int check = 0; check < 10 && !executed_retract->load() && !cancelled(h); ++check) {
        std::this_thread::sleep_for(20ms);
      }
      // Legacy Increment-3 action ABORTS intentionally after positioning.
      // Thus we only accept the exact acknowledged terminal phase together
      // with a completed action and a result that says no cleaning occurred.
      if (!executed_retract->load() ||
          completed.code != rclcpp_action::ResultCode::ABORTED ||
          !completed.result || completed.result->success ||
          completed.result->message.find("NO cleaning") == std::string::npos) {
        finish(h, false, "Positioning was not fully acknowledged; aborting before reinspection"); return;
      }
      ++positions_done;
      phase(h, "POSITION_ONLY_COMPLETE count=" + std::to_string(positions_done) +
              " (NOT CLEANED); REQUESTING_FRESH_SCAN");
    }
    finish(h, false, "Supervisor reached unexpected end of cycle");
  }

  bool require_scene_{false};
  std::atomic<bool> scene_ready_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scene_sub_;
  int max_passes_{};
  double timeout_inspection_{}, timeout_target_{}, timeout_motion_{};
  double max_joint_age_{}, max_scan_age_{};
  std::mutex data_mutex_;
  std::deque<Target> recent_targets_;
  std::atomic<int64_t> last_joint_state_ns_{0};
  std::atomic<bool> busy_{false};
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  rclcpp_action::Client<Inspect>::SharedPtr inspect_client_;
  rclcpp_action::Client<Motion>::SharedPtr motion_client_;
  rclcpp_action::Server<Run>::SharedPtr server_;
  rclcpp::Subscription<Target>::SharedPtr selected_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Supervisor>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
