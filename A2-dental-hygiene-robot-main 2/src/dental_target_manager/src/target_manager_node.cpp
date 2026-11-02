// Gateway 2, Increment 2: select a measured dental finding and express it in
// the robot base frame. This node NEVER commands the robot or invents findings.
//
// Perception contract (unchanged): /dental/perception/findings is one completed
// inspection, with positions in metres and outward unit normals in its stamped
// header.frame_id. See docs/Important Markdowns/PERCEPTION_HANDOFF.md.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "dental_interfaces/msg/dental_finding.hpp"
#include "dental_interfaces/msg/dental_finding_array.hpp"
#include "dental_interfaces/msg/dental_target.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {
using Finding = dental_interfaces::msg::DentalFinding;
using FindingArray = dental_interfaces::msg::DentalFindingArray;
using Target = dental_interfaces::msg::DentalTarget;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using namespace std::chrono_literals;

bool finitePoint(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool finiteNormal(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double normalLength(const geometry_msgs::msg::Vector3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

void normalize(geometry_msgs::msg::Vector3 & v)
{
  const double length = normalLength(v);
  v.x /= length;
  v.y /= length;
  v.z /= length;
}

bool supportedIssue(uint8_t type)
{
  // Only plaque/stain can currently map to planned treatments.
  return type == Finding::ISSUE_PLAQUE || type == Finding::ISSUE_STAIN;
}
}  // namespace

class TargetManagerNode : public rclcpp::Node {
 public:
  TargetManagerNode() : Node("target_manager")
  {
    base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    confidence_threshold_ = declare_parameter<double>("min_confidence", 0.75);
    max_inspection_age_sec_ = declare_parameter<double>("max_inspection_age_sec", 10.0);
    transform_wait_sec_ = declare_parameter<double>("transform_wait_sec", 1.5);

    if (base_frame_.empty() || base_frame_.front() == '/' ||
        !std::isfinite(confidence_threshold_) || confidence_threshold_ < 0.0 ||
        confidence_threshold_ > 1.0 || !std::isfinite(max_inspection_age_sec_) ||
        max_inspection_age_sec_ < 0.0 || !std::isfinite(transform_wait_sec_) ||
        transform_wait_sec_ <= 0.0) {
      throw std::invalid_argument("Invalid target manager parameters; see parameter documentation");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Uses a dedicated internal TF listener node/thread. Our subscription and
    // timer can therefore remain non-blocking on the main ROS executor.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    targets_pub_ = create_publisher<Target>(
        "/dental/targets/selected", rclcpp::QoS(10));
    // Transient-local lets RViz display the latest selected target on startup.
    markers_pub_ = create_publisher<MarkerArray>(
        "/dental/targets/markers", rclcpp::QoS(1).reliable().transient_local());

    findings_sub_ = create_subscription<FindingArray>(
        "/dental/perception/findings", rclcpp::QoS(10),
        [this](FindingArray::ConstSharedPtr msg) {onFindings(std::move(msg));});

    retry_timer_ = create_wall_timer(100ms, [this]() {tryPublishPending();});
    RCLCPP_INFO(get_logger(),
        "Target manager ready: base=%s, min_confidence=%.2f, max_age=%.1fs, TF_wait=%.1fs. "
        "Selection uses confidence (NOT robot reachability).",
        base_frame_.c_str(), confidence_threshold_, max_inspection_age_sec_,
        transform_wait_sec_);
  }

 private:
  struct Pending {
    Target target;
    std::string source_frame;
    std::chrono::steady_clock::time_point received_at;
  };

  bool validFinding(const Finding & finding) const
  {
    if (!supportedIssue(finding.issue_type) ||
        !std::isfinite(finding.confidence) ||
        finding.confidence < confidence_threshold_ || finding.confidence > 1.0 ||
        !finitePoint(finding.position) || !finiteNormal(finding.surface_normal)) {
      return false;
    }
    const double magnitude = normalLength(finding.surface_normal);
    // Normals must already be approximately unit length as contracted with
    // perception. Renormalisation below corrects small numerical errors only.
    return std::isfinite(magnitude) && std::abs(magnitude - 1.0) <= 0.10;
  }

  void onFindings(FindingArray::ConstSharedPtr msg)
  {
    // New inspections supersede previous pending selections. Never publish an
    // old selection after a newer inspection has arrived.
    pending_.reset();
    clearMarkers();

    if (msg->header.frame_id.empty() || msg->header.frame_id.front() == '/' ||
        (msg->header.stamp.sec <= 0 || msg->header.stamp.nanosec >= 1000000000u)) {
      RCLCPP_WARN(get_logger(), "Rejected inspection: missing/invalid frame or timestamp");
      return;
    }

    const rclcpp::Time acquired(msg->header.stamp, get_clock()->get_clock_type());
    const double age_sec = (get_clock()->now() - acquired).seconds();
    if ((max_inspection_age_sec_ > 0.0 && age_sec > max_inspection_age_sec_) ||
        age_sec < -1.0) {
      RCLCPP_WARN(get_logger(),
          "Rejected inspection from '%s': acquisition timestamp age %.2fs "
          "(check clock synchronisation or max_inspection_age_sec)",
          msg->header.frame_id.c_str(), age_sec);
      return;
    }

    if (msg->findings.empty()) {
      RCLCPP_INFO(get_logger(), "Completed inspection contains no findings; no target selected");
      return;
    }

    std::set<uint32_t> seen_ids;
    for (const auto & finding : msg->findings) {
      if (!seen_ids.insert(finding.id).second) {
        RCLCPP_WARN(get_logger(),
            "Rejected inspection: duplicate inspection-local finding ID %u", finding.id);
        return;
      }
    }

    const Finding * best = nullptr;
    for (const auto & finding : msg->findings) {
      if (!validFinding(finding)) {
        continue;
      }
      if (!best || finding.confidence > best->confidence ||
          (finding.confidence == best->confidence && finding.id < best->id)) {
        best = &finding;
      }
    }
    if (!best) {
      RCLCPP_WARN(get_logger(),
          "No eligible plaque/stain target among %zu findings (threshold %.2f)",
          msg->findings.size(), confidence_threshold_);
      return;
    }

    Target selected;
    selected.header = msg->header;  // preserve acquisition time
    selected.header.frame_id = base_frame_;
    selected.finding = *best;
    normalize(selected.finding.surface_normal);
    pending_ = Pending{selected, msg->header.frame_id, std::chrono::steady_clock::now()};

    // Fast path when TF is available. The timer handles late camera TF without
    // blocking the subscription callback or fabricating a static calibration.
    tryPublishPending();
  }

  void tryPublishPending()
  {
    if (!pending_) {
      return;
    }
    auto & pending = *pending_;
    Target converted = pending.target;

    try {
      if (pending.source_frame != base_frame_) {
        // Query at the CAMERA ACQUISITION timestamp; using TimePointZero
        // could give a different target if the camera/teeth have moved.
        const auto tf = tf_buffer_->lookupTransform(
            base_frame_, pending.source_frame,
            rclcpp::Time(converted.header.stamp, get_clock()->get_clock_type()));

        geometry_msgs::msg::PointStamped input_point, output_point;
        input_point.header.frame_id = pending.source_frame;
        input_point.header.stamp = converted.header.stamp;
        input_point.point = converted.finding.position;
        tf2::doTransform(input_point, output_point, tf);

        geometry_msgs::msg::Vector3Stamped input_normal, output_normal;
        input_normal.header = input_point.header;
        input_normal.vector = converted.finding.surface_normal;
        // Vector3Stamped transforms by ROTATION ONLY: translation must never
        // be applied to an outward surface normal.
        tf2::doTransform(input_normal, output_normal, tf);
        converted.finding.position = output_point.point;
        converted.finding.surface_normal = output_normal.vector;
      }
    } catch (const tf2::TransformException & error) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - pending.received_at).count();
      if (elapsed >= transform_wait_sec_) {
        RCLCPP_WARN(get_logger(),
            "No TF %s <- %s at inspection time after %.1fs; dropped target: %s",
            base_frame_.c_str(), pending.source_frame.c_str(), elapsed, error.what());
        pending_.reset();
      }
      return;
    }

    const double magnitude = normalLength(converted.finding.surface_normal);
    if (!finitePoint(converted.finding.position) ||
        !finiteNormal(converted.finding.surface_normal) ||
        !std::isfinite(magnitude) || magnitude < 1e-9) {
      RCLCPP_ERROR(get_logger(), "TF result contains invalid position/normal; target dropped");
      pending_.reset();
      return;
    }
    normalize(converted.finding.surface_normal);

    const double latest_age = (get_clock()->now() -
        rclcpp::Time(converted.header.stamp, get_clock()->get_clock_type())).seconds();
    if (max_inspection_age_sec_ > 0.0 && latest_age > max_inspection_age_sec_) {
      RCLCPP_WARN(get_logger(), "Target became stale during TF lookup (%.2fs); dropped",
          latest_age);
      pending_.reset();
      return;
    }

    targets_pub_->publish(converted);
    publishMarkers(converted);
    RCLCPP_INFO(get_logger(),
        "Selected inspection-local ID=%u (type=%u, confidence=%.2f): "
        "%s -> %s, position=(%.3f, %.3f, %.3f)m",
        converted.finding.id, static_cast<unsigned>(converted.finding.issue_type),
        converted.finding.confidence, pending.source_frame.c_str(),
        base_frame_.c_str(), converted.finding.position.x,
        converted.finding.position.y, converted.finding.position.z);
    pending_.reset();
  }

  void clearMarkers()
  {
    MarkerArray markers;
    Marker clear;
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);
    markers_pub_->publish(markers);
  }

  void publishMarkers(const Target & target)
  {
    MarkerArray markers;
    Marker dot;
    dot.header = target.header;
    dot.ns = "dental_selected_target";
    dot.id = 0;
    dot.type = Marker::SPHERE;
    dot.action = Marker::ADD;
    dot.pose.orientation.w = 1.0;
    dot.pose.position = target.finding.position;
    dot.scale.x = 0.012;
    dot.scale.y = 0.012;
    dot.scale.z = 0.012;
    dot.color.r = 0.1f;
    dot.color.g = 0.9f;
    dot.color.b = 0.2f;
    dot.color.a = 1.0f;
    markers.markers.push_back(dot);

    Marker arrow;
    arrow.header = target.header;
    arrow.ns = "dental_selected_normal";
    arrow.id = 1;
    arrow.type = Marker::ARROW;
    arrow.action = Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.points.push_back(target.finding.position);
    auto tip = target.finding.position;
    tip.x += target.finding.surface_normal.x * 0.035;
    tip.y += target.finding.surface_normal.y * 0.035;
    tip.z += target.finding.surface_normal.z * 0.035;
    arrow.points.push_back(tip);
    arrow.scale.x = 0.003;
    arrow.scale.y = 0.008;
    arrow.scale.z = 0.008;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.7f;
    arrow.color.b = 0.1f;
    arrow.color.a = 1.0f;
    markers.markers.push_back(arrow);

    markers_pub_->publish(markers);
  }

  std::string base_frame_;
  double confidence_threshold_{};
  double max_inspection_age_sec_{};
  double transform_wait_sec_{};
  std::optional<Pending> pending_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<Target>::SharedPtr targets_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<FindingArray>::SharedPtr findings_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TargetManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("target_manager"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
