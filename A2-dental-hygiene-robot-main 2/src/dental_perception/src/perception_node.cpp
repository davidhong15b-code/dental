#include <memory>
#include <string>

#include "dental_interfaces/action/inspect_teeth.hpp"
#include "dental_interfaces/msg/dental_finding_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

class PerceptionNode : public rclcpp::Node {
 public:
  using Inspect = dental_interfaces::action::InspectTeeth;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Inspect>;

  PerceptionNode() : Node("perception") {
    const auto rgb_topic = declare_parameter<std::string>(
        "rgb_topic", "/camera/color/image_raw");
    const auto depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/aligned_depth_to_color/image_raw");
    const auto camera_info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/camera/color/camera_info");

    // These are actual sensor input endpoints. They do NOT generate test data.
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          (void)msg;
          // TODO(perception): synchronize RGB, depth and camera intrinsics,
          // detect issues, estimate metric 3D points/normals, publish findings.
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { (void)msg; });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { (void)msg; });

    findings_pub_ = create_publisher<dental_interfaces::msg::DentalFindingArray>(
        "/dental/perception/findings", rclcpp::QoS(10));

    // Deliberately REJECTS inspection goals until real inspection is implemented.
    inspect_server_ = rclcpp_action::create_server<Inspect>(
        this, "/dental/perception/inspect",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const Inspect::Goal>) {
          RCLCPP_WARN(get_logger(), "InspectTeeth unavailable: perception not implemented");
          return rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle>) {});
    RCLCPP_INFO(get_logger(),
        "Perception scaffold ready; expecting camera topics. No detections are generated.");
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<dental_interfaces::msg::DentalFindingArray>::SharedPtr findings_pub_;
  rclcpp_action::Server<Inspect>::SharedPtr inspect_server_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
