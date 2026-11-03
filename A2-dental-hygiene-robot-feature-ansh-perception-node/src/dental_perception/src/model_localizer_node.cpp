// MTRN4231 Gateway 2: overhead camera -> four-marker planar PnP -> jaw and teeth TF.
// Measurement-only node. NO invented camera extrinsics or fiducial placement.
// Marker centres/yaws are model-CAD-frame coordinates, not image coordinates.
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
namespace {
constexpr double kToothTopM = 0.031;  // checked against docs/Jaw_model.stl in CAD mm
const std::array<double, 4> kToothX{{-0.021, -0.007, 0.007, 0.021}};

bool finite(const std::vector<double>& v) {
  return std::all_of(v.begin(), v.end(), [](double x){return std::isfinite(x);});
}
std::vector<cv::Point3f> markerCorners(double x, double y, double z, double yaw,
                                       double length) {
  const double c = std::cos(yaw), s = std::sin(yaw), h = length / 2.0;
  // ArUco detected corner order: TL, TR, BR, BL. Model +Y is marker-up.
  std::vector<cv::Point3f> result;
  for (const auto& offset : std::array<std::array<double, 2>, 4>{{
          {{-h, h}}, {{h, h}}, {{h, -h}}, {{-h, -h}}}}) {
    result.emplace_back(x + c*offset[0] - s*offset[1],
                        y + s*offset[0] + c*offset[1], z);
  }
  return result;
}
}  // namespace

class ModelLocalizer : public rclcpp::Node {
public:
  ModelLocalizer() : Node("dental_model_localizer") {
    // Two normal operating modes: camera (USB) or image (pre-recorded files).
    // The legacy ROS topic mode is retained for compatibility with existing launch files.
    input_mode_ = declare_parameter<std::string>("overhead_input_mode", "image");
    image_files_ = declare_parameter<std::vector<std::string>>(
        "overhead_images", {"samples/overhead_01.png", "samples/overhead_02.png"});
    image_interval_sec_ = declare_parameter<double>("overhead_image_interval_sec", 1.0);
    publish_offline_pose_ = declare_parameter<bool>("publish_offline_model_pose", false);
    device_path_ = declare_parameter<std::string>("overhead_device", "/dev/dental_overhead");
    camera_frame_ = declare_parameter<std::string>("overhead_optical_frame", "overhead_color_optical_frame");
    camera_width_ = declare_parameter<int>("overhead_width", 640);
    camera_height_ = declare_parameter<int>("overhead_height", 480);
    camera_fps_ = declare_parameter<int>("overhead_fps", 15);
    camera_matrix_ = declare_parameter<std::vector<double>>(
        "overhead_camera_matrix", std::vector<double>(9, 0.0));
    distortion_ = declare_parameter<std::vector<double>>(
        "overhead_distortion", std::vector<double>(5, 0.0));
    camera_calibrated_ = declare_parameter<bool>("overhead_intrinsics_calibrated", false);
    if (input_mode_ != "ros" && input_mode_ != "camera" && input_mode_ != "image")
      throw std::invalid_argument("overhead_input_mode must be camera, image, or ros");
    if (image_interval_sec_ <= 0 || image_files_.empty())
      throw std::invalid_argument("overhead_images must be populated; interval must be positive");
    if (camera_width_ <= 0 || camera_height_ <= 0 || camera_fps_ <= 0 ||
        camera_frame_.empty() || device_path_.empty() || camera_matrix_.size() != 9 ||
        !(distortion_.size() == 0 || distortion_.size() == 4 ||
          distortion_.size() == 5 || distortion_.size() == 8) ||
        !finite(camera_matrix_) || !finite(distortion_))
      throw std::invalid_argument("Invalid V4L2 overhead camera configuration");
    if (input_mode_ != "ros" && camera_calibrated_ &&
        (camera_matrix_[0] <= 0 || camera_matrix_[4] <= 0 ||
         std::abs(camera_matrix_[8] - 1.0) > 1e-6))
      throw std::invalid_argument("Calibrated overhead intrinsics must have fx/fy>0, K[8]=1");
    overhead_image_topic_ = declare_parameter<std::string>(
        "overhead_image_topic", "/overhead/color/image_raw");
    const auto camera_info_topic = declare_parameter<std::string>(
        "overhead_camera_info_topic", "/overhead/color/camera_info");
    base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    model_frame_ = declare_parameter<std::string>("model_frame", "dental_model");
    // LAB: publish the jaw relative to the calibrated overhead camera, even if
    // the UR5e is powered off. For execution, use the calibrated base transform.
    // "camera" never invents or publishes a base_link -> camera transform.
    model_pose_reference_ = declare_parameter<std::string>("model_pose_reference", "base");
    if (model_pose_reference_ != "camera" && model_pose_reference_ != "base") {
      throw std::invalid_argument("model_pose_reference must be camera or base");
    }
    calibrated_ = declare_parameter<bool>("marker_layout_calibrated", false);
    marker_size_m_ = declare_parameter<double>("marker_size_m", 0.0);
    centres_ = declare_parameter<std::vector<double>>(
        "marker_centres_xyz_m", std::vector<double>(12, 0.0));
    yaws_ = declare_parameter<std::vector<double>>(
        "marker_yaw_rad", std::vector<double>(4, 0.0));
    max_reprojection_px_ = declare_parameter<double>("max_reprojection_error_px", 2.5);
    dictionary_id_ = declare_parameter<int>("aruco_dictionary", cv::aruco::DICT_ARUCO_ORIGINAL);
    if (base_frame_.empty() || model_frame_.empty() || base_frame_ == model_frame_ ||
        centres_.size() != 12 || yaws_.size() != 4 || !finite(centres_) ||
        !finite(yaws_) || !std::isfinite(max_reprojection_px_) || max_reprojection_px_ <= 0 ||
        dictionary_id_ < 0 || dictionary_id_ > cv::aruco::DICT_ARUCO_ORIGINAL) {
      throw std::invalid_argument("Invalid model localizer parameters");
    }
    if (calibrated_ && (!std::isfinite(marker_size_m_) || marker_size_m_ <= 0)) {
      throw std::invalid_argument("Calibrated marker side length must be positive, in metres");
    }
    if (calibrated_) {
      // Markers must occupy distinct measured positions; do not use repeated zero defaults.
      for (int a=0; a<4; ++a) for (int b=a+1; b<4; ++b) {
        const double d = std::hypot(centres_[3*a]-centres_[3*b],
                                    centres_[3*a+1]-centres_[3*b+1]);
        if (d < marker_size_m_*0.8)
          throw std::invalid_argument("Marker layout overlaps; check measured centres");
      }
    }
    try {
      dictionary_ = cv::aruco::getPredefinedDictionary(
          static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id_));
    } catch (const cv::Exception & e) { throw std::invalid_argument(e.what()); }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/dental/perception/model_pose", rclcpp::QoS(10));
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/dental/perception/model_localized", rclcpp::QoS(10));
    preview_pub_ = create_publisher<Image>("/dental/perception/overhead_preview", rclcpp::QoS(2));
    marker_ids_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
        "/dental/perception/marker_ids", rclcpp::QoS(10));
    if (input_mode_ == "ros") {
      camera_info_sub_ = create_subscription<CameraInfo>(camera_info_topic,
          rclcpp::SensorDataQoS(), [this](CameraInfo::ConstSharedPtr msg) { info_ = msg; });
      image_sub_ = create_subscription<Image>(overhead_image_topic_, rclcpp::SensorDataQoS(),
          [this](Image::ConstSharedPtr msg) { onImage(*msg); });
    } else if (input_mode_ == "camera") {
      camera_thread_ = std::thread([this] { captureLoop(); });
    } else {
      camera_thread_ = std::thread([this] { imageLoop(); });
    }
    publishToothFrames();
    RCLCPP_INFO(get_logger(), "ArUco localizer: mode=%s input=%s model=%s reference=%s marker_layout=%s intrinsics=%s",
        input_mode_.c_str(), (input_mode_ == "ros" ? overhead_image_topic_ :
                                 input_mode_ == "camera" ? device_path_ : "sample images").c_str(),
        model_frame_.c_str(), model_pose_reference_.c_str(),
        calibrated_ ? "yes" : "NO", camera_calibrated_ ? "yes" : "NO");
  }
  ~ModelLocalizer() override {
    stop_camera_.store(true);
    if (camera_thread_.joinable()) camera_thread_.join();
  }
private:
  void captureLoop() {
    using namespace std::chrono_literals;
    while (!stop_camera_.load() && rclcpp::ok()) {
      cv::VideoCapture cap;
      try {
        if (!cap.open(device_path_, cv::CAP_V4L2))
          throw std::runtime_error("Cannot open " + device_path_);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
        cap.set(cv::CAP_PROP_FPS, camera_fps_);
        const int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        const int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        if (w != camera_width_ || h != camera_height_)
          throw std::runtime_error("V4L2 negotiated " + std::to_string(w) + "x" +
              std::to_string(h) + ", not calibrated " + std::to_string(camera_width_) +
              "x" + std::to_string(camera_height_));
        auto info = std::make_shared<CameraInfo>();
        info->header.frame_id = camera_frame_;
        info->width = static_cast<uint32_t>(w);
        info->height = static_cast<uint32_t>(h);
        info->distortion_model = "plumb_bob";
        info->k = {camera_matrix_[0],camera_matrix_[1],camera_matrix_[2],
                   camera_matrix_[3],camera_matrix_[4],camera_matrix_[5],
                   camera_matrix_[6],camera_matrix_[7],camera_matrix_[8]};
        info->d = distortion_;
        RCLCPP_INFO(get_logger(), "Overhead V4L2 stream connected: %s (%dx%d)",
                    device_path_.c_str(), w, h);
        int failures = 0;
        while (!stop_camera_.load() && rclcpp::ok()) {
          cv::Mat bgr;
          if (!cap.read(bgr) || bgr.empty() || bgr.size() != cv::Size(w,h)) {
            if (++failures >= 5) throw std::runtime_error("Overhead stream lost");
            continue;
          }
          failures = 0;
          std_msgs::msg::Header header;
          header.stamp = now(); // ROS clock, consistent with tf2 and the arm state.
          header.frame_id = camera_frame_;
          info->header.stamp = header.stamp;
          info_ = info;  // only capture thread sets this in direct mode.
          const auto msg = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
          onImage(*msg);  // same detection pipeline as file mode.
        }
      } catch (const std::exception& e) {
        if (!stop_camera_.load())
          RCLCPP_WARN(get_logger(), "Overhead camera: %s; retrying", e.what());
      }
      cap.release();
      for (int n=0; n<10 && !stop_camera_.load() && rclcpp::ok(); ++n)
        std::this_thread::sleep_for(100ms);
    }
  }
  static std::string samplePath(const std::string& name) {
    const std::filesystem::path file(name);
    if (file.is_absolute()) return file.string();
    return (std::filesystem::path(
        ament_index_cpp::get_package_share_directory("dental_perception")) / file).string();
  }

  void imageLoop() {
    // File playback simulates new camera acquisitions for repeatable OFFLINE testing.
    // Publishing an image-based model TF is disabled by default: stale photographs
    // must never be interpreted as a live robot workcell measurement.
    std::vector<cv::Mat> images;
    for (const auto& file : image_files_) {
      auto frame = cv::imread(samplePath(file), cv::IMREAD_COLOR);
      if (frame.empty()) {
        RCLCPP_ERROR(get_logger(), "Cannot load overhead image: %s", samplePath(file).c_str());
        return;
      }
      images.push_back(frame);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu overhead image(s)", images.size());
    size_t index = 0;
    while (!stop_camera_.load() && rclcpp::ok()) {
      const cv::Mat& frame = images[index];
      auto info = std::make_shared<CameraInfo>();
      info->header.frame_id = camera_frame_;
      info->width = static_cast<uint32_t>(frame.cols);
      info->height = static_cast<uint32_t>(frame.rows);
      info->distortion_model = "plumb_bob";
      info->k = {camera_matrix_[0],camera_matrix_[1],camera_matrix_[2],
                 camera_matrix_[3],camera_matrix_[4],camera_matrix_[5],
                 camera_matrix_[6],camera_matrix_[7],camera_matrix_[8]};
      info->d = distortion_;
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = camera_frame_;
      info->header.stamp = header.stamp;
      info_ = info;
      onImage(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
      index = (index + 1) % images.size();
      const int steps = std::max(1, static_cast<int>(image_interval_sec_ * 20));
      for (int step = 0; step < steps && !stop_camera_.load() && rclcpp::ok(); ++step)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void publishToothFrames() {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (size_t i=0; i<kToothX.size(); ++i) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = model_frame_;
      t.child_frame_id = "tooth_" + std::to_string(i+1);
      t.transform.translation.x = kToothX[i];
      t.transform.translation.z = kToothTopM;
      t.transform.rotation.w = 1.0;
      transforms.push_back(t);
    }
    static_broadcaster_->sendTransform(transforms);
  }
  void onImage(const Image & img) {
    auto announce = [this](bool ok) {
      std_msgs::msg::Bool msg; msg.data = ok; valid_pub_->publish(msg);
    };
    // ArUco is detected and displayed even when the workcell isn't calibrated.
    // This lets the supplied JPG/PNG images be used immediately for a real CV test.
    cv::Mat rgb;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    try {
      rgb = cv_bridge::toCvCopy(img, "bgr8")->image;
      cv::aruco::detectMarkers(rgb, dictionary_, corners, ids);
      std_msgs::msg::Int32MultiArray ids_msg;
      for (int id : ids) ids_msg.data.push_back(id);
      marker_ids_pub_->publish(ids_msg);
      if (input_mode_ == "image" || (++preview_counter_ % std::max(1,camera_fps_) == 0)) {
        cv::Mat annotated = rgb.clone();
        if (!ids.empty()) cv::aruco::drawDetectedMarkers(annotated, corners, ids);
        // Cyan arrows are each marker's canonical +X (from left to right in
        // the decoded marker frame). Use these to determine LAB marker_yaw_rad.
        for (const auto& c : corners) {
          const cv::Point2f center = (c[0] + c[1] + c[2] + c[3]) * 0.25f;
          const cv::Point2f right = (c[1] + c[2]) * 0.5f;
          cv::arrowedLine(annotated, center, right,
              cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.25);
        }
        cv::putText(annotated, "ArUco IDs: " + std::to_string(ids.size()),
            cv::Point(20,45), cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0,255,0), 2);
        preview_pub_->publish(*cv_bridge::CvImage(img.header,"bgr8",annotated).toImageMsg());
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "ArUco image error: %s", error.what());
      announce(false); return;
    }
    if (!calibrated_ || (input_mode_ != "ros" && !camera_calibrated_) ||
        (input_mode_ == "image" && !publish_offline_pose_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Marker IDs visible, but metric jaw TF disabled (see lab measurements and image safety settings)");
      announce(false); return;
    }
    if (!info_ || info_->header.frame_id != img.header.frame_id ||
        info_->width != img.width || info_->height != img.height ||
        img.header.frame_id.empty() || img.header.stamp.sec == 0) {
      announce(false); return;
    }
    try {
      // Require exactly one instance of each required ID. Other markers may be present.
      std::array<int,4> indices{{-1,-1,-1,-1}};
      for (size_t i=0; i<ids.size(); ++i) {
        if (ids[i]>=0 && ids[i]<=3) {
          if (indices[ids[i]]!=-1) {announce(false); return;}
          indices[ids[i]] = static_cast<int>(i);
        }
      }
      if (std::any_of(indices.begin(), indices.end(), [](int v){return v<0;})) {
        announce(false); return;
      }
      cv::Mat K = (cv::Mat_<double>(3,3) <<
          info_->k[0],info_->k[1],info_->k[2],
          info_->k[3],info_->k[4],info_->k[5],
          info_->k[6],info_->k[7],info_->k[8]);
      if (!(info_->k[0]>0 && info_->k[4]>0)) {announce(false); return;}
      cv::Mat dist(info_->d, true); // row/column vector containing actual lens coefficients
      if (info_->distortion_model != "plumb_bob" &&
          info_->distortion_model != "rational_polynomial" &&
          !(info_->distortion_model.empty() && info_->d.empty())) {
        announce(false); return;
      }
      std::vector<cv::Point3f> object_pts;
      std::vector<cv::Point2f> image_pts;
      for (int id=0; id<4; ++id) {
        auto model_corners = markerCorners(centres_[3*id], centres_[3*id+1],
                                           centres_[3*id+2], yaws_[id], marker_size_m_);
        object_pts.insert(object_pts.end(), model_corners.begin(), model_corners.end());
        const auto& detected_corners = corners[indices[id]];
        image_pts.insert(image_pts.end(), detected_corners.begin(), detected_corners.end());
      }
      cv::Mat rvec, tvec, inliers;
      if (!cv::solvePnPRansac(object_pts, image_pts, K, dist, rvec, tvec,
                             false, 100, max_reprojection_px_, 0.99, inliers,
                             cv::SOLVEPNP_ITERATIVE) || inliers.rows < 12) {
        announce(false); return;
      }
      std::vector<cv::Point2f> projected;
      cv::projectPoints(object_pts, rvec, tvec, K, dist, projected);
      double squared_error=0;
      for (size_t i=0; i<image_pts.size(); ++i) {
        const double dx=projected[i].x-image_pts[i].x;
        const double dy=projected[i].y-image_pts[i].y;
        squared_error += dx*dx+dy*dy;
      }
      const double rms=std::sqrt(squared_error/image_pts.size());
      if (!std::isfinite(rms) || rms>max_reprojection_px_ || tvec.at<double>(2)<=0) {
        announce(false); return;
      }
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      tf2::Matrix3x3 basis(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                           R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                           R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
      tf2::Quaternion q;
      basis.getRotation(q); q.normalize();
      tf2::Transform camera_T_model(q, tf2::Vector3(tvec.at<double>(0),
                                                     tvec.at<double>(1),tvec.at<double>(2)));
      tf2::Transform reference_T_model;
      std::string reference_frame;
      if (model_pose_reference_ == "camera") {
        // Offline lab calibration: this is an OBSERVED optical-frame pose.
        // Do not invent base_link or assume the robot is running.
        reference_frame = img.header.frame_id;
        reference_T_model = camera_T_model;
        // The CAD tooth-top (+Z) must face the viewing camera, rather than
        // pointing through the table. This rejects planar PnP mirror poses.
        const tf2::Vector3 normal_in_camera =
            camera_T_model.getBasis() * tf2::Vector3(0,0,1);
        const tf2::Vector3 camera_to_model = camera_T_model.getOrigin().normalized();
        if (normal_in_camera.dot(camera_to_model) > -0.20) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
              "Rejected camera-frame model pose: CAD +Z is not facing overhead camera");
          announce(false); return;
        }
      } else {
        // Robot-relative localisation requires a REAL calibrated TF.
        auto base_T_camera_msg = tf_buffer_->lookupTransform(
            base_frame_, img.header.frame_id, rclcpp::Time(img.header.stamp),
            rclcpp::Duration::from_seconds(0.2));
        tf2::Transform base_T_camera;
        tf2::fromMsg(base_T_camera_msg.transform, base_T_camera);
        reference_T_model = base_T_camera * camera_T_model;
        reference_frame = base_frame_;
        // In the UR base frame, CAD +Z must physically point upwards.
        const tf2::Vector3 model_z =
            reference_T_model.getBasis() * tf2::Vector3(0,0,1);
        if (model_z.z() < 0.65) {announce(false); return;}
      }
      geometry_msgs::msg::TransformStamped transform;
      transform.header = img.header;
      transform.header.frame_id = reference_frame;
      transform.child_frame_id = model_frame_;
      transform.transform = tf2::toMsg(reference_T_model);
      tf_broadcaster_->sendTransform(transform);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = reference_T_model.getOrigin().x();
      pose.pose.position.y = reference_T_model.getOrigin().y();
      pose.pose.position.z = reference_T_model.getOrigin().z();
      pose.pose.orientation = transform.transform.rotation;
      pose_pub_->publish(pose);
      announce(true);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "Four markers localized jaw in %s: RMS=%.2f px, depth=%.3f m",
          reference_frame.c_str(), rms, tvec.at<double>(2));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Model localization failed: %s", e.what());
      announce(false);
    }
  }
  std::string input_mode_, device_path_, camera_frame_;
  std::vector<std::string> image_files_;
  double image_interval_sec_{};
  bool publish_offline_pose_{false};
  uint64_t preview_counter_{0};
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr marker_ids_pub_;
  int camera_width_{}, camera_height_{}, camera_fps_{};
  std::vector<double> camera_matrix_, distortion_;
  bool camera_calibrated_{};
  std::atomic<bool> stop_camera_{false};
  std::thread camera_thread_;
  rclcpp::Publisher<Image>::SharedPtr preview_pub_;
  std::string overhead_image_topic_, base_frame_, model_frame_, model_pose_reference_;
  bool calibrated_{};
  double marker_size_m_{}, max_reprojection_px_{};
  int dictionary_id_{};
  std::vector<double> centres_, yaws_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  CameraInfo::ConstSharedPtr info_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ModelLocalizer>());
  rclcpp::shutdown();
  return 0;
}
