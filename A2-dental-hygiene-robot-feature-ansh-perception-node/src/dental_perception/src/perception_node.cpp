// MTRN4231: fresh RGB-D inspection of four measured dental model tooth regions.
// The only target outputs are actual depth-backed dark regions, in RGB optical coordinates.
// A failed action NEVER publishes an empty success array (which would mean "clean").
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#ifdef DENTAL_HAVE_REALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#endif
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "dental_interfaces/action/inspect_teeth.hpp"
#include "dental_interfaces/msg/dental_finding.hpp"
#include "dental_interfaces/msg/dental_finding_array.hpp"

using namespace std::chrono_literals;
namespace {
using Inspect = dental_interfaces::action::InspectTeeth;
using Finding = dental_interfaces::msg::DentalFinding;
using Array = dental_interfaces::msg::DentalFindingArray;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Clock = std::chrono::steady_clock;

bool validStamp(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec > 0 && stamp.nanosec < 1000000000u;
}
int64_t stampNs(const builtin_interfaces::msg::Time & t) {
  return static_cast<int64_t>(t.sec)*1000000000LL + t.nanosec;
}
cv::Mat cameraMatrix(const CameraInfo & i) {
  return (cv::Mat_<double>(3,3) << i.k[0],i.k[1],i.k[2],
                                     i.k[3],i.k[4],i.k[5],
                                     i.k[6],i.k[7],i.k[8]);
}
double depthMetres(const cv::Mat& depth, int x, int y) {
  if (x<0 || y<0 || x>=depth.cols || y>=depth.rows) return 0;
  if (depth.type()==CV_16UC1) return depth.at<uint16_t>(y,x)*0.001;
  if (depth.type()==CV_32FC1) return depth.at<float>(y,x);
  return 0;
}
}  // namespace

class PerceptionNode final : public rclcpp::Node {
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<Inspect>;
  PerceptionNode() : Node("perception") {
    input_mode_ = declare_parameter<std::string>("rgbd_input_mode", "image");
    rgbd_images_ = declare_parameter<std::vector<std::string>>(
        "rgbd_images", {"samples/wrist_01.png", "samples/wrist_02.png"});
    rgbd_depth_images_ = declare_parameter<std::vector<std::string>>(
        "rgbd_depth_images", std::vector<std::string>{});
    rgbd_image_interval_sec_ = declare_parameter<double>("rgbd_image_interval_sec", 1.0);
    image_camera_calibrated_ = declare_parameter<bool>("rgbd_image_intrinsics_calibrated", false);
    image_camera_matrix_ = declare_parameter<std::vector<double>>(
        "rgbd_image_camera_matrix", std::vector<double>(9,0.0));
    image_distortion_ = declare_parameter<std::vector<double>>(
        "rgbd_image_distortion", std::vector<double>(5,0.0));
    allow_image_findings_ = declare_parameter<bool>("allow_offline_findings", false);
    cad_plane_fallback_ = declare_parameter<bool>("allow_cad_plane_fallback", false);
    preview_dark_threshold_ = declare_parameter<int>("preview_dark_threshold", 95);
    preview_min_blob_area_ = declare_parameter<int>("preview_min_blob_area", 30);
    depth_camera_serial_ = declare_parameter<std::string>("rgbd_serial", "");
    rgbd_optical_frame_ = declare_parameter<std::string>(
        "rgbd_optical_frame", "rgbd_color_optical_frame");
    color_width_ = declare_parameter<int>("rgbd_color_width", 640);
    color_height_ = declare_parameter<int>("rgbd_color_height", 480);
    depth_width_ = declare_parameter<int>("rgbd_depth_width", 640);
    depth_height_ = declare_parameter<int>("rgbd_depth_height", 480);
    rgbd_fps_ = declare_parameter<int>("rgbd_fps", 15);
    if ((input_mode_ != "ros" && input_mode_ != "camera" && input_mode_ != "image") ||
        rgbd_optical_frame_.empty() || color_width_ <= 0 || color_height_ <= 0 ||
        depth_width_ <= 0 || depth_height_ <= 0 || rgbd_fps_ <= 0 ||
        rgbd_image_interval_sec_ <= 0 || rgbd_images_.empty() ||
        (rgbd_depth_images_.size() != 0 && rgbd_depth_images_.size() != rgbd_images_.size()) ||
        image_camera_matrix_.size() != 9 || preview_dark_threshold_ < 0 ||
        preview_dark_threshold_ > 255 || preview_min_blob_area_ < 1)
      throw std::invalid_argument("Invalid RGB-D backend/stream parameters");
#ifndef DENTAL_HAVE_REALSENSE
    if (input_mode_ == "camera")
      throw std::runtime_error("rgbd_input_mode=camera requires librealsense2 SDK and rebuild");
#endif
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
        "depth_topic", "/camera/aligned_depth_to_color/image_raw");
    info_topic_ = declare_parameter<std::string>(
        "camera_info_topic", "/camera/color/camera_info");
    model_frame_ = declare_parameter<std::string>("model_frame", "dental_model");
    sync_tolerance_sec_ = declare_parameter<double>("rgb_depth_tolerance_sec", 0.06);
    acquisition_timeout_sec_ = declare_parameter<double>("acquisition_timeout_sec", 5.0);
    model_pose_max_age_sec_ = declare_parameter<double>("model_pose_max_age_sec", 2.0);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 2.0);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.10);
    max_surface_error_m_ = declare_parameter<double>("max_surface_error_m", 0.012);
    tooth_half_width_m_ = declare_parameter<double>("tooth_half_width_m", 0.005);
    tooth_half_length_m_ = declare_parameter<double>("tooth_half_length_m", 0.005);
    dark_threshold_ = declare_parameter<int>("dark_threshold", 105);
    min_surface_valid_fraction_ = declare_parameter<double>("min_surface_valid_fraction", 0.30);
    minimum_blob_pixels_ = declare_parameter<int>("min_blob_area_pixels", 12);
    if (sync_tolerance_sec_<=0 || acquisition_timeout_sec_<=0 ||
        model_pose_max_age_sec_<=0 || min_depth_m_<=0 || max_depth_m_<=min_depth_m_ ||
        max_surface_error_m_<=0 || tooth_half_width_m_<=0 || tooth_half_length_m_<=0 ||
        dark_threshold_<0 || dark_threshold_>255 || minimum_blob_pixels_<1 ||
        min_surface_valid_fraction_<=0 || min_surface_valid_fraction_>1) {
      throw std::invalid_argument("Invalid perception parameters");
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (input_mode_ == "ros") {
      rgb_sub_ = create_subscription<Image>(rgb_topic_, rclcpp::SensorDataQoS(),
          [this](Image::ConstSharedPtr img) {append(rgb_frames_, std::move(img));});
      depth_sub_ = create_subscription<Image>(depth_topic_, rclcpp::SensorDataQoS(),
          [this](Image::ConstSharedPtr img) {append(depth_frames_, std::move(img));});
      info_sub_ = create_subscription<CameraInfo>(info_topic_, rclcpp::SensorDataQoS(),
          [this](CameraInfo::ConstSharedPtr info) {
            std::lock_guard<std::mutex> lock(mutex_);
            camera_info_ = std::move(info);
            cv_.notify_all();
          });
    }
    model_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/dental/perception/model_pose", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {
          std::lock_guard<std::mutex> lock(mutex_);
          model_pose_stamp_ = pose->header.stamp;
          model_pose_rx_ = Clock::now();
        });
    model_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/dental/perception/model_localized", rclcpp::QoS(10),
        [this](std_msgs::msg::Bool::ConstSharedPtr status) {
          std::lock_guard<std::mutex> lock(mutex_);
          model_visible_ = status->data;
        });
    findings_pub_ = create_publisher<Array>("/dental/perception/findings", rclcpp::QoS(10));
    debug_pub_ = create_publisher<Image>("/dental/perception/debug", rclcpp::QoS(2));
    rgbd_preview_pub_ = create_publisher<Image>("/dental/perception/rgbd_preview", rclcpp::QoS(2));
    enable_inspection_action_ = declare_parameter<bool>("enable_inspection_action", true);
    if (enable_inspection_action_) {
    inspect_server_ = rclcpp_action::create_server<Inspect>(
        this, "/dental/perception/inspect",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Inspect::Goal> g) {
          const std::string & id = g->region_id;
          if (!(id.empty() || id=="tooth_1" || id=="tooth_2" ||
                id=="tooth_3" || id=="tooth_4"))
            return rclcpp_action::GoalResponse::REJECT;
          bool expected=false;
          if (!busy_.compare_exchange_strong(expected,true))
            return rclcpp_action::GoalResponse::REJECT;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> h) {
          if (worker_.joinable()) worker_.join();
          worker_=std::thread([this,h]{run(h);});
        });
    }
    if (input_mode_ == "camera")
      camera_thread_ = std::thread([this] { cameraLoop(); });
    else if (input_mode_ == "image")
      camera_thread_ = std::thread([this] { imageLoop(); });
    RCLCPP_INFO(get_logger(), "Inspection backend=%s, frame=%s, action=/dental/perception/inspect",
                input_mode_.c_str(), rgbd_optical_frame_.c_str());
  }
  ~PerceptionNode() override {
    stopping_.store(true);
    cv_.notify_all();
    if (camera_thread_.joinable()) camera_thread_.join();
    if (worker_.joinable()) worker_.join();
  }
private:
  static std::string samplePath(const std::string& filename) {
    const std::filesystem::path path(filename);
    if (path.is_absolute()) return path.string();
    return (std::filesystem::path(
        ament_index_cpp::get_package_share_directory("dental_perception")) / path).string();
  }

  // 2D preview only. Normalize the four marker centres to a canonical rectangle
  // so a rotated or perspective-skewed jaw still has a consistent search region.
  // IMPORTANT: these pixel-space candidates are NOT published as 3D robot targets.
  cv::Mat annotateOfflineImage(const cv::Mat& bgr) const {
    cv::Mat debug = bgr.clone();
    const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(bgr, dict, corners, ids);
    if (!ids.empty()) cv::aruco::drawDetectedMarkers(debug, corners, ids);
    std::array<cv::Point2f,4> centres{};
    std::array<bool,4> present{{false,false,false,false}};
    for (size_t i=0; i<ids.size(); ++i) {
      if (ids[i]<0 || ids[i]>3 || present[ids[i]]) continue;
      cv::Point2f c(0,0);
      for (const auto& corner : corners[i]) c += corner;
      centres[ids[i]] = c * 0.25f;
      present[ids[i]] = true;
    }
    if (!std::all_of(present.begin(),present.end(),[](bool b){return b;})) {
      cv::putText(debug,"Need markers 0-3 for 2D preview", cv::Point(30,55),
                  cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,0,255),2);
      return debug;
    }
    // Marker layout as printed in the lab photographs: 2(top-left),
    // 0(top-right), 1(bottom-right), 3(bottom-left).
    std::vector<cv::Point2f> src{centres[2],centres[0],centres[1],centres[3]};
    std::vector<cv::Point2f> dst{{0,0},{640,0},{640,480},{0,480}};
    const cv::Mat transform = cv::getPerspectiveTransform(src,dst);
    cv::Mat aligned;
    cv::warpPerspective(bgr,aligned,transform,cv::Size(640,480));
    cv::Mat gray,binary;
    cv::cvtColor(aligned,gray,cv::COLOR_BGR2GRAY);
    cv::threshold(gray,binary,preview_dark_threshold_,255,cv::THRESH_BINARY_INV);
    cv::Mat roi = cv::Mat::zeros(binary.size(),CV_8UC1);
    // Broad central band covers horizontal AND rotated vertical 4-tooth rows.
    cv::rectangle(roi,cv::Rect(235,90,170,300),cv::Scalar(255),cv::FILLED);
    cv::bitwise_and(binary,roi,binary);
    cv::morphologyEx(binary,binary,cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));
    cv::Mat labels,stats,centroids;
    const int count = cv::connectedComponentsWithStats(binary,labels,stats,centroids,8,CV_32S);
    cv::Mat inverse = transform.inv();
    int blobs=0;
    for (int i=1; i<count; ++i) {
      const int area=stats.at<int>(i,cv::CC_STAT_AREA);
      if (area<preview_min_blob_area_ || area>12000) continue;
      std::vector<cv::Point2f> point{{static_cast<float>(centroids.at<double>(i,0)),
                                     static_cast<float>(centroids.at<double>(i,1))}};
      std::vector<cv::Point2f> pixel;
      cv::perspectiveTransform(point,pixel,inverse);
      cv::circle(debug,pixel.front(),14,cv::Scalar(0,0,255),3);
      ++blobs;
    }
    cv::putText(debug,"2D dark candidates: " + std::to_string(blobs) + " (NO DEPTH)",
                cv::Point(30,55),cv::FONT_HERSHEY_SIMPLEX,0.75,cv::Scalar(0,0,255),2);
    return debug;
  }

  void imageLoop() {
    std::vector<cv::Mat> colors,depths;
    try {
      for (size_t i=0; i<rgbd_images_.size(); ++i) {
        cv::Mat bgr = cv::imread(samplePath(rgbd_images_[i]),cv::IMREAD_COLOR);
        if (bgr.empty()) throw std::runtime_error("Cannot load RGB image: " + rgbd_images_[i]);
        colors.push_back(bgr);
        if (!rgbd_depth_images_.empty()) {
          cv::Mat d = cv::imread(samplePath(rgbd_depth_images_[i]),cv::IMREAD_UNCHANGED);
          if (d.empty() || d.size()!=bgr.size() ||
              (d.type()!=CV_16UC1 && d.type()!=CV_32FC1))
            throw std::runtime_error("Depth image must be registered, same size, 16UC1 mm or 32FC1 m");
          depths.push_back(d);
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Image playback: %s", e.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu RGB photo(s), registered depth=%s. RGB-only is preview-only.",
                colors.size(),depths.empty()?"NO":"YES");
    size_t index=0;
    while (!stopping_.load() && rclcpp::ok()) {
      const cv::Mat& rgb = colors[index];
      std_msgs::msg::Header header;
      header.stamp=now();
      header.frame_id=rgbd_optical_frame_;
      const auto image = cv_bridge::CvImage(header,"bgr8",rgb).toImageMsg();
      rgbd_preview_pub_->publish(
          *cv_bridge::CvImage(header,"bgr8",annotateOfflineImage(rgb)).toImageMsg());
      if (!depths.empty() && image_camera_calibrated_) {
        const cv::Mat& depth=depths[index];
        auto info=std::make_shared<CameraInfo>();
        info->header=header;
        info->width=static_cast<uint32_t>(rgb.cols);
        info->height=static_cast<uint32_t>(rgb.rows);
        info->distortion_model="plumb_bob";
        info->k={image_camera_matrix_[0],image_camera_matrix_[1],image_camera_matrix_[2],
                 image_camera_matrix_[3],image_camera_matrix_[4],image_camera_matrix_[5],
                 image_camera_matrix_[6],image_camera_matrix_[7],image_camera_matrix_[8]};
        info->d=image_distortion_;
        if (info->k[0]>0 && info->k[4]>0) {
          std::lock_guard<std::mutex> lock(mutex_);
          camera_info_=info;
          rgb_frames_.push_back(image);
          depth_frames_.push_back(cv_bridge::CvImage(header,
              depth.type()==CV_16UC1 ? sensor_msgs::image_encodings::TYPE_16UC1 :
                                       sensor_msgs::image_encodings::TYPE_32FC1,
              depth).toImageMsg());
          while (rgb_frames_.size()>4) rgb_frames_.pop_front();
          while (depth_frames_.size()>4) depth_frames_.pop_front();
          cv_.notify_all();
        }
      }
      index=(index+1)%colors.size();
      const int steps=std::max(1,static_cast<int>(rgbd_image_interval_sec_*20));
      for (int k=0;k<steps && !stopping_.load() && rclcpp::ok();++k)
        std::this_thread::sleep_for(50ms);
    }
  }

  void cameraLoop() {
#ifdef DENTAL_HAVE_REALSENSE
    using namespace std::chrono_literals;
    while (!stopping_.load() && rclcpp::ok()) {
      try {
        rs2::context context;
        const auto devices = context.query_devices();
        if (depth_camera_serial_.empty() && devices.size() != 1)
          throw std::runtime_error("Set rgbd_serial: detected " +
              std::to_string(devices.size()) + " RealSense devices (requires exactly 1 otherwise)");
        rs2::pipeline pipeline(context);
        rs2::config config;
        if (!depth_camera_serial_.empty()) config.enable_device(depth_camera_serial_);
        config.enable_stream(RS2_STREAM_COLOR, color_width_, color_height_,
                             RS2_FORMAT_BGR8, rgbd_fps_);
        config.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_,
                             RS2_FORMAT_Z16, rgbd_fps_);
        pipeline.start(config);
        rs2::align align_to_color(RS2_STREAM_COLOR);
        RCLCPP_INFO(get_logger(), "RealSense connected (serial=%s); RGB-D aligned to color",
                    depth_camera_serial_.empty() ? "auto" : depth_camera_serial_.c_str());
        int preview_counter = 0;
        auto last_frames = Clock::now();
        cv::Mat rectify_map_x, rectify_map_y;
        while (!stopping_.load() && rclcpp::ok()) {
          rs2::frameset frames;
          if (!pipeline.poll_for_frames(&frames)) {
            // Poll rather than block indefinitely so unplug/shutdown can stop cleanly.
            if (std::chrono::duration<double>(Clock::now()-last_frames).count() > 3.0)
              throw std::runtime_error("No RealSense frames for 3 seconds: reconnecting");
            std::this_thread::sleep_for(5ms);
            continue;
          }
          last_frames = Clock::now();
          const auto aligned = align_to_color.process(frames);
          const rs2::video_frame color = aligned.get_color_frame();
          const rs2::depth_frame depth = aligned.get_depth_frame();
          if (!color || !depth || color.get_width() != depth.get_width() ||
              color.get_height() != depth.get_height()) continue;
          if (color.get_frame_timestamp_domain() == depth.get_frame_timestamp_domain() &&
              std::abs(color.get_timestamp() - depth.get_timestamp()) >
                  sync_tolerance_sec_ * 1000.0) continue;
          const auto intr = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
          if (intr.fx <= 0 || intr.fy <= 0 ||
              intr.width != color.get_width() || intr.height != color.get_height())
            throw std::runtime_error("RealSense color intrinsics are invalid");
          // Rectify SDK-specific distortions (including D4xx inverse Brown-Conrady)
          // into a virtual pinhole camera. Reuse one map for the active stream.
          // Both the color and SDK color-aligned depth are remapped identically.
          if (intr.model != RS2_DISTORTION_NONE && rectify_map_x.empty()) {
            rectify_map_x.create(intr.height, intr.width, CV_32FC1);
            rectify_map_y.create(intr.height, intr.width, CV_32FC1);
            for (int y=0; y<intr.height; ++y) {
              for (int x=0; x<intr.width; ++x) {
                const float ray[3] = {
                    (x-intr.ppx)/intr.fx, (y-intr.ppy)/intr.fy, 1.0f};
                float source_pixel[2];
                rs2_project_point_to_pixel(source_pixel, &intr, ray);
                rectify_map_x.at<float>(y,x) = source_pixel[0];
                rectify_map_y.at<float>(y,x) = source_pixel[1];
              }
            }
          }
          auto info = std::make_shared<CameraInfo>();
          info->header.frame_id = rgbd_optical_frame_;
          info->width = static_cast<uint32_t>(color.get_width());
          info->height = static_cast<uint32_t>(color.get_height());
          info->distortion_model = "plumb_bob";
          info->k = {intr.fx,0.0,intr.ppx, 0.0,intr.fy,intr.ppy, 0.0,0.0,1.0};
          info->d.assign(5, 0.0); // output has been rectified to a pinhole model
          cv::Mat rgb_view(color.get_height(), color.get_width(), CV_8UC3,
                           const_cast<void*>(color.get_data()), color.get_stride_in_bytes());
          cv::Mat depth_raw(depth.get_height(), depth.get_width(), CV_16UC1,
                            const_cast<void*>(depth.get_data()), depth.get_stride_in_bytes());
          cv::Mat depth_metres;
          depth_raw.convertTo(depth_metres, CV_32FC1, depth.get_units());
          cv::Mat rectified_rgb, rectified_depth;
          if (!rectify_map_x.empty()) {
            cv::remap(rgb_view, rectified_rgb, rectify_map_x, rectify_map_y,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            cv::remap(depth_metres, rectified_depth, rectify_map_x, rectify_map_y,
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT);
          } else {
            rectified_rgb = rgb_view;
            rectified_depth = depth_metres;
          }
          // Host ROS time approximates acquisition time: scan only when wrist is stationary.
          std_msgs::msg::Header header;
          header.stamp = now();
          header.frame_id = rgbd_optical_frame_;
          info->header.stamp = header.stamp;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            camera_info_ = info;
            rgb_frames_.push_back(cv_bridge::CvImage(header,"bgr8",rectified_rgb).toImageMsg());
            depth_frames_.push_back(cv_bridge::CvImage(header,
                sensor_msgs::image_encodings::TYPE_32FC1,rectified_depth).toImageMsg());
            while (rgb_frames_.size() > 4) rgb_frames_.pop_front();
            while (depth_frames_.size() > 4) depth_frames_.pop_front();
          }
          cv_.notify_all();
          if ((++preview_counter % std::max(1, rgbd_fps_)) == 0)
            rgbd_preview_pub_->publish(*cv_bridge::CvImage(header, "bgr8", rectified_rgb).toImageMsg());
        }
        pipeline.stop();
      } catch (const std::exception &e) {
        if (!stopping_.load())
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                               "RealSense: %s (retrying)", e.what());
        { std::lock_guard<std::mutex> lock(mutex_);
          camera_info_.reset(); rgb_frames_.clear(); depth_frames_.clear(); }
        for (int n=0; n<10 && !stopping_.load() && rclcpp::ok(); ++n)
          std::this_thread::sleep_for(100ms);
      }
    }
#endif
  }
  struct Snapshot {
    Image::ConstSharedPtr rgb, depth;
    CameraInfo::ConstSharedPtr info;
  };
  void append(std::deque<Image::ConstSharedPtr> & queue, Image::ConstSharedPtr image) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue.push_back(std::move(image));
    if (queue.size()>15) queue.pop_front();
    cv_.notify_all();
  }
  bool interrupted(const std::shared_ptr<GoalHandle>& h) const {
    return stopping_.load() || !rclcpp::ok() || h->is_canceling();
  }
  void phase(const std::shared_ptr<GoalHandle>& h, const std::string& phase_text) {
    if (!h->is_active()) return;
    auto f=std::make_shared<Inspect::Feedback>();
    f->phase=phase_text;
    h->publish_feedback(f);
  }
  void terminal(const std::shared_ptr<GoalHandle>& h, bool ok, const std::string& message,
                const Array& findings=Array()) {
    auto result=std::make_shared<Inspect::Result>();
    result->success=ok;
    result->message=message;
    if (ok) result->findings=findings;
    busy_.store(false);
    if (h->is_canceling()) h->canceled(result);
    else if (ok) h->succeed(result);
    else h->abort(result);
    RCLCPP_INFO(get_logger(), "Inspection %s: %s", ok?"complete":"failed",message.c_str());
  }
  bool getFreshPair(const Clock::time_point start_time,
                    const std::shared_ptr<GoalHandle>& h, Snapshot& s) {
    auto deadline=Clock::now()+std::chrono::duration<double>(acquisition_timeout_sec_);
    std::unique_lock<std::mutex> lock(mutex_);
    while (Clock::now()<deadline && !interrupted(h)) {
      if (camera_info_) {
        // Each received frame has to arrive after goal acceptance; no cached stale data.
        // The queue is cleared on goal start by run() before calling this routine.
        for (auto r=rgb_frames_.rbegin(); r!=rgb_frames_.rend(); ++r) {
          if (!validStamp((*r)->header.stamp)) continue;
          for (auto d=depth_frames_.rbegin(); d!=depth_frames_.rend(); ++d) {
            if (!validStamp((*d)->header.stamp)) continue;
            const double dt = std::abs(stampNs((*r)->header.stamp)-stampNs((*d)->header.stamp))*1e-9;
            if (dt>sync_tolerance_sec_) continue;
            if ((*r)->header.frame_id != (*d)->header.frame_id ||
                camera_info_->header.frame_id != (*r)->header.frame_id ||
                camera_info_->width != (*r)->width || camera_info_->height != (*r)->height ||
                (*d)->width != (*r)->width || (*d)->height != (*r)->height) continue;
            if (camera_info_->k[0] <= 0 || camera_info_->k[4] <= 0 ||
                ((*r)->header.stamp.sec==0)) continue;
            s={*r,*d,camera_info_};
            return true;
          }
          // Only use a model-plane ray if the depth stream is unavailable.
          if (cad_plane_fallback_ && input_mode_ != "image" &&
              camera_info_->header.frame_id==(*r)->header.frame_id &&
              camera_info_->width==(*r)->width && camera_info_->height==(*r)->height &&
              camera_info_->k[0]>0 && camera_info_->k[4]>0) {
            s={*r,nullptr,camera_info_};
            return true;
          }
        }
      }
      cv_.wait_for(lock,100ms);
    }
    (void)start_time;
    return false;
  }
  bool modelFresh(const builtin_interfaces::msg::Time& image_stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!model_visible_ || !validStamp(model_pose_stamp_)) return false;
    if (std::chrono::duration<double>(Clock::now()-model_pose_rx_).count()>
        model_pose_max_age_sec_) return false;
    // Prevent a depth scan from using a model observation many seconds earlier.
    return std::abs(stampNs(model_pose_stamp_)-stampNs(image_stamp))*1e-9 <=
           model_pose_max_age_sec_;
  }
  static cv::Point2f project(const tf2::Transform& optical_T_tooth,
                             double x, double y, const cv::Mat& K, const cv::Mat& dist) {
    const tf2::Vector3 p=optical_T_tooth*tf2::Vector3(x,y,0);
    if (p.z()<=0) throw std::runtime_error("Tooth surface behind camera");
    std::vector<cv::Point3f> xyz{{static_cast<float>(p.x()),
                                  static_cast<float>(p.y()),static_cast<float>(p.z())}};
    std::vector<cv::Point2f> uv;
    cv::projectPoints(xyz,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,dist,uv);
    return uv.front();
  }
  Array analyze(const Snapshot& s, cv::Mat& annotated, const std::string& region_id) {
    if (s.info->distortion_model != "plumb_bob" &&
        s.info->distortion_model != "rational_polynomial" &&
        !(s.info->distortion_model.empty() && s.info->d.empty())) {
      throw std::runtime_error("Unsupported CameraInfo distortion model");
    }
    cv::Mat rgb=cv_bridge::toCvCopy(*s.rgb,"bgr8")->image;
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
    cv::Mat blurred;
    cv::GaussianBlur(gray,blurred,cv::Size(3,3),0);
    cv::Mat depth;
    if (s.depth) {
      depth=cv_bridge::toCvCopy(*s.depth,s.depth->encoding)->image;
      if (s.depth->encoding!=sensor_msgs::image_encodings::TYPE_16UC1 &&
          s.depth->encoding!=sensor_msgs::image_encodings::TYPE_32FC1)
        throw std::runtime_error("Depth must be 16UC1 millimetres or 32FC1 metres");
    } else if (!cad_plane_fallback_ || input_mode_ == "image") {
      throw std::runtime_error("Missing registered depth; CAD fallback not commissioned");
    }
    const cv::Mat K=cameraMatrix(*s.info), dist(s.info->d,true);
    annotated=rgb.clone();
    Array result;
    result.header=s.rgb->header;
    uint32_t id=1001;
    for (int tooth=1; tooth<=4; ++tooth) {
      const std::string tooth_name="tooth_"+std::to_string(tooth);
      if (!region_id.empty() && region_id!=tooth_name) continue;
      const auto tf_msg = tf_buffer_->lookupTransform(
          s.rgb->header.frame_id,tooth_name,rclcpp::Time(s.rgb->header.stamp),
          rclcpp::Duration::from_seconds(0.5));
      tf2::Transform optical_T_tooth;
      tf2::fromMsg(tf_msg.transform,optical_T_tooth);
      const tf2::Vector3 normal = optical_T_tooth.getBasis()*tf2::Vector3(0,0,1);
      // Tooth top must face the camera; otherwise ROI/depth is likely occluded or inverted.
      if (normal.z()>-0.2) throw std::runtime_error(tooth_name+" does not face RGB-D camera");
      std::vector<cv::Point> polygon;
      for (const auto& xy : std::array<std::array<double,2>,4>{{
          {{-tooth_half_width_m_,-tooth_half_length_m_}},
          {{tooth_half_width_m_,-tooth_half_length_m_}},
          {{tooth_half_width_m_,tooth_half_length_m_}},
          {{-tooth_half_width_m_,tooth_half_length_m_}}}}) {
        const auto uv=project(optical_T_tooth,xy[0],xy[1],K,dist);
        polygon.emplace_back(static_cast<int>(std::lround(uv.x)),
                             static_cast<int>(std::lround(uv.y)));
      }
      cv::Mat mask=cv::Mat::zeros(gray.size(),CV_8UC1);
      cv::fillConvexPoly(mask,polygon,cv::Scalar(255));
      // Remove the region boundary to avoid dark background and dental print edges.
      cv::erode(mask,mask,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)));
      const int mask_pixels=cv::countNonZero(mask);
      if (mask_pixels<25) throw std::runtime_error(tooth_name+" projected ROI is too small");
      const double expected=optical_T_tooth.getOrigin().z();
      int valid_surface_pixels=0;
      if (!depth.empty()) for (int y=0; y<mask.rows; ++y) {
        for (int x=0; x<mask.cols; ++x) {
          if (mask.at<uint8_t>(y,x)==0) continue;
          const double depth_z=depthMetres(depth,x,y);
          if (std::isfinite(depth_z) && depth_z>=min_depth_m_ && depth_z<=max_depth_m_ &&
              std::abs(depth_z-expected)<=max_surface_error_m_) ++valid_surface_pixels;
        }
      }
      if (!depth.empty() &&
          static_cast<double>(valid_surface_pixels)/mask_pixels<min_surface_valid_fraction_ &&
          !cad_plane_fallback_)
        throw std::runtime_error(tooth_name+" has insufficient measured tooth-surface depth");
      cv::Mat binary, absolute_dark, adaptive_dark;
      cv::threshold(blurred,absolute_dark,dark_threshold_,255,cv::THRESH_BINARY_INV);
      cv::adaptiveThreshold(blurred,adaptive_dark,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                            cv::THRESH_BINARY_INV,21,12);
      cv::bitwise_and(absolute_dark,adaptive_dark,binary);
      cv::bitwise_and(binary,mask,binary);
      cv::morphologyEx(binary,binary,cv::MORPH_OPEN,
          cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));
      cv::Mat labels,stats,centroids;
      const int n=cv::connectedComponentsWithStats(binary,labels,stats,centroids,8,CV_32S);
      int best=-1;
      for (int component=1; component<n; ++component) {
        if (stats.at<int>(component,cv::CC_STAT_AREA)>=minimum_blob_pixels_ &&
            (best<0 || stats.at<int>(component,cv::CC_STAT_AREA) >
                       stats.at<int>(best,cv::CC_STAT_AREA))) best=component;
      }
      cv::polylines(annotated,polygon,true,cv::Scalar(0,210,0),2);
      if (best<0) continue;
      const double u=centroids.at<double>(best,0), v=centroids.at<double>(best,1);
      std::vector<double> z_values;
      const int cx=static_cast<int>(std::lround(u)), cy=static_cast<int>(std::lround(v));
      if (!depth.empty()) {
        for (int y=std::max(0,cy-6); y<=std::min(depth.rows-1,cy+6); ++y)
          for (int x=std::max(0,cx-6); x<=std::min(depth.cols-1,cx+6); ++x) {
            if (labels.at<int>(y,x)!=best) continue;
            const double z=depthMetres(depth,x,y);
            if (std::isfinite(z) && z>=min_depth_m_ && z<=max_depth_m_)
              z_values.push_back(z);
          }
      }
      // A valid depth patch is preferred. Otherwise project the RGB pixel
      // onto the known CAD tooth-top tangent plane (hover only, opt-in).
      bool use_plane=(z_values.size()<3);
      if (use_plane && !cad_plane_fallback_)
        throw std::runtime_error(tooth_name+" stain has insufficient valid depth");
      double z=0.0;
      if (!use_plane) {
        auto median_it=z_values.begin()+z_values.size()/2;
        std::nth_element(z_values.begin(),median_it,z_values.end());
        z=*median_it;
        if (std::abs(expected-z)>max_surface_error_m_) {
          if (!cad_plane_fallback_)
            throw std::runtime_error(tooth_name+" detected dark patch is off expected tooth surface");
          use_plane=true;
        }
      }
      // Use actual image calibration, including distortion, for camera-ray backprojection.
      std::vector<cv::Point2f> undistorted;
      cv::undistortPoints(std::vector<cv::Point2f>{{static_cast<float>(u),static_cast<float>(v)}},
                          undistorted,K,dist);
      const cv::Point2f ray=undistorted.front();
      if (use_plane) {
        const tf2::Vector3 ray_direction(ray.x,ray.y,1.0);
        const double denominator=normal.dot(ray_direction);
        if (std::abs(denominator)<0.20)
          throw std::runtime_error("CAD tooth plane almost parallel to camera ray");
        z=normal.dot(optical_T_tooth.getOrigin())/denominator;
        if (!std::isfinite(z) || z<min_depth_m_ || z>max_depth_m_)
          throw std::runtime_error("Projected RGB target outside calibrated camera range");
        RCLCPP_WARN(get_logger(),"%s RGB stain projected onto CAD plane (NOT measured depth)",
                    tooth_name.c_str());
      }
      Finding f;
      f.id=id++;
      f.issue_type=Finding::ISSUE_STAIN; // simulated black marking; no claim of plaque chemistry
      f.tooth_id=tooth_name;
      f.position.x=ray.x*z;
      f.position.y=ray.y*z;
      f.position.z=z;
      f.surface_normal.x=normal.x();
      f.surface_normal.y=normal.y();
      f.surface_normal.z=normal.z();
      const int area=stats.at<int>(best,cv::CC_STAT_AREA);
      // Heuristic score, not statistically calibrated. Minimum >= target manager's 0.75.
      f.confidence=static_cast<float>(std::min(0.98,0.80+0.01*std::sqrt(area)));
      result.findings.push_back(f);
      cv::circle(annotated,cv::Point(cx,cy),5,cv::Scalar(0,0,255),2);
      cv::putText(annotated,tooth_name,cv::Point(cx+6,cy-6),
                  cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255),2);
    }
    return result;
  }
  void run(const std::shared_ptr<GoalHandle> h) {
    try {
      if (input_mode_ == "image" && rgbd_depth_images_.empty()) {
        terminal(h,false,"RGB-only sample images: 2D preview available, but no depth for 3D findings");
        return;
      }
      if (input_mode_ == "image" && (!allow_image_findings_ || !image_camera_calibrated_)) {
        terminal(h,false,"Offline image 3D findings disabled: verify calibration and enable only in simulation");
        return;
      }
      const auto start=Clock::now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // Do not use any image collected before this goal.
        rgb_frames_.clear(); depth_frames_.clear();
      }
      phase(h,"WAITING_FOR_FRESH_RGB_DEPTH_OR_CALIBRATED_CAD_PLANE");
      Snapshot snapshot;
      if (!getFreshPair(start,h,snapshot)) {
        terminal(h,false,"No new synchronized, registered RGB/depth/CameraInfo pair"); return;
      }
      if (interrupted(h)) {terminal(h,false,"Cancelled");return;}
      if (!modelFresh(snapshot.rgb->header.stamp)) {
        terminal(h,false,"Four-marker jaw localization unavailable or stale"); return;
      }
      phase(h,"DETECTING_DARK_MARKINGS");
      cv::Mat debug;
      Array findings=analyze(snapshot,debug,h->get_goal()->region_id);
      if (interrupted(h)) {terminal(h,false,"Cancelled");return;}
      // This exact result is published and returned to the supervisor.
      findings_pub_->publish(findings);
      const cv_bridge::CvImage debug_img(snapshot.rgb->header,"bgr8",debug);
      debug_pub_->publish(*debug_img.toImageMsg());
      terminal(h,true,snapshot.depth ? "Acquired fresh RGB-D and inspected tooth regions" :
          "RGB-only inspection projected onto calibrated CAD tooth planes (no measured depth)",findings);
    } catch (const std::exception & e) {
      terminal(h,false,std::string("Inspection exception: ")+e.what());
    }
  }
  bool enable_inspection_action_{true};
  std::string input_mode_, depth_camera_serial_, rgbd_optical_frame_;
  std::vector<std::string> rgbd_images_,rgbd_depth_images_;
  std::vector<double> image_camera_matrix_,image_distortion_;
  double rgbd_image_interval_sec_{};
  bool image_camera_calibrated_{false},allow_image_findings_{false};
  bool cad_plane_fallback_{false};
  int preview_dark_threshold_{95},preview_min_blob_area_{30};
  int color_width_{},color_height_{},depth_width_{},depth_height_{},rgbd_fps_{};
  std::thread camera_thread_;
  std::string rgb_topic_,depth_topic_,info_topic_,model_frame_;
  double sync_tolerance_sec_{},acquisition_timeout_sec_{},model_pose_max_age_sec_{};
  double min_depth_m_{},max_depth_m_{},max_surface_error_m_{};
  double tooth_half_width_m_{},tooth_half_length_m_{};
  int dark_threshold_{},minimum_blob_pixels_{};
  double min_surface_valid_fraction_{};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Image::ConstSharedPtr> rgb_frames_,depth_frames_;
  CameraInfo::ConstSharedPtr camera_info_;
  builtin_interfaces::msg::Time model_pose_stamp_{};
  Clock::time_point model_pose_rx_{};
  bool model_visible_{false};
  std::atomic<bool> busy_{false},stopping_{false};
  std::thread worker_;
  rclcpp::Subscription<Image>::SharedPtr rgb_sub_,depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr model_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr model_valid_sub_;
  rclcpp::Publisher<Array>::SharedPtr findings_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<Image>::SharedPtr rgbd_preview_pub_;
  rclcpp_action::Server<Inspect>::SharedPtr inspect_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<PerceptionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
