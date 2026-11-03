// MTRN4231: Move wrist RGB-D camera to a model-relative inspection pose,
// and return to the exact joint configuration present at the start of a cycle.
// All positions are computed from live TF and MoveIt robot FK. No fixed
// robot-space inspection or home coordinates are stored in source code.
// Only non-contact, collision-checked trajectories are permitted.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "dental_interfaces/action/move_to_station.hpp"

namespace {
using Action = dental_interfaces::action::MoveToStation;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Transform & t) {
  const Eigen::Quaterniond q(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm()-1.0)>0.02)
    throw std::runtime_error("Invalid quaternion from TF");
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear()=q.normalized().toRotationMatrix();
  out.translation()=Eigen::Vector3d(t.translation.x,t.translation.y,t.translation.z);
  if (!out.matrix().allFinite()) throw std::runtime_error("Non-finite TF");
  return out;
}
}

class StationMotion final : public rclcpp::Node {
public:
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  template<class T> T param(const std::string & name, const T & default_value) {
    T result{};
    return get_parameter(name,result) ? result : declare_parameter<T>(name,default_value);
  }
  StationMotion():Node("station_motion",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    robot_mode_=param<std::string>("robot_mode","fake");
    base_=param<std::string>("robot_base_frame","base_link");
    tool_=param<std::string>("pose_link","tool0");
    camera_=param<std::string>("rgbd_optical_frame","rgbd_color_optical_frame");
    model_=param<std::string>("model_frame","dental_model");
    group_name_=param<std::string>("planning_group","ur_manipulator");
    inspection_height_=param<double>("inspection_height_m",0.30);
    // A generic fake UR5e often initializes far from the virtual jaw; allow a
    // longer simulated first transfer without weakening the real robot limit.
    // This is a straight-line displacement guard, NOT a collision-free path.
    max_first_move_=param<double>("max_station_move_m", robot_mode_=="fake" ? 1.80 : 0.45);
    max_tf_age_=param<double>("max_model_tf_age_sec",2.0);
    // The aggregate L1 distance sums the absolute movement of ALL six joints.
    // The nominal fake UR starts fully extended and may need a large, valid
    // multi-joint transfer to inspect. Keep the real-hardware budget unchanged.
    max_joint_travel_=param<double>("max_joint_path_l1_rad", robot_mode_=="fake" ? 16.0 : 6.0);
    velocity_=param<double>("velocity_scaling",0.05);
    execute_fake_=param<bool>("execute_fake_motion",false);
    confirm_fake_=param<bool>("confirm_fake_hardware",false);
    execute_real_=param<bool>("execute_real_motion",false);
    // Synthetic mock-only look-ahead. This is NOT a real perception target
    // or an approval for any physical motion.
    fake_probe_enabled_=param<bool>("fake_treatment_probe_enabled",false);
    fake_probe_frame_=param<std::string>("fake_treatment_probe_tooth_frame","tooth_1");
    fake_probe_pre_m_=param<double>("fake_treatment_probe_preapproach_m",0.15);
    fake_probe_move_m_=param<double>("fake_treatment_probe_first_move_m",0.35);
    fake_probe_brush_offset_=param<std::vector<double>>(
        "fake_treatment_probe_brush_offset_tool_m",std::vector<double>{0.0,0.0,0.0});
    // A physical UR has multiple IK branches for one TCP pose. The approved
    // home is a fixed *measured* joint vector, not a free pose-goal IK guess.
    // CSV is used because nested ROS 2 launch substitution lists are brittle.
    const auto home_csv=param<std::string>("real_home_joint_positions_csv", "");
    if(robot_mode_=="real" && !home_csv.empty()) {
      std::istringstream stream(home_csv);
      std::string piece;
      while(std::getline(stream,piece,',')) {
        try {
          size_t used=0;
          const double v=std::stod(piece,&used);
          if(used!=piece.size() || !std::isfinite(v) || std::abs(v)>2.0*M_PI)
            throw std::invalid_argument("Invalid joint value");
          commissioned_home_joints_.push_back(v);
        } catch(const std::exception &) {
          throw std::invalid_argument("Invalid real home joint CSV");
        }
      }
      if(commissioned_home_joints_.size()!=6)
        throw std::invalid_argument("Commissioned home must contain exactly six UR5e joints");
    }
    calibrated_=param<bool>("confirm_real_calibration",false);
    workcell_confirmed_=param<bool>("confirm_real_workcell",false);
    if ((robot_mode_!="fake" && robot_mode_!="real") || base_.empty() || tool_.empty() ||
        camera_.empty() || model_.empty() || group_name_.empty() ||
        !std::isfinite(inspection_height_) || inspection_height_<0.20 || inspection_height_>0.60 ||
        !std::isfinite(max_first_move_) || max_first_move_<0.05 ||
        max_first_move_>(robot_mode_=="fake" ? 2.00 : 0.60) ||
        !std::isfinite(max_tf_age_) || max_tf_age_<=0 ||
        !std::isfinite(max_joint_travel_) || max_joint_travel_<=0 ||
        max_joint_travel_>(robot_mode_=="fake" ? 18.0 : 10.0) ||
        !std::isfinite(velocity_) || velocity_<0.01 || velocity_>0.15)
      throw std::invalid_argument("Invalid station motion limits/configuration");
    if(fake_probe_enabled_ && (robot_mode_!="fake" || fake_probe_frame_.empty() ||
        !std::isfinite(fake_probe_pre_m_) || fake_probe_pre_m_<0.03 || fake_probe_pre_m_>0.20 ||
        !std::isfinite(fake_probe_move_m_) || fake_probe_move_m_<0.05 || fake_probe_move_m_>0.50 ||
        fake_probe_brush_offset_.size()!=3 ||
        !std::all_of(fake_probe_brush_offset_.begin(),fake_probe_brush_offset_.end(),
                     [](double v){return std::isfinite(v) && std::abs(v)<0.30;})))
      throw std::invalid_argument("Invalid fake-only treatment preview configuration");
    if (execute_fake_ && (robot_mode_!="fake" || !confirm_fake_))
      throw std::invalid_argument("Fake execution requires explicit fake-hardware confirmation");
    if (execute_real_ && (robot_mode_!="real" || !calibrated_ || !workcell_confirmed_))
      throw std::invalid_argument("Real movement requires calibrated cameras, tool and measured MoveIt scene");
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Separate home-ready latch: a direct TreatTarget action cannot bypass
    // the supervisor's home-first sequence on the physical UR5e.
    home_ready_pub_=create_publisher<std_msgs::msg::Bool>(
        "/dental/motion/real_home_ready", rclcpp::QoS(1).reliable().transient_local());
    publishHomeReady(false);
    scene_sub_=create_subscription<std_msgs::msg::Bool>("/dental/workcell/ready",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg){scene_ready_.store(msg->data);});
    server_=rclcpp_action::create_server<Action>(this,"/dental/motion/move_to_station",
        [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal> goal){
          if (goal->station!="inspect" && goal->station!="home") return rclcpp_action::GoalResponse::REJECT;
          if (robot_mode_=="fake" && execute_fake_ && !scene_ready_.load()) {
            RCLCPP_ERROR(get_logger(),"Fake motion locked: collision workcell not verified");
            return rclcpp_action::GoalResponse::REJECT;
          }
          if (robot_mode_=="real" && (!scene_ready_.load() ||
              !calibrated_ || !workcell_confirmed_)) {
            RCLCPP_ERROR(get_logger(),"Real planning locked: measured TF/workcell not confirmed");
            return rclcpp_action::GoalResponse::REJECT;
          }
          bool expected=false;
          return busy_.compare_exchange_strong(expected,true) ?
              rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
        },
        [this](std::shared_ptr<Handle>){cancel_.store(true);stop();return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<Handle> h){if(worker_.joinable())worker_.join();
          cancel_.store(false);worker_=std::thread([this,h]{run(h);});});
    RCLCPP_INFO(get_logger(), "DENTAL_MVP_BUILD_ID=station_revisit_v4_20261103");
    RCLCPP_INFO(get_logger(), "DENTAL_REAL_HOME_V5_STATION");
    RCLCPP_INFO(get_logger(), "DENTAL_FAKE_APPROACH_LOOKAHEAD_V7 probe_enabled=%s",
        fake_probe_enabled_ ? "true" : "false");
    RCLCPP_WARN(get_logger(),"Station server ready: %s mode; execution=%s, no-contact",
        robot_mode_.c_str(),(robot_mode_=="fake" ? execute_fake_ : execute_real_) ? "ENABLED" : "DISABLED");
  }
  ~StationMotion() override {cancel_.store(true);stop();if(worker_.joinable())worker_.join();}
private:
  void stop() {
    std::lock_guard<std::mutex> lk(group_mutex_);
    if(active_group_)active_group_->stop();
  }
  void publishHomeReady(bool ready) {
    if(robot_mode_!="real") return;
    std_msgs::msg::Bool msg; msg.data=ready;
    home_ready_pub_->publish(msg);
  }
  void feedback(std::shared_ptr<Handle> h,const std::string & what){
    auto f=std::make_shared<Action::Feedback>();f->phase=what;h->publish_feedback(f);
    RCLCPP_INFO(get_logger(),"[STATION] %s",what.c_str());
  }
  void done(std::shared_ptr<Handle> h,bool ok,const std::string & message){
    auto result=std::make_shared<Action::Result>();result->success=ok;result->message=message;
    busy_.store(false);
    if(h->is_canceling())h->canceled(result);
    else if(ok)h->succeed(result);
    else h->abort(result);
    RCLCPP_INFO(get_logger(),"[STATION] %s: %s",ok?"COMPLETE":"FAIL",message.c_str());
  }
  void run(std::shared_ptr<Handle> h) {
    try {perform(h);}catch(const std::exception & e){
      if(robot_mode_=="real") {home_reached_=false;publishHomeReady(false);}
      done(h,false,e.what());
    }
  }
  void perform(std::shared_ptr<Handle> h) {
    const auto station=h->get_goal()->station;
    feedback(h,"VALIDATING_"+station);
    if(robot_mode_=="real") {
      if(commissioned_home_joints_.size()!=6)
        throw std::runtime_error("No commissioned real home joints; run read-only capture and approval first");
      if(station=="inspect" && !home_reached_)
        throw std::runtime_error("Real inspect locked until verified home-first station action completes");
      if(station=="home") {home_reached_=false;publishHomeReady(false);}
    }
    auto group=std::make_shared<MoveGroup>(shared_from_this(),group_name_);
    {std::lock_guard<std::mutex> lk(group_mutex_);active_group_=group;}
    if(!group->setEndEffectorLink(tool_))throw std::runtime_error("MoveIt tool0 pose link unavailable");
    group->setPoseReferenceFrame(base_);
    group->setPlanningTime(12.0);
    group->setNumPlanningAttempts(5);
    group->setMaxVelocityScalingFactor(velocity_);
    group->setMaxAccelerationScalingFactor(velocity_);
    group->setGoalPositionTolerance(0.006);
    group->setGoalOrientationTolerance(0.07);
    group->allowReplanning(false);
    auto state=group->getCurrentState(8.0);
    if(!state)throw std::runtime_error("Missing fresh robot joint states");
    const auto model=state->getRobotModel();
    const auto * jmg=model->getJointModelGroup(group_name_);
    if(!jmg || !model->hasLinkModel(base_) || !model->hasLinkModel(tool_))
      throw std::runtime_error("MoveIt model missing configured base/tool/planning group");
    state->updateLinkTransforms();
    const Eigen::Isometry3d base_T_tool=state->getGlobalLinkTransform(base_).inverse()*
                                      state->getGlobalLinkTransform(tool_);
    std::vector<double> current_joints;
    state->copyJointGroupPositions(jmg,current_joints);
    std::vector<geometry_msgs::msg::PoseStamped> fake_inspection_candidates;
    // Within one simulated MVP cycle, reinspection must reproduce the exact
    // joint configuration in which the FIRST inspection succeeded. Solving a
    // fresh pose goal for pass 2 can switch UR5e IK branches and move the arm
    // through an unnecessary wrist revolution, making the same treatment
    // pose unreachable. This cache is fake-only: real operation must check
    // the measured jaw TF and plan against its current location every time.
    const bool reuse_fake_inspection =
        station == "inspect" && robot_mode_ == "fake" && inspection_locked_;
    Eigen::Isometry3d requested_model_tf = Eigen::Isometry3d::Identity();
    if(station=="inspect" && reuse_fake_inspection) {
      if(inspection_joints_.size() != current_joints.size() ||
          inspection_joints_.empty())
        throw std::runtime_error("Saved fake inspection joint state is missing or malformed");
      // Even for fake hardware, fail closed if the jaw fixture has moved.
      // Do not blindly return to a pose associated with an old model frame.
      const auto model_tf = tf_buffer_->lookupTransform(base_, model_, rclcpp::Time(0),
          rclcpp::Duration::from_seconds(1.0));
      const Eigen::Isometry3d current_model_tf = toEigen(model_tf.transform);
      const double model_translation_change =
          (current_model_tf.translation() - inspection_model_tf_.translation()).norm();
      const double model_rotation_change = Eigen::AngleAxisd(
          inspection_model_tf_.linear().transpose() * current_model_tf.linear()).angle();
      if(!std::isfinite(model_translation_change) ||
          !std::isfinite(model_rotation_change) ||
          model_translation_change > 0.002 || model_rotation_change > 0.0175)
        throw std::runtime_error(
            "Fake jaw TF changed since initial inspection; cannot reuse cached inspection joints");
      RCLCPP_INFO(get_logger(),
          "[STATION] REVISIT_inspect: planning to saved first-pass inspection joints "
          "(jaw TF drift %.4f m, %.4f rad); skipping random pose/IK search",
          model_translation_change, model_rotation_change);
      if(!group->setJointValueTarget(inspection_joints_))
        throw std::runtime_error("Saved fake inspection joint target is invalid");
    } else if(station=="inspect") {
      // Record the true first joint configuration as "home". Never use an
      // arbitrary hard-coded joint configuration on an actual workcell.
      // Keep the original home after the first successful inspection transfer,
      // but refresh it on a new attempt if the previous attempt never moved.
      // Otherwise a user moving the FAKE robot in RViz after a failed plan
      // would accidentally retain a stale "home" from the old attempt.
      if(!home_locked_)home_joints_=current_joints;
      const auto model_tf=tf_buffer_->lookupTransform(base_,model_,rclcpp::Time(0),
          rclcpp::Duration::from_seconds(1.0));
      const auto camera_tf=tf_buffer_->lookupTransform(tool_,camera_,rclcpp::Time(0),
          rclcpp::Duration::from_seconds(1.0));
      if(robot_mode_=="real") {
        const double age=(now()-rclcpp::Time(model_tf.header.stamp,get_clock()->get_clock_type())).seconds();
        if(!std::isfinite(age)||age< -0.2||age>max_tf_age_)
          throw std::runtime_error("Jaw/overhead TF is stale; do not move with stale model position");
      }
      const Eigen::Isometry3d base_T_model=toEigen(model_tf.transform);
      requested_model_tf = base_T_model;
      const Eigen::Isometry3d tool_T_camera=toEigen(camera_tf.transform);
      // Camera looks along its optical +Z. Desired optical +Z points toward
      // the model, opposite model +Z. Preserve wrist yaw when already facing
      // down to avoid unnecessary full-wrist rotations.
      const Eigen::Isometry3d base_T_camera_now=base_T_tool*tool_T_camera;
      const Eigen::Vector3d surface_normal=base_T_model.linear().col(2);
      const double facing=base_T_camera_now.linear().col(2).dot(surface_normal);
      Eigen::Matrix3d chosen=base_T_camera_now.linear();
      if(facing>-0.90) {
        chosen=base_T_model.linear()*Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()).toRotationMatrix();
      }
      Eigen::Isometry3d desired_camera=Eigen::Isometry3d::Identity();
      desired_camera.linear()=chosen;
      // Model origin is at jaw base; 31 mm is nominal STL tooth-top height.
      desired_camera.translation()=base_T_model.translation()+surface_normal*inspection_height_;
      const Eigen::Isometry3d desired_tool=desired_camera*tool_T_camera.inverse();
      const Eigen::Vector3d start_xyz = base_T_tool.translation();
      const Eigen::Vector3d target_xyz = desired_tool.translation();
      const double first_move_distance = (target_xyz-start_xyz).norm();
      RCLCPP_INFO(get_logger(),
          "Inspection FK tool0 start=[%.3f, %.3f, %.3f] m; target=[%.3f, %.3f, %.3f] m; "
          "displacement=%.3f m; configured limit=%.3f m (%s)",
          start_xyz.x(),start_xyz.y(),start_xyz.z(),
          target_xyz.x(),target_xyz.y(),target_xyz.z(),
          first_move_distance,max_first_move_,robot_mode_.c_str());
      if(!std::isfinite(first_move_distance) || first_move_distance>max_first_move_)
        throw std::runtime_error(
          "Inspection pose exceeds first-move travel bound: displacement="+
          std::to_string(first_move_distance)+"m, limit="+
          std::to_string(max_first_move_)+"m (no motion commanded)");
      if(robot_mode_=="fake" && first_move_distance>0.45)
        RCLCPP_WARN(get_logger(),
          "FAKE HARDWARE ONLY: long initial positioning transfer; "
          "MoveIt collision/path checks remain mandatory before execution");
      const Eigen::Quaterniond q(desired_tool.linear());
      geometry_msgs::msg::PoseStamped pose;pose.header.frame_id=base_;pose.header.stamp=now();
      pose.pose.position.x=desired_tool.translation().x();
      pose.pose.position.y=desired_tool.translation().y();
      pose.pose.position.z=desired_tool.translation().z();
      pose.pose.orientation.x=q.x();pose.pose.orientation.y=q.y();
      pose.pose.orientation.z=q.z();pose.pose.orientation.w=q.w();
      if(robot_mode_=="fake") {
        // Only the view direction matters for this synthetic RGB inspection.
        // Rotate the optical XY axes around optical +Z, keeping its position
        // and downward-looking +Z unchanged. Each variant is independently
        // planned and subjected to the SAME joint, collision and scene checks.
        // This avoids accepting unnecessary wrist winding from one arbitrary
        // MoveIt IK branch. Never perform this search for the real robot.
        const std::vector<double> optical_yaws{0.0, M_PI, M_PI/2.0, -M_PI/2.0};
        for (const double yaw : optical_yaws) {
          Eigen::Isometry3d variant_camera=desired_camera;
          variant_camera.linear()=desired_camera.linear() *
              Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
          const Eigen::Isometry3d variant_tool=variant_camera*tool_T_camera.inverse();
          const double distance=(variant_tool.translation()-start_xyz).norm();
          if(!std::isfinite(distance) || distance>max_first_move_) {
            RCLCPP_WARN(get_logger(),
              "[STATION] Skipping optical yaw %.0f deg: tool0 transfer %.3f m exceeds %.3f m",
              yaw*180.0/M_PI,distance,max_first_move_);
            continue;
          }
          const Eigen::Quaterniond variant_q(variant_tool.linear());
          geometry_msgs::msg::PoseStamped candidate=pose;
          candidate.pose.position.x=variant_tool.translation().x();
          candidate.pose.position.y=variant_tool.translation().y();
          candidate.pose.position.z=variant_tool.translation().z();
          candidate.pose.orientation.x=variant_q.x();
          candidate.pose.orientation.y=variant_q.y();
          candidate.pose.orientation.z=variant_q.z();
          candidate.pose.orientation.w=variant_q.w();
          fake_inspection_candidates.push_back(candidate);
        }
        if(fake_inspection_candidates.empty())
          throw std::runtime_error("No fake inspection yaw variant within first-move limit");
      } else {
        group->setPoseTarget(pose,tool_);
      }
    } else {
      // Real HOME is the commissioned, fixed UR5e joint branch. Fake HOME
      // keeps the original cycle-start behaviour (verified MVP regression).
      const auto & target_joints = robot_mode_=="real" ? commissioned_home_joints_ : home_joints_;
      if(target_joints.size()!=current_joints.size())
        throw std::runtime_error("No valid six-joint home configuration");
      if(!group->setJointValueTarget(target_joints))
        throw std::runtime_error("Commissioned/saved home target rejected by MoveIt joint limits");
    }
    feedback(h,"PLANNING_"+station);
    // Inspect the complete trajectory before execution, regardless of whether
    // the target came from a regular station request or a fake-only yaw choice.
    // Return a diagnostic string so another equivalent fake target may be
    // tried; malformed or unsafe plans are NEVER executed.
    auto validate_trajectory=[this,&station](const MoveGroup::Plan & candidate)->std::string {
      const auto & jt=candidate.trajectory_.joint_trajectory;
      if(jt.joint_names.empty() || jt.points.empty())
        return "Empty planned trajectory";
      for(const auto & point:jt.points) {
        if(point.positions.size()!=jt.joint_names.size())
          return "Malformed trajectory: joint count mismatch";
        for(const double position:point.positions)
          if(!std::isfinite(position))return "Malformed trajectory: non-finite joint position";
      }
      double travel=0.0;
      double net_joint_travel=0.0;
      double largest_step=0.0;
      std::vector<double> individual_travel(jt.joint_names.size(),0.0);
      for(size_t j=0;j<jt.joint_names.size();++j)
        net_joint_travel+=std::abs(jt.points.back().positions[j]-jt.points.front().positions[j]);
      for(size_t i=1;i<jt.points.size();++i) {
        for(size_t j=0;j<jt.joint_names.size();++j) {
          const double d=std::abs(jt.points[i].positions[j]-jt.points[i-1].positions[j]);
          if(!std::isfinite(d)||d>0.50)return "Abrupt trajectory joint jump";
          individual_travel[j]+=d;
          largest_step=std::max(largest_step,d);
          travel+=d;
        }
      }
      RCLCPP_INFO(get_logger(),
          "[STATION] Planned %s: %zu waypoints; cumulative joint L1=%.3f rad; "
          "net joint L1=%.3f rad; largest step=%.3f rad; allowed L1=%.3f rad (%s)",
          station.c_str(),jt.points.size(),travel,net_joint_travel,largest_step,
          max_joint_travel_,robot_mode_.c_str());
      for(size_t j=0;j<jt.joint_names.size();++j) {
        RCLCPP_INFO(get_logger(),"[STATION] Joint %s: cumulative travel %.3f rad",
            jt.joint_names[j].c_str(),individual_travel[j]);
        if(individual_travel[j]>2.0*M_PI+0.1)
          return "Planned path winds joint "+jt.joint_names[j]+" more than one revolution";
      }
      if(travel>max_joint_travel_)
        return "Excessive joint-space path travel: "+std::to_string(travel)+
          " rad cumulative L1 exceeds configured "+std::to_string(max_joint_travel_)+
          " rad limit ("+robot_mode_+")";
      if(robot_mode_=="fake" && travel>6.0)
        RCLCPP_WARN(get_logger(),
          "FAKE HARDWARE ONLY: using extended aggregate joint-travel budget; "
          "collision and joint-jump validation remain mandatory");
      return {};
    };

    // Look ahead only in the synthetic mock fixture: the mock tooth frame is
    // known before the first image is acquired. Its surface normal comes from
    // the static fake jaw transform. The real system must instead consider a
    // measured target AFTER inspection and must not use this shortcut.
    //
    // Most importantly, an inspection pose that is collision-free does not
    // imply its IK branch can subsequently reach a tooth. Probe the NEXT
    // transfer before executing the very first inspection transfer.
    auto preview_fake_treatment = [&](const MoveGroup::Plan & inspection_plan,
                                      size_t inspect_candidate)->double {
      if(!fake_probe_enabled_) return 0.0;
      const auto & inspect_traj=inspection_plan.trajectory_.joint_trajectory;
      if(inspect_traj.points.empty() || inspect_traj.joint_names.empty())
        return std::numeric_limits<double>::infinity();
      // Construct the exact projected joint state at the inspection endpoint.
      auto projected=std::make_shared<moveit::core::RobotState>(*state);
      const auto & endpoint=inspect_traj.points.back().positions;
      if(endpoint.size()!=inspect_traj.joint_names.size())
        return std::numeric_limits<double>::infinity();
      for(size_t j=0;j<endpoint.size();++j)
        projected->setVariablePosition(inspect_traj.joint_names[j],endpoint[j]);
      projected->update();
      const auto tooth_tf=tf_buffer_->lookupTransform(
          base_,fake_probe_frame_,rclcpp::Time(0),rclcpp::Duration::from_seconds(1.0));
      const auto base_T_tooth=toEigen(tooth_tf.transform);
      const Eigen::Vector3d normal=requested_model_tf.linear().col(2).normalized();
      const Eigen::Vector3d tip_xyz=base_T_tooth.translation()+fake_probe_pre_m_*normal;
      const Eigen::Isometry3d base_T_tool_projected=
          projected->getGlobalLinkTransform(base_).inverse()*
          projected->getGlobalLinkTransform(tool_);
      const Eigen::Vector3d brush_offset(fake_probe_brush_offset_[0],
          fake_probe_brush_offset_[1],fake_probe_brush_offset_[2]);
      const auto * probe_jmg=model->getJointModelGroup(group_name_);
      if(!probe_jmg) return std::numeric_limits<double>::infinity();
      const std::array<double,4> yaws{{0.0,M_PI/2.0,-M_PI/2.0,M_PI}};
      double shortest=std::numeric_limits<double>::infinity();
      size_t winning_yaw=0;
      for(size_t yaw_index=0;yaw_index<yaws.size();++yaw_index) {
        if(cancel_.load()||h->is_canceling())
          throw std::runtime_error("Cancelled during fake inspection look-ahead");
        const Eigen::Matrix3d rotation=Eigen::AngleAxisd(yaws[yaw_index],normal).toRotationMatrix()*
            base_T_tool_projected.linear();
        const Eigen::Vector3d tool_goal=tip_xyz-rotation*brush_offset;
        if((tool_goal-base_T_tool_projected.translation()).norm()>fake_probe_move_m_)
          continue;
        Eigen::Isometry3d base_T_goal=Eigen::Isometry3d::Identity();
        base_T_goal.translation()=tool_goal;
        base_T_goal.linear()=rotation;
        const Eigen::Isometry3d model_T_goal=
            projected->getGlobalLinkTransform(base_)*base_T_goal;
        for(int method=0;method<2;++method) {
          if(cancel_.load()||h->is_canceling())
            throw std::runtime_error("Cancelled during fake inspection look-ahead");
          group->setStartState(*projected);
          bool target_set=false;
          if(method==0) {
            auto ik_state=std::make_shared<moveit::core::RobotState>(*projected);
            const std::vector<double> limits(probe_jmg->getVariableCount(),1.5);
            if(ik_state->setFromIK(probe_jmg,model_T_goal,tool_,limits,0.20)) {
              ik_state->update();
              std::vector<double> before,after;
              projected->copyJointGroupPositions(probe_jmg,before);
              ik_state->copyJointGroupPositions(probe_jmg,after);
              bool nearby=before.size()==after.size();
              for(size_t k=0;k<before.size() && nearby;++k)
                nearby=std::isfinite(after[k]) && std::abs(after[k]-before[k])<=1.5;
              if(nearby) target_set=group->setJointValueTarget(*ik_state);
            }
          } else {
            const Eigen::Quaterniond q(rotation);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id=base_;pose.header.stamp=now();
            pose.pose.position.x=tool_goal.x();pose.pose.position.y=tool_goal.y();
            pose.pose.position.z=tool_goal.z();
            pose.pose.orientation.x=q.x();pose.pose.orientation.y=q.y();
            pose.pose.orientation.z=q.z();pose.pose.orientation.w=q.w();
            target_set=group->setPoseTarget(pose,tool_);
          }
          if(!target_set) {group->clearPoseTargets();continue;}
          MoveGroup::Plan lookahead;
          // This is a planning-only feasibility test; never execute its result.
          group->setPlanningTime(2.0);
          const auto result=group->plan(lookahead);
          group->clearPoseTargets();
          group->setPlanningTime(12.0);
          if(result!=moveit::core::MoveItErrorCode::SUCCESS) continue;
          const auto & jt=lookahead.trajectory_.joint_trajectory;
          if(jt.points.size()<2 || jt.joint_names.size()!=probe_jmg->getVariableCount())
            continue;
          bool valid=true;
          std::vector<double> per_joint(jt.joint_names.size(),0.0);
          double l1=0.0;
          for(size_t k=1;k<jt.points.size() && valid;++k) {
            if(jt.points[k].positions.size()!=per_joint.size() ||
               jt.points[k-1].positions.size()!=per_joint.size()) {valid=false;break;}
            for(size_t j=0;j<per_joint.size();++j) {
              const double change=std::abs(jt.points[k].positions[j]-
                                           jt.points[k-1].positions[j]);
              if(!std::isfinite(change)||change>0.50) {valid=false;break;}
              per_joint[j]+=change;l1+=change;
              if(per_joint[j]>1.5 || l1>6.0) {valid=false;break;}
            }
          }
          // Reject a plan that silently changes the candidate start state.
          if(valid) for(size_t j=0;j<jt.joint_names.size();++j) {
            const double expected=projected->getVariablePosition(jt.joint_names[j]);
            if(std::abs(jt.points.front().positions[j]-expected)>0.03)
              valid=false;
          }
          if(valid && l1<shortest) {shortest=l1;winning_yaw=yaw_index;}
        }
      }
      RCLCPP_INFO(get_logger(),
          "[STATION] Fake inspect candidate %zu treatment look-ahead: %s, "
          "shortest guarded preapproach L1=%.3f rad (yaw %zu/4)",
          inspect_candidate,std::isfinite(shortest)?"PASS":"REJECTED",
          shortest,winning_yaw+1);
      return shortest;
    };

    MoveGroup::Plan plan;
    if(station=="inspect" && robot_mode_=="fake" && !reuse_fake_inspection) {
      bool accepted=false;
      double best_score=std::numeric_limits<double>::infinity();
      double best_travel=std::numeric_limits<double>::infinity();
      size_t best_index=0;
      std::string last_failure="No valid inspection trajectory";
      // Candidate count can be smaller if a yaw exceeded the travel bound.
      for(size_t i=0;i<fake_inspection_candidates.size();++i) {
        if(cancel_.load()||h->is_canceling())
          throw std::runtime_error("Cancelled during inspection planning");
        const auto & pose=fake_inspection_candidates[i];
        RCLCPP_INFO(get_logger(),
          "[STATION] Fake inspect pose candidate %zu/%zu: camera yaw variant; "
          "tool0 target=[%.3f, %.3f, %.3f] m",
          i+1,fake_inspection_candidates.size(),pose.pose.position.x,
          pose.pose.position.y,pose.pose.position.z);
        group->setStartStateToCurrentState();
        group->setPoseTarget(pose,tool_);
        MoveGroup::Plan attempt;
        const auto status=group->plan(attempt);
        group->clearPoseTargets();
        if(status!=moveit::core::MoveItErrorCode::SUCCESS) {
          last_failure="MoveIt could not plan a collision-free inspection yaw variant";
          RCLCPP_WARN(get_logger(),"[STATION] Rejected inspection candidate %zu: %s",
              i+1,last_failure.c_str());
          continue;
        }
        const auto reason=validate_trajectory(attempt);
        if(!reason.empty()) {
          last_failure=reason;
          RCLCPP_WARN(get_logger(),"[STATION] Rejected inspection candidate %zu: %s",
              i+1,reason.c_str());
          continue;
        }
        // MoveIt/OMPL is stochastic. A safe first plan can wind the wrist
        // several radians, while another valid optical yaw is much shorter.
        // Evaluate every candidate and keep the shortest validated joint path.
        // Do not change joint travel limits to accommodate a random IK branch.
        double candidate_travel=0.0;
        const auto & jt=attempt.trajectory_.joint_trajectory;
        for(size_t k=1;k<jt.points.size();++k)
          for(size_t j=0;j<jt.joint_names.size();++j)
            candidate_travel+=std::abs(jt.points[k].positions[j]-jt.points[k-1].positions[j]);
        RCLCPP_INFO(get_logger(),
          "[STATION] Valid candidate %zu/%zu has %.3f rad cumulative travel",
          i+1,fake_inspection_candidates.size(),candidate_travel);
        // Consider the next (synthetic) tooth approach in the score, not only
        // the first inspection transfer. A non-reachable IK branch must never
        // become the cached inspection pose for the remainder of the cycle.
        const double preview_l1=preview_fake_treatment(attempt,i+1);
        if(!std::isfinite(preview_l1)) {
          last_failure="Inspection IK branch cannot reach mock tooth without collision "
                       "or exceeding guarded preapproach travel";
          continue;
        }
        const double score=candidate_travel+preview_l1;
        if(score<best_score) {
          best_score=score;
          best_travel=candidate_travel;
          best_index=i;
          plan=std::move(attempt);
          accepted=true;
        }
      }
      if(!accepted)
        throw std::runtime_error("All fake inspection pose candidates rejected; "+last_failure+
            ". Inspect joint targets and collision geometry; do not raise safety limits");
      RCLCPP_INFO(get_logger(),
          "[STATION] Selected shortest valid fake inspect candidate %zu/%zu: "
          "joint L1 %.3f rad (limit %.3f)",
          best_index+1,fake_inspection_candidates.size(),best_travel,max_joint_travel_);
    } else {
      const auto planning=group->plan(plan);
      group->clearPoseTargets();
      if(planning!=moveit::core::MoveItErrorCode::SUCCESS)
        throw std::runtime_error("MoveIt could not plan collision-free "+station+" motion");
      const auto reason=validate_trajectory(plan);
      if(!reason.empty())throw std::runtime_error(reason);
      // The return to inspection is a small transfer from the previous
      // verified retract pose, not another full workcell traversal. If OMPL
      // chooses a winding route to the *same* saved joints, reject it.
      if(reuse_fake_inspection) {
        double revisit_l1 = 0.0;
        const auto & jt = plan.trajectory_.joint_trajectory;
        for(size_t k=1;k<jt.points.size();++k)
          for(size_t j=0;j<jt.joint_names.size();++j)
            revisit_l1 += std::abs(jt.points[k].positions[j] - jt.points[k-1].positions[j]);
        RCLCPP_INFO(get_logger(),
            "[STATION] REVISIT_inspect: saved joint target; path L1=%.3f rad (max 4.000)",
            revisit_l1);
        if(revisit_l1>4.0)
          throw std::runtime_error("Reinspection joint path winds excessively; no motion commanded");
      }
    }
    if(cancel_.load()||h->is_canceling())throw std::runtime_error("Cancelled before motion");
    if(!(robot_mode_=="fake" ? execute_fake_ : execute_real_))
      throw std::runtime_error("PLAN ONLY: enable an explicitly verified controller to execute");
    if(robot_mode_=="real" && !scene_ready_.load())
      throw std::runtime_error("Lost measured workcell scene before execution");
    feedback(h,"EXECUTING_"+station);
    const auto result=group->execute(plan);
    if(cancel_.load()||h->is_canceling()||result!=moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Station execution canceled or failed; manual inspection required");
    // Do not assume a controller SUCCESS means the measured joint state is
    // precisely at the goal. Check the physical/current state before caching
    // a pose that another task will rely on. Fail closed on missing feedback.
    auto after = group->getCurrentState(5.0);
    if(!after) throw std::runtime_error("Station executed, but post-execution joint state unavailable");
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if(trajectory.points.empty() || trajectory.joint_names.empty())
      throw std::runtime_error("Station executed with malformed trajectory result");
    const auto & end_positions = trajectory.points.back().positions;
    if(end_positions.size() != trajectory.joint_names.size())
      throw std::runtime_error("Station endpoint joint names/positions mismatch");
    double max_endpoint_error = 0.0;
    const auto & state_variable_names = after->getVariableNames();
    for(size_t j=0;j<end_positions.size();++j) {
      if(std::find(state_variable_names.begin(), state_variable_names.end(),
                   trajectory.joint_names[j]) == state_variable_names.end())
        throw std::runtime_error("Executed station joint missing from measured state: " +
            trajectory.joint_names[j]);
      const double actual = after->getVariablePosition(trajectory.joint_names[j]);
      if(!std::isfinite(actual) || !std::isfinite(end_positions[j]))
        throw std::runtime_error("Non-finite post-execution joint state");
      max_endpoint_error = std::max(max_endpoint_error,
          std::abs(actual-end_positions[j]));
    }
    RCLCPP_INFO(get_logger(),"[STATION] %s actual/planned endpoint max joint error %.5f rad",
        station.c_str(),max_endpoint_error);
    if(max_endpoint_error > 0.03)
      throw std::runtime_error("Station endpoint differs from measured joints >0.03 rad");
    feedback(h,"EXECUTED_"+station);
    if(robot_mode_=="real" && station=="home") {
      home_reached_=true;
      publishHomeReady(true);
      RCLCPP_INFO(get_logger(),"[STATION] COMMISSIONED_HOME_VERIFIED: six measured joints reached");
    }
    if(station=="home") {
      home_joints_.clear();
      home_locked_=false;
      inspection_joints_.clear();
      inspection_locked_=false;
    } else if(station=="inspect") {
      if(robot_mode_=="fake" && !inspection_locked_) {
        after->copyJointGroupPositions(jmg,inspection_joints_);
        inspection_model_tf_=requested_model_tf;
        inspection_locked_=true;
        RCLCPP_INFO(get_logger(),
            "[STATION] Saved verified first-pass fake inspection joint configuration "
            "(%zu joints) for deterministic reinspection",inspection_joints_.size());
      }
      home_locked_=true; // preserve original home through both inspections
    }
    done(h,true,"Reached "+station+" using verified no-contact MoveIt trajectory");
  }
  std::string robot_mode_,base_,tool_,camera_,model_,group_name_;
  double inspection_height_{},max_first_move_{},max_tf_age_{},max_joint_travel_{},velocity_{};
  bool execute_fake_{false},confirm_fake_{false},execute_real_{false};
  bool fake_probe_enabled_{false};
  std::string fake_probe_frame_;
  double fake_probe_pre_m_{0.15},fake_probe_move_m_{0.35};
  std::vector<double> fake_probe_brush_offset_;
  bool calibrated_{false},workcell_confirmed_{false};
  std::vector<double> home_joints_;
  bool home_locked_{false}; // only lock after the first successful inspect move
  // Fake-only, per-cycle cache. Cleared only after a successful return home.
  std::vector<double> inspection_joints_;
  Eigen::Isometry3d inspection_model_tf_{Eigen::Isometry3d::Identity()};
  bool inspection_locked_{false};
  std::atomic<bool> busy_{false},cancel_{false},scene_ready_{false};
  std::mutex group_mutex_;
  std::shared_ptr<MoveGroup> active_group_;
  std::thread worker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scene_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr home_ready_pub_;
  std::vector<double> commissioned_home_joints_;
  bool home_reached_{false};
  rclcpp_action::Server<Action>::SharedPtr server_;
};
int main(int argc,char ** argv){
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<StationMotion>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);executor.spin();executor.remove_node(node);
    node.reset();
  }catch(const std::exception & e){
    RCLCPP_FATAL(rclcpp::get_logger("station_motion"),"Fatal: %s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
