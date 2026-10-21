#include "motion_lease.hpp"

#include <capabilities2_runner/runner_base.hpp>
#include <hri_capability_interfaces/msg/camera_orientation.hpp>
#include <hri_capability_interfaces/srv/orient_camera.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace hri_naoqi_providers
{
namespace
{
using Clock = std::chrono::steady_clock;
using Orientation = hri_capability_interfaces::msg::CameraOrientation;
using Orient = hri_capability_interfaces::srv::OrientCamera;
constexpr double kMaximumElevation = 0.25;
constexpr double kMaximumStep = 0.03;
constexpr double kYawTolerance = 0.08;
constexpr double kPositionTolerance = 0.008;
constexpr double kMaximumFeedbackAge = 0.5;

struct CameraState
{
  std::mutex mutex;
  bool running{true};
  bool received{false};
  bool commanded{false};
  bool moving{false};
  unsigned settled{0};
  int64_t stamp_ns{0};
  double elevation{0.0};
  double azimuth{0.0};
  double target{0.0};
  Clock::time_point received_at;
  Clock::time_point command_at;
  Clock::time_point started_at{Clock::now()};
  std::string fault;
  std::string feedback_topic;
  std::string service_name;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr backend;
  rclcpp::Publisher<Orientation>::SharedPtr state_publisher;
  std::shared_ptr<MotionLease> lease;
};

bool fresh(const CameraState & state)
{
  if (!state.received) {
    return false;
  }
  const double age = (state.node->now().nanoseconds() - state.stamp_ns) * 1e-9;
  return age >= -0.05 && age <= kMaximumFeedbackAge &&
         std::chrono::duration<double>(Clock::now() - state.received_at).count() <=
         kMaximumFeedbackAge;
}

void send_target(CameraState & state, double elevation)
{
  trajectory_msgs::msg::JointTrajectory message;
  message.header.stamp = state.node->now();
  message.joint_names = {"HeadPitch"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {-elevation};
  // This NAOqi driver interprets velocities[0] as setAngles speed FRACTION,
  // not rad/s, and ignores time_from_start. Never command HeadYaw or stiffness.
  point.velocities = {0.05};
  message.points = {point};
  state.backend->publish(message);
}

void fault(CameraState & state, const std::string & reason)
{
  if (!state.fault.empty()) {
    return;
  }
  state.fault = reason;
  if (state.commanded && fresh(state) && state.backend->get_subscription_count() > 0) {
    send_target(state, state.elevation);
  }
  state.commanded = false;
  state.moving = false;
}

bool healthy(CameraState & state)
{
  if (!state.fault.empty()) {
    return false;
  }
  const auto node = state.node;
  if (node->get_publishers_info_by_topic(state.backend->get_topic_name()).size() > 1 ||
    node->get_publishers_info_by_topic(state.state_publisher->get_topic_name()).size() > 1 ||
    node->get_publishers_info_by_topic(state.feedback_topic).size() > 1)
  {
    fault(state, "camera_publisher_conflict");
    return false;
  }
  unsigned services = 0;
  for (const auto & name : node->get_node_graph_interface()->get_node_names()) {
    const auto slash = name.find_last_of('/');
    const auto short_name = name.substr(slash + 1);
    const auto ns = slash == 0 ? "/" : name.substr(0, slash);
    const auto offered = node->get_service_names_and_types_by_node(short_name, ns);
    services += offered.count(state.service_name);
  }
  if (services > 1) {
    fault(state, "camera_service_conflict");
    return false;
  }
  if (!fresh(state)) {
    if (state.received) {
      fault(state, "stale_camera_feedback");
    }
    return false;
  }
  if (std::abs(state.azimuth) > kYawTolerance ||
    std::abs(state.elevation) > kMaximumElevation + kPositionTolerance)
  {
    fault(state, "camera_pose_out_of_bounds");
    return false;
  }
  if (state.backend->get_subscription_count() == 0) {
    if (state.commanded) {
      fault(state, "camera_backend_unavailable");
    }
    return false;
  }
  return Clock::now() - state.started_at >= std::chrono::milliseconds(500);
}

void publish_state(CameraState & state)
{
  Orientation message;
  message.header.stamp = rclcpp::Time(state.stamp_ns);
  message.header.frame_id = "camera_mount";
  message.elevation = state.elevation;
  message.azimuth = state.azimuth;
  message.ready = healthy(state);
  message.moving = state.moving;
  message.provider = "hri_naoqi_providers/NaoqiOrientCamera";
  message.status = !state.fault.empty() ? state.fault :
    (!message.ready ? "awaiting_camera_backend" : (state.moving ? "moving" : "ready"));
  state.state_publisher->publish(message);
}

void feedback(CameraState & state, const sensor_msgs::msg::JointState & joints)
{
  if (!state.running || !state.fault.empty()) {
    return;
  }
  const auto pitch = std::find(joints.name.begin(), joints.name.end(), "HeadPitch");
  const auto yaw = std::find(joints.name.begin(), joints.name.end(), "HeadYaw");
  if (pitch == joints.name.end() || yaw == joints.name.end() ||
    joints.position.size() != joints.name.size() ||
    std::count(joints.name.begin(), joints.name.end(), "HeadPitch") != 1 ||
    std::count(joints.name.begin(), joints.name.end(), "HeadYaw") != 1)
  {
    fault(state, "invalid_camera_feedback");
    return;
  }
  if (joints.header.stamp.sec < 0 || joints.header.stamp.nanosec >= 1000000000U) {
    fault(state, "invalid_camera_feedback");
    return;
  }
  const auto stamp = rclcpp::Time(joints.header.stamp).nanoseconds();
  const double elevation = -joints.position[std::distance(joints.name.begin(), pitch)];
  const double azimuth = joints.position[std::distance(joints.name.begin(), yaw)];
  const double age = (state.node->now().nanoseconds() - stamp) * 1e-9;
  if (stamp <= 0 || stamp <= state.stamp_ns || age < -0.05 ||
    age > kMaximumFeedbackAge || !std::isfinite(elevation) || !std::isfinite(azimuth))
  {
    fault(state, "invalid_camera_feedback");
    return;
  }
  const bool stable = state.received && std::abs(elevation - state.elevation) <= 0.003;
  state.elevation = elevation;
  state.azimuth = azimuth;
  state.stamp_ns = stamp;
  state.received_at = Clock::now();
  state.received = true;
  if (state.commanded) {
    state.settled = stable && std::abs(elevation - state.target) <= kPositionTolerance ?
      state.settled + 1 : 0;
    state.moving = state.settled < 3;
  } else {
    state.moving = !stable;
  }
  publish_state(state);
}
}  // namespace

class NaoqiOrientCameraRunner : public capabilities2_runner::RunnerBase
{
public:
  ~NaoqiOrientCameraRunner()
  {
    try {stop();} catch (const std::exception &) { /* ROS context may already be gone. */ }
  }

  void start(
    rclcpp::Node::SharedPtr node, const capabilities2_runner::runner_opts & config,
    std::function<void(const std::string &)> started = nullptr,
    std::function<void(const std::string &)> terminated = nullptr,
    std::function<void(const std::string &)> stopped = nullptr) override
  {
    init_base(node, config, started, terminated, stopped);
    auto state = std::make_shared<CameraState>();
    state->node = node;
    const auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    state->backend = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/nao/joint_trajectory", qos);
    state->lease = std::make_shared<MotionLease>(state->backend->get_topic_name());
    state->state_publisher = node->create_publisher<Orientation>(
      get_topic_name_by_type("hri_capability_interfaces/msg/CameraOrientation"), qos);
    state->feedback_topic = node->get_node_topics_interface()->resolve_topic_name(
      "/nao/joint_states", false);
    subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      state->feedback_topic, rclcpp::SensorDataQoS(),
      [state](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        feedback(*state, *message);
      });
    service_ = node->create_service<Orient>(
      get_service_name_by_type("hri_capability_interfaces/srv/OrientCamera"),
      [state](const Orient::Request::SharedPtr request, Orient::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(state->mutex);
        response->accepted = false;
        if (!state->running || !healthy(*state)) {
          response->message = state->fault.empty() ? "camera_not_ready" : state->fault;
          return;
        }
        if (!std::isfinite(request->elevation) ||
          (!request->hold && (std::abs(request->elevation) > kMaximumElevation ||
          std::abs(request->elevation - state->elevation) > kMaximumStep + 1e-9)))
        {
          response->message = "invalid_camera_target";
          return;
        }
        if (!request->hold && state->moving) {
          response->message = "camera_busy";
          return;
        }
        state->target = request->hold ? state->elevation : request->elevation;
        state->settled = 0;
        state->moving = true;
        state->commanded = true;
        state->command_at = Clock::now();
        send_target(*state, state->target);
        response->accepted = true;
        response->message = request->hold ? "hold_requested" : "orientation_requested";
        publish_state(*state);
      });
    state->service_name = service_->get_service_name();
    state_ = state;
    timer_ = node->create_wall_timer(std::chrono::milliseconds(50), [state]() {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->running) {return;}
        if (state->commanded && state->moving &&
          Clock::now() - state->command_at > std::chrono::seconds(2))
        {
          fault(*state, "camera_target_timeout");
        }
        publish_state(*state);
      });
    if (on_started_) {on_started_(config.interface);}
  }

  void stop() override
  {
    const auto state = state_;
    if (!state) {return;}
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->running) {return;}
      state->running = false;
      if (state->commanded && fresh(*state) && state->backend->get_subscription_count() > 0) {
        send_target(*state, state->elevation);
      }
      // A hold publication is not a robot-side stop acknowledgement.
      state->fault = "released";
      state->moving = false;
      publish_state(*state);
    }
    if (timer_) {timer_->cancel();}
    timer_.reset();
    service_.reset();
    subscription_.reset();
    state_.reset();
    if (on_stopped_) {on_stopped_(run_config_.interface);}
  }

  std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>> trigger(
    std::shared_ptr<tinyxml2::XMLElement>) override {return std::nullopt;}

private:
  std::shared_ptr<CameraState> state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Service<Orient>::SharedPtr service_;
};
}  // namespace hri_naoqi_providers

PLUGINLIB_EXPORT_CLASS(
  hri_naoqi_providers::NaoqiOrientCameraRunner, capabilities2_runner::RunnerBase)
