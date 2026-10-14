#include "motion_lease.hpp"

#include <capabilities2_runner/runner_base.hpp>
#include <hri_capability_interfaces/srv/move_relative.hpp>
#include <naoqi_utilities_msgs/srv/move_to.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace hri_naoqi_providers
{
namespace
{
using PortableMove = hri_capability_interfaces::srv::MoveRelative;
using NaoqiMove = naoqi_utilities_msgs::srv::MoveTo;
using Odometry = nav_msgs::msg::Odometry;

constexpr char kPortableMoveType[] = "hri_capability_interfaces/srv/MoveRelative";
constexpr char kDefaultPortableService[] = "/hri/move_relative";
constexpr char kNaoqiMoveService[] = "/nao/move_to";
constexpr char kOdometryTopic[] = "/nao/odom";
constexpr double kMaximumXMetres = 0.5;
constexpr double kMaximumYMetres = 0.3;
constexpr double kMaximumThetaRadians = 1.5707963267948966;
constexpr double kPositionToleranceMetres = 0.05;
constexpr double kAngularToleranceRadians = 0.15;
constexpr double kZeroTolerance = 1e-6;
constexpr auto kOdometryMaximumAge = std::chrono::seconds(1);
constexpr auto kMotionTimeout = std::chrono::seconds(30);
constexpr auto kOdometrySettleDelay = std::chrono::milliseconds(500);

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PendingRequest
{
  std::uint64_t id{0};
  std::shared_ptr<rclcpp::Service<PortableMove>> service;
  std::shared_ptr<rmw_request_id_t> request_header;
  std::string provider;
  Pose2D initial_pose;
  double target_x{0.0};
  double target_y{0.0};
  double target_theta{0.0};
  rclcpp::TimerBase::SharedPtr timeout;
  rclcpp::TimerBase::SharedPtr settle_timer;
};

struct ProviderState
{
  std::mutex mutex;
  bool running{true};
  bool faulted{false};
  bool has_odometry{false};
  std::uint64_t next_request_id{0};
  std::optional<PendingRequest> pending;
  std::string provider;
  Pose2D latest_pose;
  std::chrono::steady_clock::time_point odometry_received_at;
  rclcpp::Client<NaoqiMove>::SharedPtr move_backend;
  std::shared_ptr<MotionLease> motion_lease;
};

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2D pose_from_odometry(const Odometry & odometry)
{
  const auto & position = odometry.pose.pose.position;
  const auto & orientation = odometry.pose.pose.orientation;
  const double sin_yaw = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z);
  return Pose2D{position.x, position.y, std::atan2(sin_yaw, cos_yaw)};
}

void cancel_timers(PendingRequest & pending)
{
  if (pending.timeout) {
    pending.timeout->cancel();
  }
  if (pending.settle_timer) {
    pending.settle_timer->cancel();
  }
}

void send_response(
  const std::shared_ptr<rclcpp::Service<PortableMove>> & service,
  const std::shared_ptr<rmw_request_id_t> & request_header,
  const bool success,
  const std::string & provider,
  const std::string & message,
  const Pose2D & achieved = Pose2D{})
{
  auto response = std::make_shared<PortableMove::Response>();
  response->success = success;
  response->provider = provider;
  response->message = message;
  response->achieved_x_m = achieved.x;
  response->achieved_y_m = achieved.y;
  response->achieved_theta_rad = achieved.yaw;
  try {
    service->send_response(*request_header, *response);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      rclcpp::get_logger("hri_naoqi_providers.move_relative"),
      "Could not send MoveRelative response: %s", error.what());
  }
}

std::optional<PendingRequest> take_pending(
  const std::shared_ptr<ProviderState> & state,
  const std::uint64_t request_id,
  const bool fault_provider = false)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->pending || state->pending->id != request_id) {
    return std::nullopt;
  }
  if (fault_provider) {
    state->faulted = true;
    state->motion_lease->latch_until_process_exit();
    RCLCPP_ERROR(
      rclcpp::get_logger("hri_naoqi_providers.move_relative"),
      "Motion state uncertain; robot motion lease remains locked until server exit. "
      "Verify physical stop before restarting the server.");
  }
  auto pending = std::move(state->pending);
  state->pending.reset();
  cancel_timers(*pending);
  return pending;
}

void finish_request(
  const std::shared_ptr<ProviderState> & state,
  const std::uint64_t request_id,
  const bool success,
  const std::string & message,
  const Pose2D & achieved = Pose2D{},
  const bool fault_provider = false)
{
  auto pending = take_pending(state, request_id, fault_provider);
  if (!pending) {
    return;
  }
  send_response(
    pending->service, pending->request_header, success,
    pending->provider, message, achieved);
}

Pose2D relative_motion(const Pose2D & initial, const Pose2D & final_pose)
{
  const double delta_x = final_pose.x - initial.x;
  const double delta_y = final_pose.y - initial.y;
  const double cosine = std::cos(initial.yaw);
  const double sine = std::sin(initial.yaw);
  return Pose2D{
    cosine * delta_x + sine * delta_y,
    -sine * delta_x + cosine * delta_y,
    normalize_angle(final_pose.yaw - initial.yaw)};
}

void verify_motion(
  const std::shared_ptr<ProviderState> & state,
  const std::uint64_t request_id)
{
  Pose2D initial;
  Pose2D current;
  double target_x{0.0};
  double target_y{0.0};
  double target_theta{0.0};
  bool odometry_is_fresh{false};
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!state->pending || state->pending->id != request_id) {
      return;
    }
    initial = state->pending->initial_pose;
    target_x = state->pending->target_x;
    target_y = state->pending->target_y;
    target_theta = state->pending->target_theta;
    current = state->latest_pose;
    odometry_is_fresh = state->has_odometry &&
      std::chrono::steady_clock::now() - state->odometry_received_at <=
      kOdometryMaximumAge;
  }

  if (!odometry_is_fresh) {
    finish_request(
      state, request_id, false, "odometry_unavailable_after_motion", Pose2D{}, true);
    return;
  }

  const Pose2D achieved = relative_motion(initial, current);
  const double position_error = std::hypot(
    achieved.x - target_x, achieved.y - target_y);
  const double angular_error = std::abs(normalize_angle(
    achieved.yaw - target_theta));
  const bool reached = position_error <= kPositionToleranceMetres &&
    angular_error <= kAngularToleranceRadians;
  finish_request(
    state, request_id, reached,
    reached ? "motion_completed" : "motion_target_not_reached",
    achieved, !reached);
}
}  // namespace

class NaoqiMoveRelativeRunner : public capabilities2_runner::RunnerBase
{
public:
  ~NaoqiMoveRelativeRunner()
  {
    try {
      stop();
    } catch (const std::exception &) {
      // ROS teardown may prevent reporting a final stopped event.
    }
  }

  void start(
    rclcpp::Node::SharedPtr node,
    const capabilities2_runner::runner_opts & config,
    std::function<void(const std::string &)> started = nullptr,
    std::function<void(const std::string &)> terminated = nullptr,
    std::function<void(const std::string &)> stopped = nullptr) override
  {
    init_base(node, config, started, terminated, stopped);

    std::string service_name{kDefaultPortableService};
    try {
      service_name = get_service_name_by_type(kPortableMoveType);
    } catch (const capabilities2_runner::runner_exception &) {
      RCLCPP_WARN(
        node->get_logger(),
        "No MoveRelative resource was configured; using %s",
        kDefaultPortableService);
    }

    auto state = std::make_shared<ProviderState>();
    state->provider = config.provider;
    state->move_backend = node->create_client<NaoqiMove>(kNaoqiMoveService);
    // Existing Pepper launch files remap move_to but not necessarily cmd_vel.
    // Both adapters in this package use the robot namespace for the lease key.
    const std::string resolved_move = state->move_backend->get_service_name();
    const auto separator = resolved_move.find_last_of('/');
    const std::string velocity_topic = resolved_move.substr(0, separator + 1) + "cmd_vel";
    state->motion_lease = std::make_shared<MotionLease>(velocity_topic);

    auto odometry_callback = [state](const Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->running) {
          return;
        }
        state->latest_pose = pose_from_odometry(*message);
        state->odometry_received_at = std::chrono::steady_clock::now();
        state->has_odometry = true;
      };

    auto callback =
      [state, node](
        std::shared_ptr<rclcpp::Service<PortableMove>> service,
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<PortableMove::Request> request) {
        const bool finite = std::isfinite(request->x_m) &&
          std::isfinite(request->y_m) && std::isfinite(request->theta_rad);
        const bool zero_motion = std::abs(request->x_m) <= kZeroTolerance &&
          std::abs(request->y_m) <= kZeroTolerance &&
          std::abs(request->theta_rad) <= kZeroTolerance;
        std::string provider;
        std::uint64_t request_id{0};

        {
          std::lock_guard<std::mutex> lock(state->mutex);
          provider = state->provider;
          if (!state->running) {
            send_response(service, request_header, false, provider, "provider_stopped");
            return;
          }
          if (state->faulted) {
            send_response(
              service, request_header, false, provider,
              "provider_motion_state_uncertain");
            return;
          }
          if (!finite) {
            send_response(
              service, request_header, false, provider,
              "request_contains_non_finite_value");
            return;
          }
          if (zero_motion) {
            send_response(service, request_header, false, provider, "movement_is_zero");
            return;
          }
          if (std::abs(request->x_m) > kMaximumXMetres ||
            std::abs(request->y_m) > kMaximumYMetres ||
            std::abs(request->theta_rad) > kMaximumThetaRadians)
          {
            send_response(
              service, request_header, false, provider,
              "movement_exceeds_provider_limits");
            return;
          }
          if (state->pending) {
            send_response(service, request_header, false, provider, "provider_busy");
            return;
          }
          if (!state->move_backend->service_is_ready()) {
            send_response(
              service, request_header, false, provider,
              "move_to_service_unavailable");
            return;
          }
          const bool odometry_is_fresh = state->has_odometry &&
            std::chrono::steady_clock::now() - state->odometry_received_at <=
            kOdometryMaximumAge;
          if (!odometry_is_fresh) {
            send_response(
              service, request_header, false, provider,
              "odometry_unavailable_before_motion");
            return;
          }

          request_id = ++state->next_request_id;
          state->pending.emplace(PendingRequest{
            request_id, service, request_header, provider, state->latest_pose,
            request->x_m, request->y_m, request->theta_rad, nullptr, nullptr});
          std::weak_ptr<ProviderState> weak_state{state};
          state->pending->timeout = node->create_wall_timer(
            kMotionTimeout,
            [weak_state, request_id]() {
              const auto locked_state = weak_state.lock();
              if (locked_state) {
                finish_request(
                  locked_state, request_id, false, "motion_timeout",
                  Pose2D{}, true);
              }
            });
        }

        auto move_request = std::make_shared<NaoqiMove::Request>();
        move_request->x_coordinate = static_cast<float>(request->x_m);
        move_request->y_coordinate = static_cast<float>(request->y_m);
        move_request->theta_coordinate = static_cast<float>(request->theta_rad);
        try {
          state->move_backend->async_send_request(
            move_request,
            [state, node, request_id](rclcpp::Client<NaoqiMove>::SharedFuture future) {
              try {
                static_cast<void>(future.get());
              } catch (const std::exception & error) {
                finish_request(
                  state, request_id, false,
                  std::string("move_to_request_failed: ") + error.what(),
                  Pose2D{}, true);
                return;
              }

              std::lock_guard<std::mutex> lock(state->mutex);
              if (!state->pending || state->pending->id != request_id) {
                return;
              }
              std::weak_ptr<ProviderState> weak_state{state};
              state->pending->settle_timer = node->create_wall_timer(
                kOdometrySettleDelay,
                [weak_state, request_id]() {
                  const auto locked_state = weak_state.lock();
                  if (locked_state) {
                    verify_motion(locked_state, request_id);
                  }
                });
            });
        } catch (const std::exception & error) {
          finish_request(
            state, request_id, false,
            std::string("move_to_request_failed: ") + error.what(),
            Pose2D{}, true);
        }
      };

    state_ = state;
    odometry_subscription_ = node->create_subscription<Odometry>(
      kOdometryTopic, rclcpp::QoS(10), std::move(odometry_callback));
    service_ = node->create_service<PortableMove>(service_name, std::move(callback));

    if (on_started_) {
      on_started_(config.interface);
    }
  }

  void stop() override
  {
    const auto state = state_;
    if (!state) {
      return;
    }

    std::optional<PendingRequest> pending;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->running) {
        return;
      }
      state->running = false;
      if (state->pending) {
        state->motion_lease->latch_until_process_exit();
        RCLCPP_ERROR(
          rclcpp::get_logger("hri_naoqi_providers.move_relative"),
          "Released during a pending motion: physical cancellation is unavailable. "
          "Motion lease remains locked until server exit; verify physical stop first.");
      }
      pending = std::move(state->pending);
      state->pending.reset();
    }
    if (pending) {
      cancel_timers(*pending);
      send_response(
        pending->service, pending->request_header, false,
        pending->provider, "provider_stopped");
    }

    service_.reset();
    odometry_subscription_.reset();
    state_.reset();

    if (on_stopped_) {
      on_stopped_(run_config_.interface);
    }
  }

  std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>> trigger(
    std::shared_ptr<tinyxml2::XMLElement>) override
  {
    return std::nullopt;
  }

private:
  std::shared_ptr<ProviderState> state_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Service<PortableMove>::SharedPtr service_;
};
}  // namespace hri_naoqi_providers

PLUGINLIB_EXPORT_CLASS(
  hri_naoqi_providers::NaoqiMoveRelativeRunner,
  capabilities2_runner::RunnerBase)
