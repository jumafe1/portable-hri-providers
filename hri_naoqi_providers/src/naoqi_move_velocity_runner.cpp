#include "motion_lease.hpp"

#include <capabilities2_runner/runner_base.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace hri_naoqi_providers
{
namespace
{
using Twist = geometry_msgs::msg::Twist;
using Clock = std::chrono::steady_clock;
constexpr auto kPeriod = std::chrono::milliseconds(50);
constexpr auto kCommandMaximumAge = std::chrono::milliseconds(300);
constexpr auto kGraphSettleTime = std::chrono::milliseconds(500);

bool valid_command(const Twist & command)
{
  return std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
         std::isfinite(command.linear.z) && std::isfinite(command.angular.x) &&
         std::isfinite(command.angular.y) && std::isfinite(command.angular.z) &&
         command.linear.z == 0.0 && command.angular.x == 0.0 &&
         command.angular.y == 0.0 && std::abs(command.linear.x) <= 0.10 &&
         std::abs(command.linear.y) <= 0.05 && std::abs(command.angular.z) <= 0.20;
}

bool is_zero(const Twist & command)
{
  return command.linear.x == 0.0 && command.linear.y == 0.0 && command.angular.z == 0.0;
}

struct VelocityState
{
  std::mutex mutex;
  bool running{true};
  bool backend_ready{false};
  bool has_command{false};
  bool graph_ready{false};
  bool source_pinned{false};
  std::optional<Clock::time_point> graph_stable_since;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> source_gid{};
  std::string fault;
  std::string input_topic;
  std::weak_ptr<rclcpp::Node> node;
  Twist target;
  Twist output;
  Clock::time_point received_at;
  Clock::time_point last_tick{Clock::now()};
  std::string status{"backend_unavailable"};
  std::shared_ptr<MotionLease> lease;
  rclcpp::Publisher<Twist>::SharedPtr backend;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
};

void publish_status(const std::shared_ptr<VelocityState> & state)
{
  std_msgs::msg::String message;
  message.data = state->status;
  state->status_publisher->publish(message);
}

// Caller holds the state mutex. Stops bypass acceleration ramping.
void halt(const std::shared_ptr<VelocityState> & state, const std::string & reason)
{
  state->has_command = false;
  state->target = Twist{};
  state->output = Twist{};
  state->status = reason;
  state->backend->publish(state->output);
  publish_status(state);
}

// Caller holds the mutex. Graph discovery is asynchronous: this bounds the
// startup settling period, not a distributed lock or robot-side stop guarantee.
bool check_exclusivity(const std::shared_ptr<VelocityState> & state)
{
  if (!state->fault.empty()) {
    halt(state, state->fault);
    return false;
  }
  const auto node = state->node.lock();
  if (!node) {
    halt(state, "backend_unavailable");
    return false;
  }
  const auto inputs = node->get_publishers_info_by_topic(state->input_topic);
  const auto outputs = node->get_publishers_info_by_topic(state->backend->get_topic_name());
  if (inputs.size() > 1 || outputs.size() > 1) {
    state->fault = "multiple_velocity_publishers";
    halt(state, state->fault);
    return false;
  }
  if (inputs.size() != 1 || outputs.size() != 1) {
    state->graph_ready = false;
    state->graph_stable_since.reset();
    halt(state, "awaiting_velocity_source");
    return false;
  }
  const auto & gid = inputs.front().endpoint_gid();
  if (state->source_pinned && state->source_gid != gid) {
    state->fault = "velocity_source_changed";
    halt(state, state->fault);
    return false;
  }
  if (!state->source_pinned) {
    state->source_gid = gid;
    state->source_pinned = true;
  }
  const auto now = Clock::now();
  if (!state->graph_stable_since) {
    state->graph_stable_since = now;
  }
  if (now - *state->graph_stable_since < kGraphSettleTime) {
    halt(state, "graph_settling");
    return false;
  }
  if (!state->graph_ready) {
    state->graph_ready = true;
    halt(state, "ready");
  }
  return true;
}

void tick(const std::shared_ptr<VelocityState> & state)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->running) {
    return;
  }
  const auto now = Clock::now();
  // A delayed executor must not produce a large acceleration jump on recovery.
  const double dt = std::clamp(
    std::chrono::duration<double>(now - state->last_tick).count(), 0.0, 0.05);
  state->last_tick = now;
  if (!check_exclusivity(state)) {
    return;
  }
  if (state->backend->get_subscription_count() == 0) {
    state->backend_ready = false;
    halt(state, "backend_unavailable");
    return;
  }
  if (!state->backend_ready) {
    state->backend_ready = true;
    halt(state, "ready");
    return;
  }
  if (state->has_command && now - state->received_at >= kCommandMaximumAge) {
    halt(state, "command_timeout");
    return;
  }
  if (state->has_command) {
    const auto ramp = [dt](const double current, const double target, const double acceleration) {
        return current + std::clamp(target - current, -acceleration * dt, acceleration * dt);
      };
    state->output.linear.x = ramp(state->output.linear.x, state->target.linear.x, 0.10);
    state->output.linear.y = ramp(state->output.linear.y, state->target.linear.y, 0.10);
    state->output.angular.z = ramp(state->output.angular.z, state->target.angular.z, 0.40);
    state->status = is_zero(state->output) ? "stopped" : "moving";
  }
  state->backend->publish(state->output);
  publish_status(state);
}
}  // namespace

class NaoqiMoveVelocityRunner : public capabilities2_runner::RunnerBase
{
public:
  ~NaoqiMoveVelocityRunner()
  {
    try {
      stop();
    } catch (const std::exception &) {
      // During ROS shutdown a final publication may be impossible. This is not
      // a robot-side watchdog and cannot guarantee stopping after process loss.
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
    const auto resource_topic = [this, node](const char * type, const char * fallback) {
        try {
          return get_topic_name_by_type(type);
        } catch (const capabilities2_runner::runner_exception &) {
          RCLCPP_WARN(node->get_logger(), "No %s topic resource; using %s", type, fallback);
          return std::string(fallback);
        }
      };
    const auto input_topic = resource_topic("geometry_msgs/msg/Twist", "/hri/cmd_vel");
    const auto status_topic = resource_topic("std_msgs/msg/String", "/hri/move_velocity/status");
    auto state = std::make_shared<VelocityState>();
    state->node = node;
    state->input_topic = node->get_node_topics_interface()->resolve_topic_name(input_topic, false);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    state->backend = node->create_publisher<Twist>("/nao/cmd_vel", qos);
    if (node->get_node_topics_interface()->resolve_topic_name(input_topic, false) ==
      state->backend->get_topic_name())
    {
      throw capabilities2_runner::runner_exception("velocity_input_output_self_loop");
    }
    state->lease = std::make_shared<MotionLease>(state->backend->get_topic_name());
    state->status_publisher = node->create_publisher<std_msgs::msg::String>(
      status_topic, qos);
    halt(state, "backend_unavailable");
    subscription_ = node->create_subscription<Twist>(
      input_topic, qos,
      [state](const Twist::SharedPtr command, const rclcpp::MessageInfo & info) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->running) {
          return;
        }
        if (!check_exclusivity(state)) {
          return;
        }
        if (std::memcmp(info.get_rmw_message_info().publisher_gid.data,
            state->source_gid.data(), RMW_GID_STORAGE_SIZE) != 0)
        {
          state->fault = "velocity_source_changed";
          halt(state, state->fault);
          return;
        }
        if (!valid_command(*command)) {
          halt(state, "invalid_command");
          return;
        }
        if (!state->backend_ready || state->backend->get_subscription_count() == 0) {
          halt(state, "backend_unavailable");
          return;
        }
        state->received_at = Clock::now();
        if (is_zero(*command)) {
          halt(state, "stopped");
          return;
        }
        state->target = *command;
        state->has_command = true;
      });
    state_ = state;
    timer_ = node->create_wall_timer(kPeriod, [state]() {tick(state);});
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
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->running) {
        return;
      }
      state->running = false;
      halt(state, "stopped");
    }
    if (timer_) {
      timer_->cancel();
    }
    timer_.reset();
    subscription_.reset();
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
  std::shared_ptr<VelocityState> state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Twist>::SharedPtr subscription_;
};
}  // namespace hri_naoqi_providers

PLUGINLIB_EXPORT_CLASS(
  hri_naoqi_providers::NaoqiMoveVelocityRunner,
  capabilities2_runner::RunnerBase)
