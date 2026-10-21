#include <capabilities2_runner/runner_base.hpp>
#include <hri_capability_interfaces/msg/speech_transcript.hpp>
#include <hri_capability_interfaces/srv/transcribe_speech.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unistd.h>

namespace hri_whisper_providers
{
namespace
{
using Service = hri_capability_interfaces::srv::TranscribeSpeech;
using Transcript = hri_capability_interfaces::msg::SpeechTranscript;
using Clock = std::chrono::steady_clock;
struct State
{
  std::mutex mutex;
  bool running{true};
  std::string owner;
  std::string provider;
  std::string request_id;
  std::string public_id;
  std::uint64_t sequence{0};
  Clock::time_point deadline;
  rclcpp::Client<Service>::SharedPtr backend;
  rclcpp::Publisher<Transcript>::SharedPtr output;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lease;
  rclcpp::Service<Service>::SharedPtr pending_service;
  std::shared_ptr<rmw_request_id_t> pending_header;
  std::optional<std::int64_t> backend_sequence;
  Service::Request::SharedPtr awaiting_backend;
};

void reply(State & state, bool accepted, const std::string & message)
{
  if (!state.pending_service) {return;}
  Service::Response response;
  response.accepted = accepted;
  response.request_id = state.public_id;
  response.provider = state.provider;
  response.message = message;
  try {
    state.pending_service->send_response(*state.pending_header, response);
  } catch (const std::exception &) {
    RCLCPP_WARN(rclcpp::get_logger("whisper_proxy"), "Could not send transcription admission response");
  }
  state.pending_service.reset();
  state.pending_header.reset();
}

void clear_request(State & state)
{
  if (state.backend_sequence) {
    state.backend->remove_pending_request(*state.backend_sequence);
    state.backend_sequence.reset();
  }
  state.request_id.clear();
  state.public_id.clear();
  state.awaiting_backend.reset();
}

// Called with state mutex held; never waits for discovery or inference on the ROS executor.
void dispatch_when_ready(const std::shared_ptr<State> & state)
{
  if (!state->awaiting_backend || !state->backend->service_is_ready()) {return;}
  auto request = std::move(state->awaiting_backend);
  const auto id = state->request_id;
  const auto duration = request->duration_sec;
  try {
    auto future = state->backend->async_send_request(request,
      [state, id, duration](rclcpp::Client<Service>::SharedFuture result) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->running || state->request_id != id) {return;}
        state->backend_sequence.reset();
        try {
          const auto response = result.get();
          const bool accepted = response->accepted && response->request_id == id;
          reply(*state, accepted, accepted ? "capture_started" : response->message);
          if (!accepted) {clear_request(*state); return;}
          state->deadline = Clock::now() + std::chrono::milliseconds(static_cast<int>((duration + 62) * 1000));
        } catch (const std::exception &) {
          reply(*state, false, "worker_admission_failed");
          clear_request(*state);
        }
      });
    state->backend_sequence = future.request_id;
  } catch (const std::exception &) {
    reply(*state, false, "worker_admission_failed");
    clear_request(*state);
  }
}
}  // namespace

class WhisperTranscribeSpeechRunner : public capabilities2_runner::RunnerBase
{
public:
  void start(
    rclcpp::Node::SharedPtr node, const capabilities2_runner::runner_opts & config,
    std::function<void(const std::string &)> started = nullptr,
    std::function<void(const std::string &)> terminated = nullptr,
    std::function<void(const std::string &)> stopped = nullptr) override
  {
    init_base(node, config, started, terminated, stopped);
    auto state = std::make_shared<State>();
    static std::atomic<std::uint64_t> generations{0};
    state->owner = std::to_string(getpid()) + "-" + std::to_string(Clock::now().time_since_epoch().count()) +
      "-" + std::to_string(++generations);
    state->provider = config.provider;
    state->backend = node->create_client<Service>("/whisper/transcribe");
    state->lease = node->create_publisher<std_msgs::msg::String>("/whisper/lease", 1);
    std::string service_name{"/hri/speech/transcribe"};
    std::string topic_name{"/hri/speech/transcript"};
    try {service_name = get_service_name_by_type("hri_capability_interfaces/srv/TranscribeSpeech");}
    catch (const capabilities2_runner::runner_exception &) {}
    try {topic_name = get_topic_name_by_type("hri_capability_interfaces/msg/SpeechTranscript");}
    catch (const capabilities2_runner::runner_exception &) {}
    state->output = node->create_publisher<Transcript>(topic_name, 10);
    service_ = node->create_service<Service>(service_name,
      [state](std::shared_ptr<rclcpp::Service<Service>> service,
        std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Service::Request> request) {
        std::lock_guard<std::mutex> lock(state->mutex);
        auto reject = [&](const std::string & message) {
            Service::Response response;
            response.accepted = false;
            response.request_id = request->request_id;
            response.provider = state->provider;
            response.message = message;
            service->send_response(*header, response);
          };
        if (!state->running) {reject("provider_stopped"); return;}
        if (!state->request_id.empty()) {reject("provider_busy"); return;}
        if (!std::isfinite(request->duration_sec) || request->duration_sec < .5 || request->duration_sec > 10) {
          reject("duration_out_of_range"); return;
        }
        if (request->language != "" && request->language != "es" && request->language != "en") {
          reject("unsupported_language"); return;
        }
        if (request->request_id.size() > 128) {reject("request_id_too_long"); return;}
        // Public caller identifiers never choose a lease or collide with an earlier owner.
        state->request_id = state->owner + ":" + std::to_string(++state->sequence);
        state->public_id = request->request_id.empty() ? state->request_id : request->request_id;
        request->request_id = state->request_id;
        state->pending_service = service;
        state->pending_header = header;
        state->deadline = Clock::now() + std::chrono::seconds(2);
        state->awaiting_backend = request;
        dispatch_when_ready(state);
      });
    subscription_ = node->create_subscription<Transcript>("/whisper/transcript", 10,
      [state](Transcript::SharedPtr result) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->running || state->request_id.empty() || result->request_id != state->request_id) {return;}
        // A capture failure may beat service-response delivery on a separate DDS channel.
        if (state->pending_service) {reply(*state, true, "capture_completed");}
        result->provider = state->provider;
        result->request_id = state->public_id;
        state->output->publish(*result);
        clear_request(*state);
      });
    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [state]() {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->running) {return;}
        std_msgs::msg::String lease;
        lease.data = state->owner;
        state->lease->publish(lease);
        if (!state->request_id.empty() && Clock::now() > state->deadline) {
          if (state->pending_service) {
            reply(*state, false, state->awaiting_backend ? "whisper_worker_unavailable" : "worker_admission_timeout");
          } else {
            Transcript result;
            result.request_id = state->public_id;
            result.provider = state->provider;
            result.success = false;
            result.message = "worker_result_timeout";
            state->output->publish(result);
          }
          clear_request(*state);
          // Rotate ownership so a timed-out backend capture cannot later escape this request.
          state->owner += "r";
        }
        dispatch_when_ready(state);
      });
    state_ = state;
    if (on_started_) {on_started_(config.interface);}
  }

  void stop() override
  {
    auto state = state_;
    if (!state) {return;}
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->running) {return;}
      state->running = false;
      reply(*state, false, "provider_stopped");
      clear_request(*state);
      std_msgs::msg::String lease;
      state->lease->publish(lease);
    }
    timer_.reset();
    service_.reset();
    subscription_.reset();
    state_.reset();
    if (on_stopped_) {on_stopped_(run_config_.interface);}
  }

  std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>> trigger(
    std::shared_ptr<tinyxml2::XMLElement>) override {return std::nullopt;}

private:
  std::shared_ptr<State> state_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::Subscription<Transcript>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace hri_whisper_providers
PLUGINLIB_EXPORT_CLASS(hri_whisper_providers::WhisperTranscribeSpeechRunner, capabilities2_runner::RunnerBase)
