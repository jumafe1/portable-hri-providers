#include <capabilities2_runner/runner_base.hpp>
#include <hri_capability_interfaces/srv/speak.hpp>
#include <naoqi_utilities_msgs/srv/say.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
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
using PortableSpeak = hri_capability_interfaces::srv::Speak;
using NaoSay = naoqi_utilities_msgs::srv::Say;

constexpr char kPortableSpeakType[] = "hri_capability_interfaces/srv/Speak";
constexpr char kDefaultSpeakService[] = "/hri/speak";
constexpr char kNaoSayService[] = "/nao/say";
constexpr char kDefaultLanguage[] = "Spanish";
constexpr auto kNaoSayTimeout = std::chrono::seconds(5);

bool is_blank(const std::string & text)
{
  return std::all_of(text.begin(), text.end(), [](const unsigned char character) {
    return std::isspace(character) != 0;
  });
}

struct PendingRequest
{
  std::uint64_t id;
  std::shared_ptr<rclcpp::Service<PortableSpeak>> service;
  std::shared_ptr<rmw_request_id_t> request_header;
  std::string provider;
  rclcpp::TimerBase::SharedPtr timeout;
};

struct ProviderState
{
  std::mutex mutex;
  bool running{true};
  std::uint64_t next_request_id{0};
  std::optional<PendingRequest> pending;
  std::string provider;
  rclcpp::Client<NaoSay>::SharedPtr backend;
};

std::optional<PendingRequest> take_pending_request(
  const std::shared_ptr<ProviderState> & state,
  const std::uint64_t request_id)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->pending || state->pending->id != request_id) {
    return std::nullopt;
  }

  auto pending = std::move(state->pending);
  state->pending.reset();
  return pending;
}

void send_response(
  const std::shared_ptr<rclcpp::Service<PortableSpeak>> & service,
  const std::shared_ptr<rmw_request_id_t> & request_header,
  bool success,
  const std::string & provider,
  const std::string & message)
{
  auto response = std::make_shared<PortableSpeak::Response>();
  response->success = success;
  response->provider = provider;
  response->message = message;

  try {
    service->send_response(*request_header, *response);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      rclcpp::get_logger("hri_naoqi_providers.nao_speak"),
      "Could not send Speak response: %s", error.what());
  }
}
}  // namespace

class NaoSpeakRunner : public capabilities2_runner::RunnerBase
{
public:
  void start(
    rclcpp::Node::SharedPtr node,
    const capabilities2_runner::runner_opts & config,
    std::function<void(const std::string &)> started = nullptr,
    std::function<void(const std::string &)> terminated = nullptr,
    std::function<void(const std::string &)> stopped = nullptr) override
  {
    init_base(node, config, started, terminated, stopped);

    std::string service_name{kDefaultSpeakService};
    try {
      service_name = get_service_name_by_type(kPortableSpeakType);
    } catch (const capabilities2_runner::runner_exception &) {
      RCLCPP_WARN(
        node->get_logger(),
        "No Speak service resource was configured; using %s", kDefaultSpeakService);
    }

    auto state = std::make_shared<ProviderState>();
    state->provider = config.provider;
    state->backend = node->create_client<NaoSay>(kNaoSayService);

    auto callback =
      [state, node](
        std::shared_ptr<rclcpp::Service<PortableSpeak>> service,
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<PortableSpeak::Request> request) {
        std::string provider;
        std::uint64_t request_id{0};
        auto backend_request = std::make_shared<NaoSay::Request>();
        backend_request->text = request->text;
        backend_request->language = request->language.empty() ? kDefaultLanguage : request->language;
        backend_request->animated = false;
        backend_request->asynchronous = false;

        {
          std::lock_guard<std::mutex> lock(state->mutex);
          provider = state->provider;
          if (!state->running) {
            send_response(service, request_header, false, provider, "provider_stopped");
            return;
          }
          if (request->text.empty() || is_blank(request->text)) {
            send_response(service, request_header, false, provider, "text_is_empty");
            return;
          }
          if (state->pending) {
            send_response(service, request_header, false, provider, "provider_busy");
            return;
          }
          if (!state->backend->service_is_ready()) {
            send_response(service, request_header, false, provider, "nao_say_unavailable");
            return;
          }
          request_id = ++state->next_request_id;
          state->pending.emplace(
            PendingRequest{request_id, service, request_header, provider, nullptr});

          std::weak_ptr<ProviderState> weak_state{state};
          state->pending->timeout = node->create_wall_timer(
            kNaoSayTimeout,
            [weak_state, request_id]() {
              const auto locked_state = weak_state.lock();
              if (!locked_state) {
                return;
              }

              auto pending = take_pending_request(locked_state, request_id);
              if (!pending) {
                return;
              }

              pending->timeout->cancel();
              send_response(
                pending->service, pending->request_header, false,
                pending->provider, "nao_say_timeout");
            });

          try {
            state->backend->async_send_request(
              backend_request,
              [state, request_id](
                rclcpp::Client<NaoSay>::SharedFuture future) {
                auto pending = take_pending_request(state, request_id);
                if (!pending) {
                  return;
                }
                pending->timeout->cancel();

                try {
                  const auto backend_response = future.get();
                  send_response(
                    pending->service, pending->request_header,
                    backend_response->success, pending->provider,
                    backend_response->message);
                } catch (const std::exception & error) {
                  send_response(
                    pending->service, pending->request_header, false,
                    pending->provider,
                    std::string("nao_say_failed: ") + error.what());
                }
              });
          } catch (const std::exception & error) {
            auto pending = std::move(state->pending);
            state->pending.reset();
            if (pending) {
              pending->timeout->cancel();
              send_response(
                pending->service, pending->request_header, false,
                pending->provider,
                std::string("nao_say_request_failed: ") + error.what());
            }
          }
        }
      };

    state_ = state;
    service_ = node->create_service<PortableSpeak>(service_name, std::move(callback));

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
      pending = std::move(state->pending);
      state->pending.reset();
    }

    if (pending) {
      pending->timeout->cancel();
      send_response(
        pending->service, pending->request_header, false,
        pending->provider, "provider_stopped");
    }

    service_.reset();
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
  rclcpp::Service<PortableSpeak>::SharedPtr service_;
};
}  // namespace hri_naoqi_providers

PLUGINLIB_EXPORT_CLASS(
  hri_naoqi_providers::NaoSpeakRunner,
  capabilities2_runner::RunnerBase)
