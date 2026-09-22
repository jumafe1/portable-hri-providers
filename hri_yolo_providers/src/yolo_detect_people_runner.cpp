#include <capabilities2_runner/runner_base.hpp>
#include <hri_capability_interfaces/msg/person_detection2_d.hpp>
#include <hri_capability_interfaces/srv/detect_people.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace hri_yolo_providers
{
namespace
{
using PortableDetection = hri_capability_interfaces::msg::PersonDetection2D;
using PortableDetectPeople = hri_capability_interfaces::srv::DetectPeople;
using YoloDetections = yolo_msgs::msg::DetectionArray;

constexpr char kPortableServiceType[] = "hri_capability_interfaces/srv/DetectPeople";
constexpr char kDefaultPortableService[] = "/hri/detect_people";
constexpr char kYoloDetectionsTopic[] = "/yolo/detections";
constexpr double kDefaultMinimumConfidence = 0.5;
constexpr double kDefaultMaximumAgeSeconds = 1.0;

struct ProviderState
{
  std::mutex mutex;
  bool running{true};
  bool has_frame{false};
  std::string provider;
  YoloDetections latest_frame;
  std::chrono::steady_clock::time_point received_at;
};

void fail_response(
  PortableDetectPeople::Response & response,
  const std::string & provider,
  const std::string & message)
{
  response.success = false;
  response.provider = provider;
  response.message = message;
}
}  // namespace

class YoloDetectPeopleRunner : public capabilities2_runner::RunnerBase
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

    std::string service_name{kDefaultPortableService};
    try {
      service_name = get_service_name_by_type(kPortableServiceType);
    } catch (const capabilities2_runner::runner_exception &) {
      RCLCPP_WARN(
        node->get_logger(),
        "No DetectPeople service resource was configured; using %s",
        kDefaultPortableService);
    }

    auto state = std::make_shared<ProviderState>();
    state->provider = config.provider;

    auto detection_callback = [state](const YoloDetections::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->running) {
          return;
        }
        state->latest_frame = *message;
        state->received_at = std::chrono::steady_clock::now();
        state->has_frame = true;
      };

    auto service_callback =
      [state](
        const std::shared_ptr<PortableDetectPeople::Request> request,
        std::shared_ptr<PortableDetectPeople::Response> response) {
        YoloDetections frame;
        std::chrono::steady_clock::time_point received_at;
        std::string provider;

        {
          std::lock_guard<std::mutex> lock(state->mutex);
          provider = state->provider;
          if (!state->running) {
            fail_response(*response, provider, "provider_stopped");
            return;
          }
          if (!state->has_frame) {
            fail_response(*response, provider, "detections_unavailable");
            return;
          }
          frame = state->latest_frame;
          received_at = state->received_at;
        }

        if (!std::isfinite(request->minimum_confidence) ||
          !std::isfinite(request->maximum_age_sec))
        {
          fail_response(*response, provider, "request_contains_non_finite_value");
          return;
        }

        const double minimum_confidence = request->minimum_confidence <= 0.0 ?
          kDefaultMinimumConfidence : request->minimum_confidence;
        if (minimum_confidence > 1.0) {
          fail_response(*response, provider, "minimum_confidence_out_of_range");
          return;
        }

        const double maximum_age_seconds = request->maximum_age_sec <= 0.0 ?
          kDefaultMaximumAgeSeconds : request->maximum_age_sec;
        const double frame_age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - received_at).count();
        if (frame_age > maximum_age_seconds) {
          fail_response(*response, provider, "detections_stale");
          return;
        }

        for (const auto & detection : frame.detections) {
          if (detection.class_name != "person" || !std::isfinite(detection.score) ||
            detection.score < minimum_confidence ||
            !std::isfinite(detection.bbox.center.position.x) ||
            !std::isfinite(detection.bbox.center.position.y) ||
            !std::isfinite(detection.bbox.size.x) ||
            !std::isfinite(detection.bbox.size.y) ||
            detection.bbox.size.x < 0.0 || detection.bbox.size.y < 0.0)
          {
            continue;
          }

          PortableDetection person;
          person.confidence = detection.score;
          person.tracking_id = detection.id;
          person.center_x = detection.bbox.center.position.x;
          person.center_y = detection.bbox.center.position.y;
          person.width = detection.bbox.size.x;
          person.height = detection.bbox.size.y;
          response->people.push_back(std::move(person));
        }

        response->success = true;
        response->provider = provider;
        response->message = response->people.empty() ? "no_people" : "people_detected";
      };

    state_ = state;
    detection_subscription_ = node->create_subscription<YoloDetections>(
      kYoloDetectionsTopic, rclcpp::QoS(10), std::move(detection_callback));
    service_ = node->create_service<PortableDetectPeople>(
      service_name, std::move(service_callback));

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
      state->has_frame = false;
    }

    service_.reset();
    detection_subscription_.reset();
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
  rclcpp::Subscription<YoloDetections>::SharedPtr detection_subscription_;
  rclcpp::Service<PortableDetectPeople>::SharedPtr service_;
};
}  // namespace hri_yolo_providers

PLUGINLIB_EXPORT_CLASS(
  hri_yolo_providers::YoloDetectPeopleRunner,
  capabilities2_runner::RunnerBase)
