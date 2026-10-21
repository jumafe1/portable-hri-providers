// Software-only identity diagnostic. Publishes zeros on one private topic;
// never loads a motion provider or accesses a robot command endpoint.
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
using Twist = geometry_msgs::msg::Twist;
using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
using Clock = std::chrono::steady_clock;

std::string hex(const Gid & gid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : gid) {
    stream << std::setw(2) << static_cast<unsigned>(byte);
  }
  return stream.str();
}

bool valid_gid(const Gid & gid)
{
  return std::any_of(gid.begin(), gid.end(), [](uint8_t byte) {return byte != 0;});
}

bool is_zero(const Twist & message)
{
  return message.linear.x == 0.0 && message.linear.y == 0.0 &&
         message.linear.z == 0.0 && message.angular.x == 0.0 &&
         message.angular.y == 0.0 && message.angular.z == 0.0;
}

bool isolated_environment()
{
  const auto domain = std::getenv("ROS_DOMAIN_ID");
  const auto localhost = std::getenv("ROS_LOCALHOST_ONLY");
  if (!domain || !*domain || !localhost || std::string(localhost) != "1") {
    return false;
  }
  const std::string domain_text(domain);
  if (!std::all_of(domain_text.begin(), domain_text.end(),
      [](char character) {return character >= '0' && character <= '9';}))
  {
    return false;
  }
  // Avoid accepting overflows, partially parsed values, or an implicit domain.
  if (domain_text.size() > 3) {
    return false;
  }
  const auto value = std::strtol(domain, nullptr, 10);
  return value >= 100 && value <= 200;
}
}  // namespace

int main(int argc, char **)
{
  if (argc != 1 || !isolated_environment()) {
    std::cerr << "Refusing diagnostic: set ROS_DOMAIN_ID=100..200 and "
      "ROS_LOCALHOST_ONLY=1; command-line arguments/remappings are not accepted.\n";
    return 2;
  }
  // Deliberately do not ingest command-line remappings, even after the guard.
  rclcpp::init(0, nullptr);
  int result = 1;
  try {
    const auto suffix = std::to_string(getpid()) + "_" +
      std::to_string(Clock::now().time_since_epoch().count());
    const auto topic = "/velocity_gid_diagnostic_" + suffix + "/zero_only";
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.use_intra_process_comms(false);
    auto sender = std::make_shared<rclcpp::Node>("gid_sender_" + suffix, options);
    auto receiver = std::make_shared<rclcpp::Node>("gid_receiver_" + suffix, options);
    const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
    auto publisher = sender->create_publisher<Twist>(topic, qos);
    Gid graph_gid{}, sample_gid{};
    std::size_t samples = 0;
    std::string error;
    auto subscription = receiver->create_subscription<Twist>(
      topic, qos, [&](const Twist::SharedPtr message, const rclcpp::MessageInfo & info) {
        if (!error.empty()) {
          return;
        }
        if (!is_zero(*message)) {
          error = "Unexpected nonzero sample on private diagnostic endpoint";
          return;
        }
        const auto endpoints = receiver->get_publishers_info_by_topic(topic);
        if (endpoints.empty()) {
          return;  // Samples can precede discovery; do not infer a graph identity.
        }
        if (endpoints.size() != 1) {
          error = "Private diagnostic endpoint has multiple publishers";
          return;
        }
        const auto current_graph = endpoints.front().endpoint_gid();
        Gid current_sample{};
        std::copy_n(info.get_rmw_message_info().publisher_gid.data,
          current_sample.size(), current_sample.begin());
        if (!valid_gid(current_graph) || !valid_gid(current_sample)) {
          error = "Graph or received-sample identity is unavailable/all zero";
          return;
        }
        if (samples == 0) {
          graph_gid = current_graph;
          sample_gid = current_sample;
        } else if (graph_gid != current_graph || sample_gid != current_sample) {
          error = "Identity changed within its own representation for one publisher";
          return;
        }
        ++samples;
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sender);
    executor.add_node(receiver);
    const auto deadline = Clock::now() + std::chrono::seconds(10);
    auto next_publication = Clock::now();
    while (rclcpp::ok() && samples < 8 && error.empty() && Clock::now() < deadline) {
      if (Clock::now() >= next_publication) {
        publisher->publish(Twist{});
        next_publication = Clock::now() + std::chrono::milliseconds(100);
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    executor.remove_node(receiver);
    executor.remove_node(sender);
    if (!error.empty()) {
      throw std::runtime_error(error);
    }
    if (samples < 8) {
      throw std::runtime_error("Timed out before eight stable zero samples");
    }
    std::cout << "rmw=" << rmw_get_implementation_identifier() << '\n'
              << "topic=" << topic << '\n'
              << "graph_gid=" << hex(graph_gid) << '\n'
              << "sample_gid=" << hex(sample_gid) << '\n'
              << "cross_representation_equal=" << (graph_gid == sample_gid ? "true" : "false")
              << '\n' << "stable_zero_samples=" << samples << '\n'
              << "passed=true (continuity checked independently; equality is diagnostic only)\n";
    result = 0;
  } catch (const std::exception & error) {
    std::cerr << "passed=false: " << error.what() << '\n';
  }
  rclcpp::shutdown();
  return result;
}
