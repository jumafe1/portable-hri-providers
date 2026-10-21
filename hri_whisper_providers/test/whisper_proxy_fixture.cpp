// Test-only runner host; no robot interfaces or inference backend are started.
#include <capabilities2_runner/runner_base.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>
#include <csignal>
#include <thread>
volatile std::sig_atomic_t finish_requested = 0;
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) {finish_requested = 1;});
  auto node = std::make_shared<rclcpp::Node>("whisper_proxy_fixture");
  pluginlib::ClassLoader<capabilities2_runner::RunnerBase> loader("capabilities2_runner", "capabilities2_runner::RunnerBase");
  auto runner = loader.createSharedInstance("hri_whisper_providers::WhisperTranscribeSpeechRunner");
  capabilities2_runner::runner_opts config;
  config.interface = "hri_capability_interfaces/TranscribeSpeech";
  config.provider = "hri_whisper_providers/WhisperTranscribeSpeech";
  runner->start(node, config);
  auto control = node->create_subscription<std_msgs::msg::String>("/test/whisper/control", 10,
    [&](std_msgs::msg::String::SharedPtr message) {
      if (message->data == "stop") {runner->stop();}
      if (message->data == "start") {runner->start(node, config);}
    });
  while (!finish_requested) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  runner->stop();
  rclcpp::shutdown();
  return 0;
}
