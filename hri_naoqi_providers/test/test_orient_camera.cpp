// Software-only: remapped private test endpoints; never commands /nao topics.
#include "motion_lease.hpp"

#include <hri_capability_interfaces/msg/camera_orientation.hpp>
#include <hri_capability_interfaces/srv/orient_camera.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>

using Orient = hri_capability_interfaces::srv::OrientCamera;
using Orientation = hri_capability_interfaces::msg::CameraOrientation;
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class Fixture
{
public:
  explicit Fixture(pluginlib::ClassLoader<capabilities2_runner::RunnerBase> & loader)
  {
    static unsigned sequence = 0;
    prefix = "/camera_probe_" + std::to_string(getpid()) + "_" + std::to_string(++sequence);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "/nao/joint_trajectory:=" + prefix + "/command",
      "-r", "/nao/joint_states:=" + prefix + "/joints"});
    node = std::make_shared<rclcpp::Node>("provider_" + std::to_string(sequence), options);
    backend = std::make_shared<rclcpp::Node>("fake_camera_" + std::to_string(sequence));
    joints = backend->create_publisher<sensor_msgs::msg::JointState>(prefix + "/joints", 1);
    command_sub = backend->create_subscription<Trajectory>(prefix + "/command", 10,
      [this](const Trajectory::SharedPtr message) {
        commands.push_back(*message);
        if (track_target) {elevation = -message->points.front().positions.front();}
      });
    state_sub = backend->create_subscription<Orientation>(prefix + "/state", 10,
      [this](const Orientation::SharedPtr message) {state = *message; states.push_back(*message);});
    client = backend->create_client<Orient>(prefix + "/orient");
    capabilities2_runner::runner_opts config;
    config.interface = "hri_capability_interfaces/OrientCamera";
    config.provider = "hri_naoqi_providers/NaoqiOrientCamera";
    config.resources = {
      {prefix + "/state", "topic", "hri_capability_interfaces/msg/CameraOrientation"},
      {prefix + "/orient", "service", "hri_capability_interfaces/srv/OrientCamera"}};
    runner = loader.createSharedInstance("hri_naoqi_providers::NaoqiOrientCameraRunner");
    runner->start(node, config);
    executor.add_node(node);
    executor.add_node(backend);
    require(wait([this]() {return state.ready && !state.moving;}), "initial ready missing");
  }

  ~Fixture()
  {
    runner->stop();
    executor.remove_node(node);
    executor.remove_node(backend);
    runner.reset();
  }

  sensor_msgs::msg::JointState sample()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = backend->now();
    message.name = {"HeadYaw", "HeadPitch"};
    message.position = {yaw, -elevation};
    return message;
  }

  void tick()
  {
    if (publish_feedback && Clock::now() - last_feedback > std::chrono::milliseconds(20)) {
      joints->publish(sample());
      last_feedback = Clock::now();
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }

  bool wait(const std::function<bool()> & predicate, double seconds = 4.0)
  {
    const auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
    while (Clock::now() < deadline) {
      tick();
      if (predicate()) {return true;}
    }
    return predicate();
  }

  Orient::Response::SharedPtr request(double target, bool hold = false)
  {
    auto request = std::make_shared<Orient::Request>();
    request->elevation = target;
    request->hold = hold;
    auto future = client->async_send_request(request);
    require(wait([&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      }), "service response timeout");
    return future.get();
  }

  void invalid_feedback(sensor_msgs::msg::JointState message)
  {
    publish_feedback = false;
    joints->publish(message);
    require(wait([this]() {return state.status == "invalid_camera_feedback";}),
      "invalid feedback did not latch");
    require(!request(0.0)->accepted, "fault allowed new target");
  }

  std::string prefix;
  rclcpp::Node::SharedPtr node, backend;
  std::shared_ptr<capabilities2_runner::RunnerBase> runner;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints;
  rclcpp::Subscription<Trajectory>::SharedPtr command_sub;
  rclcpp::Subscription<Orientation>::SharedPtr state_sub;
  rclcpp::Client<Orient>::SharedPtr client;
  Orientation state;
  std::vector<Orientation> states;
  std::vector<Trajectory> commands;
  bool publish_feedback{true};
  bool track_target{true};
  double elevation{0.0}, yaw{0.0};
  Clock::time_point last_feedback{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  unsigned checks = 0;
  try {
    pluginlib::ClassLoader<capabilities2_runner::RunnerBase> loader(
      "capabilities2_runner", "capabilities2_runner::RunnerBase");
    {
      Fixture f(loader);
      require(f.commands.empty(), "idle provider moved head"); ++checks;
      hri_naoqi_providers::MotionLease base(f.prefix + "/cmd_vel"); ++checks;
      require(f.request(0.02)->accepted, "valid step rejected");
      require(f.wait([&f]() {return !f.commands.empty() && !f.state.moving;}),
        "target never settled");
      const auto & command = f.commands.front();
      require(command.joint_names == std::vector<std::string>{"HeadPitch"}, "unexpected joint");
      require(command.points.size() == 1 && command.points[0].positions ==
        std::vector<double>{-0.02} && command.points[0].velocities ==
        std::vector<double>{0.05}, "wrong backend sign or speed semantics"); ++checks;
      const auto count = f.commands.size();
      for (double target : {0.30, 0.10, std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()})
      {
        require(!f.request(target)->accepted, "invalid target accepted"); ++checks;
      }
      require(f.commands.size() == count, "rejected target commanded motion"); ++checks;
      require(f.request(0.0, true)->accepted, "hold rejected");
      require(f.wait([&f, count]() {return f.commands.size() > count && !f.state.moving;}),
        "hold did not settle");
      require(f.commands.back().points[0].positions[0] == -0.02, "hold recentered head"); ++checks;
      f.runner->stop();
      require(f.wait([&f, count]() {return f.commands.size() > count + 1;}),
        "release omitted fresh hold");
      require(f.commands.back().points[0].positions[0] == -0.02, "release recentered head"); ++checks;
    }
    {
      Fixture f(loader);
      f.publish_feedback = false;
      require(f.wait([&f]() {return f.state.status == "stale_camera_feedback";}),
        "missing stale watchdog");
      require(f.commands.empty() && !f.request(0.02)->accepted, "stale state commanded head"); ++checks;
    }
    {
      Fixture f(loader);
      f.yaw = 0.1;
      require(f.wait([&f]() {return f.state.status == "camera_pose_out_of_bounds";}),
        "yaw guard missing");
      require(f.commands.empty(), "yaw fault automatically changed head"); ++checks;
    }
    for (unsigned kind = 0; kind < 6; ++kind) {
      Fixture f(loader);
      auto message = f.sample();
      if (kind == 0) {message.position[1] = std::numeric_limits<double>::quiet_NaN();}
      if (kind == 1) {message.name.pop_back();}
      if (kind == 2) {message.header.stamp = f.state.header.stamp;}
      if (kind == 3) {message.header.stamp.sec += 2;}
      if (kind == 4) {message.header.stamp.sec = -1;}
      if (kind == 5) {message.name[0] = "HeadPitch";}
      f.invalid_feedback(message); ++checks;
    }
    {
      Fixture f(loader);
      f.track_target = false;
      require(f.request(0.03)->accepted, "timeout trial step rejected");
      require(f.wait([&f]() {return f.state.status == "camera_target_timeout";}),
        "unreached target did not timeout");
      require(f.wait([&f]() {return f.commands.size() >= 2;}), "timeout omitted hold");
      require(f.commands.back().points[0].positions[0] == 0.0, "timeout hold not measured"); ++checks;
    }
    {
      Fixture f(loader);
      auto intruder = f.backend->create_publisher<Trajectory>(f.prefix + "/command", 1);
      require(f.wait([&f]() {return f.state.status == "camera_publisher_conflict";}),
        "duplicate backend publisher accepted"); ++checks;
    }
    {
      Fixture f(loader);
      auto intruder = f.backend->create_service<Orient>(f.prefix + "/orient",
        [](const Orient::Request::SharedPtr, Orient::Response::SharedPtr) {});
      require(f.wait([&f]() {return f.state.status == "camera_service_conflict";}),
        "duplicate service accepted"); ++checks;
    }
    std::cout << "OrientCamera fake-backend checks passed: " << checks << '\n';
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "OrientCamera check " << checks + 1 << " failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
