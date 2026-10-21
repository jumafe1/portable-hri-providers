#include "velocity_source_guard.hpp"

#include <iostream>
#include <stdexcept>

using Guard = hri_naoqi_providers::VelocitySourceGuard;
using Result = Guard::Result;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main()
{
  try {
    const auto now = Guard::Clock::now();
    const auto later = now + std::chrono::milliseconds(500);
    Guard::Gid graph{};
    graph[0] = 1;
    auto replacement = graph;
    replacement[0] = 2;
    rmw_gid_t message{};
    message.implementation_identifier = "test_rmw";
    message.data[0] = 9;  // Legitimate delivery identity deliberately differs.
    Guard no_graph;
    require(no_graph.observe_sample(message, true, now) == Result::invalid,
      "sample without graph identity accepted");
    require(no_graph.observe_graph(graph) == Result::invalid,
      "missing-graph fault not latched");
    Guard guard;
    require(guard.observe_graph(graph) == Result::accepted, "graph rejected");
    require(guard.observe_sample(message, false, now) == Result::awaiting_zero,
      "first nonzero armed movement");
    require(guard.observe_sample(message, true, now) == Result::settling, "zero not learned");
    require(guard.observe_sample(message, false, now) == Result::settling,
      "movement allowed before settling");
    require(guard.observe_graph(graph) == Result::accepted, "stable graph rejected");
    require(guard.observe_sample(message, true, now + std::chrono::milliseconds(400)) ==
      Result::settling, "repeated zero ended settling early");
    require(guard.observe_sample(message, false, later) == Result::accepted,
      "stable identities rejected or repeated zero restarted settling");
    auto changed = message;
    changed.data[0] = 10;
    require(guard.observe_sample(changed, true, later) == Result::changed,
      "foreign zero hid source replacement");
    require(guard.observe_sample(message, true, later) == Result::changed,
      "sample fault not latched");
    Guard settling_change;
    settling_change.observe_graph(graph);
    settling_change.observe_sample(message, true, now);
    require(settling_change.observe_sample(changed, false, now) == Result::changed,
      "changed nonzero or malformed source escaped during settling");
    require(settling_change.observe_sample(message, true, later) == Result::changed,
      "settling source fault not latched");
    Guard changed_graph;
    changed_graph.observe_graph(graph);
    require(changed_graph.observe_graph(replacement) == Result::changed,
      "graph replacement before sample not rejected");
    require(changed_graph.observe_graph(graph) == Result::changed, "graph fault not latched");
    Guard after_sample;
    after_sample.observe_graph(graph);
    after_sample.observe_sample(message, true, now);
    require(after_sample.observe_graph(replacement) == Result::changed,
      "graph replacement after sample not rejected");
    require(after_sample.observe_sample(message, false, later) == Result::changed,
      "old queued sender resumed movement after replacement");
    Guard invalid_graph;
    require(invalid_graph.observe_graph(Guard::Gid{}) == Result::invalid, "empty graph accepted");
    for (int invalid = 0; invalid < 3; ++invalid) {
      Guard invalid_sample;
      invalid_sample.observe_graph(graph);
      auto value = message;
      if (invalid == 0) {std::fill_n(value.data, RMW_GID_STORAGE_SIZE, 0);}
      if (invalid == 1) {value.implementation_identifier = nullptr;}
      if (invalid == 2) {value.implementation_identifier = "";}
      require(invalid_sample.observe_sample(value, true, now) == Result::invalid,
        "unsupported sample identity accepted");
      require(invalid_sample.observe_sample(message, true, later) == Result::invalid,
        "invalid identity fault not latched");
    }
    Guard implementation;
    implementation.observe_graph(graph);
    implementation.observe_sample(message, true, now);
    auto other_rmw = message;
    other_rmw.implementation_identifier = "other_rmw";
    require(implementation.observe_sample(other_rmw, true, later) == Result::changed,
      "changed implementation accepted");
    Guard new_session;
    require(new_session.observe_graph(replacement) == Result::accepted, "new session graph failed");
    require(new_session.observe_sample(changed, true, now) == Result::settling,
      "new session handshake failed");
    require(new_session.observe_sample(changed, false, later) == Result::accepted,
      "new session remains faulted");
    std::cout << "Velocity source continuity and zero-handshake checks passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
