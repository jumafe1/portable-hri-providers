#include "motion_lease.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using hri_naoqi_providers::MotionLease;

bool can_acquire(const std::string & topic)
{
  try {
    MotionLease lease(topic);
    return true;
  } catch (const capabilities2_runner::runner_exception &) {
    return false;
  }
}

void require(const bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int child_attempt(const char * executable, const std::string & topic)
{
  const pid_t pid = fork();
  require(pid >= 0, "fork failed");
  if (pid == 0) {
    // exec discards inherited O_CLOEXEC descriptors, testing a separate process
    // rather than retaining a copy of the parent's flock file description.
    execl(executable, executable, "--attempt", topic.c_str(), nullptr);
    _exit(127);
  }
  int status{0};
  require(waitpid(pid, &status, 0) == pid, "waitpid failed");
  require(WIFEXITED(status), "child terminated abnormally");
  return WEXITSTATUS(status);
}

int main(int argc, char ** argv)
{
  if (argc == 3 && std::string(argv[1]) == "--attempt") {
    return can_acquire(argv[2]) ? 0 : 1;
  }
  try {
    const std::string topic = "/lease_test_" + std::to_string(getpid()) + "/cmd_vel";
    {
      MotionLease owner(topic);
      require(!can_acquire(topic), "same-process contention allowed");
      require(child_attempt(argv[0], topic) == 1, "cross-process contention allowed");
      require(can_acquire(topic + "_other"), "different robot incorrectly blocked");
    }
    require(can_acquire(topic), "normal release retained lease");
    require(child_attempt(argv[0], topic) == 0, "normal release blocked another process");
    {
      MotionLease uncertain(topic);
      uncertain.latch_until_process_exit();
    }
    require(!can_acquire(topic), "uncertain lease released in same process");
    require(child_attempt(argv[0], topic) == 1, "uncertain lease released across processes");
    std::cout << "Motion lease: same/cross-process exclusion, normal release, and latch passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
