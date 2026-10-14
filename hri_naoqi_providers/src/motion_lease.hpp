#pragma once

#include <capabilities2_runner/runner_base.hpp>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdint>
#include <string>

namespace hri_naoqi_providers
{
// Advisory exclusion for our providers on this host and Unix user. This does not
// arbitrate external teleop publishers, other users, hosts, or isolated containers.
class MotionLease
{
public:
  explicit MotionLease(const std::string & resolved_velocity_topic)
  {
    // Stable across processes (unlike implementation-defined std::hash).
    std::uint64_t key = UINT64_C(14695981039346656037);
    for (const unsigned char character : resolved_velocity_topic) {
      key = (key ^ character) * UINT64_C(1099511628211);
    }
    const std::string path = "/tmp/portable_hri_motion_" +
      std::to_string(getuid()) + "_" + std::to_string(key) + ".lock";
    fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    struct stat info {};
    if (fd_ < 0 || fstat(fd_, &info) != 0 || !S_ISREG(info.st_mode) ||
      info.st_uid != getuid() || flock(fd_, LOCK_EX | LOCK_NB) != 0)
    {
      if (fd_ >= 0) {
        close(fd_);
      }
      fd_ = -1;
      throw capabilities2_runner::runner_exception(
              "motion_resource_busy_or_unavailable: " + resolved_velocity_topic);
    }
  }

  MotionLease(const MotionLease &) = delete;
  MotionLease & operator=(const MotionLease &) = delete;

  ~MotionLease()
  {
    if (fd_ >= 0 && !latched_) {
      close(fd_);
    }
    // A latched descriptor intentionally stays open until process exit. Never
    // unlink: replacing the inode would let a new process bypass a held lock.
  }

  void latch_until_process_exit() {latched_ = true;}

private:
  int fd_{-1};
  bool latched_{false};
};
}  // namespace hri_naoqi_providers
