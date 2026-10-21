#pragma once

#include <rmw/types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace hri_naoqi_providers
{
// Graph GIDs and delivery GIDs are separate opaque identity domains. In some
// RMW implementations a delivery GID is a publication handle, not a graph GUID.
// These are continuity guards, NOT authentication or an atomic discovery lock.
class VelocitySourceGuard
{
public:
  using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  using Clock = std::chrono::steady_clock;
  enum class Result {accepted, awaiting_zero, settling, invalid, changed};

  Result observe_graph(const Gid & gid)
  {
    if (fault_) {return *fault_;}
    if (!valid(gid)) {return fail(Result::invalid);}
    if (graph_ && *graph_ != gid) {return fail(Result::changed);}
    graph_ = gid;
    return Result::accepted;
  }

  Result observe_sample(const rmw_gid_t & gid, bool zero, Clock::time_point now)
  {
    if (fault_) {return *fault_;}
    Gid identity{};
    std::copy_n(gid.data, identity.size(), identity.begin());
    if (!graph_ || !valid(identity) || !gid.implementation_identifier ||
      gid.implementation_identifier[0] == '\0')
    {
      return fail(Result::invalid);
    }
    if (sample_ && (*sample_ != identity || implementation_ != gid.implementation_identifier)) {
      return fail(Result::changed);
    }
    if (!sample_) {
      // Never learn an initial sender from a command that could move the robot.
      if (!zero) {return Result::awaiting_zero;}
      sample_ = identity;
      implementation_ = gid.implementation_identifier;
      pinned_at_ = now;
    }
    return now - pinned_at_ < std::chrono::milliseconds(500) ?
           Result::settling : Result::accepted;
  }

private:
  static bool valid(const Gid & gid)
  {
    return std::any_of(gid.begin(), gid.end(), [](uint8_t byte) {return byte != 0;});
  }
  Result fail(Result result) {fault_ = result; return result;}
  std::optional<Gid> graph_, sample_;
  std::optional<Result> fault_;
  std::string implementation_;
  Clock::time_point pinned_at_{};
};
}  // namespace hri_naoqi_providers
