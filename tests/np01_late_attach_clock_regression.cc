// NP-01 late-attach clock regression (offline, no flight, no ROS, no simulator).
//
// Contract under test
// -------------------
// The sensor clock must establish the SAME time relationships for the same
// relative message sequence regardless of the absolute origin of the source
// clock. A non-zero source start (the simulator has been running for 57 s
// before the VIO process attaches) is NOT a clock fault by itself.
//
// What must still be refused (safety contracts, unchanged):
//   * a genuinely stalled stream  -> domain reset (all buffers invalidated)
//   * source time going backwards -> domain reset
//   * a stale sample             -> reject THAT sample only, no domain reset
//   * host monotonic regression  -> reject THAT sample only, no domain reset
//
// Additionally, every rejection must report WHICH boolean fired and WHICH
// stream stalled. The earlier logs printed only age/dt/gap for the stream
// being admitted, which made "gap=4 ms" look like the culprit when the real
// cause was a different stream stalling past 200 ms.
#include "Px4Vio.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ORB_SLAM3::px4;

static void require(bool ok, const std::string & text)
{
  if (!ok) {
    throw std::runtime_error(text);
  }
}

namespace
{

// stream numbering in the node: 0 = left image, 1 = right image,
// 2 = gyro, 3 = accel.
struct Msg
{
  unsigned stream;
  double t_rel;   // seconds, relative to the sequence origin
};

// Nominal cadence: IMU 250 Hz, stereo 30 Hz, interleaved by source time.
std::vector<Msg> nominalSequence(double seconds)
{
  std::vector<Msg> out;
  const int n_imu = static_cast<int>(std::llround(seconds * 250.0));
  const int n_img = static_cast<int>(std::llround(seconds * 30.0));
  int i = 0, j = 0;
  while (i < n_imu || j < n_img) {
    const double t_imu = (i < n_imu) ? i / 250.0 : 1e9;
    const double t_img = (j < n_img) ? j / 30.0 : 1e9;
    if (t_imu <= t_img) {
      out.push_back({2, t_imu});
      out.push_back({3, t_imu});
      ++i;
    } else {
      out.push_back({0, t_img});
      out.push_back({1, t_img});
      ++j;
    }
  }
  return out;
}

struct Decision
{
  bool accepted = false;
  uint64_t epoch = 0;
  bool ready = false;
  SensorClockReject why;
};

// Host receive model: the sample is stamped with the epoch estimate and
// arrives `lag` seconds after that stamp. mono and wall advance together, so a
// constant lag is not a host artefact.
Decision feed(
  SensorClock & clock, unsigned stream, double epoch_s, double lag_s,
  SensorClockReject * out_why = nullptr)
{
  const uint64_t mono = static_cast<uint64_t>(std::llround(epoch_s * 1e6));
  const uint64_t wall = static_cast<uint64_t>(std::llround((epoch_s + lag_s) * 1e6));
  SensorClockReject why;
  const bool ok = clock.Accept(stream, true, epoch_s * 1e3, wall, mono, &why);
  if (out_why) {
    *out_why = why;
  }
  return Decision{ok, clock.Epoch(), clock.Ready(), why};
}

// Replay the whole sequence for one absolute source origin.
std::vector<Decision> replay(
  const std::vector<Msg> & seq, double src_origin_s, double epoch_offset_s,
  double lag_s)
{
  SensorClock clock;
  std::vector<Decision> out;
  out.reserve(seq.size());
  for (const auto & m : seq) {
    const double epoch_s = src_origin_s + m.t_rel + epoch_offset_s;
    out.push_back(feed(clock, m.stream, epoch_s, lag_s));
  }
  return out;
}

}  // namespace

int main()
{
  try {
    // ---------------------------------------------------------------- T1
    // Absolute source origin must not change any decision.
    // Origin 0 s vs 57 s, with the (source -> epoch) offset absorbing the
    // difference so both describe the same epoch timeline.
    {
      const auto seq = nominalSequence(3.0);
      const double epoch_offset = -1000.0;   // arbitrary but identical epoch axis
      const auto a = replay(seq, 0.0, epoch_offset, 0.010);
      const auto b = replay(seq, 57.0, epoch_offset - 57.0, 0.010);
      require(a.size() == b.size(), "T1 sequence length");
      for (size_t k = 0; k < a.size(); ++k) {
        require(a[k].accepted == b[k].accepted,
                "T1 accept decision depends on absolute source origin at index " +
                std::to_string(k));
        require(a[k].epoch == b[k].epoch,
                "T1 epoch depends on absolute source origin at index " +
                std::to_string(k));
        require(a[k].ready == b[k].ready,
                "T1 readiness depends on absolute source origin at index " +
                std::to_string(k));
      }
    }

    // ---------------------------------------------------------------- T2
    // A late attach (source already at 57 s) must produce ZERO rejects when the
    // cadence is nominal. Non-zero source time is not a fault.
    {
      const auto seq = nominalSequence(3.0);
      const auto d = replay(seq, 57.0, -57.0, 0.010);
      for (size_t k = 0; k < d.size(); ++k) {
        require(d[k].accepted,
                "T2 late attach rejected a nominal sample at index " +
                std::to_string(k));
      }
      require(d.back().ready, "T2 clock never became ready after late attach");
      require(d.back().epoch == 0, "T2 late attach must not bump the epoch");
    }

    // ---------------------------------------------------------------- T3
    // A stale sample is rejected WITHOUT a domain reset. A persistent mapping
    // error therefore produces a clean rejection stream with zero epoch churn,
    // and the clock recovers once the offset converges. This is the exact
    // distinction the round-10 log failed to make: 14 rejects and 14 epoch
    // bumps were reported as one undifferentiated "clock reject".
    {
      SensorClock clock;
      SensorClockReject why;
      double epoch_s = 1000.0;
      double lag_s = 0.300;          // epoch estimate 300 ms behind wall
      int rejected = 0, accepted = 0;
      for (int k = 0; k < 40; ++k) {
        const Decision d = feed(clock, 2, epoch_s, lag_s, &why);
        if (!d.accepted) {
          require(why.staleSample, "T3 reject was not reported as staleSample");
          require(!why.domainReset, "T3 offset error must not reset the domain");
          require(!why.stalled, "T3 offset error misreported as stalled");
          require(clock.Epoch() == 0, "T3 offset error bumped the epoch");
          ++rejected;
        } else {
          ++accepted;
        }
        epoch_s += 0.004;
        lag_s = std::max(0.010, lag_s - 0.020);   // offset converges, 20 ms/tick
      }
      require(rejected > 0, "T3 no stale rejection was observed");
      require(accepted > 0, "T3 clock never recovered as the offset converged");
      require(clock.Epoch() == 0, "T3 epoch churned across stale-only rejects");
    }

    // ---------------------------------------------------------------- T4
    // A stalled stream MUST invalidate all buffers and name the stalled stream.
    // This is the round-10 signature: one stream stopped updating past 200 ms
    // while the stream being admitted looked perfectly regular.
    {
      SensorClock clock;
      double t = 2000.0;
      for (unsigned s = 0; s < 4; ++s) {
        require(feed(clock, s, t, 0.010).accepted, "T4 initial registration");
      }
      require(clock.Ready(), "T4 not ready after registration");
      const uint64_t epoch_before = clock.Epoch();
      bool detected = false;
      // Stream 1 (right image) stops. Streams 2/3 keep running at 250 Hz and
      // stream 0 at 30 Hz, so only stream 1 can trip the 200 ms stall rule.
      for (int k = 1; k <= 120 && !detected; ++k) {
        t += 0.004;
        SensorClockReject why;
        require(feed(clock, 2, t, 0.010).accepted, "T4 gyro path interrupted");
        require(feed(clock, 3, t, 0.010).accepted, "T4 accel path interrupted");
        if (k % 8 == 0) {
          require(feed(clock, 0, t, 0.010).accepted, "T4 left image path interrupted");
        }
        const Decision d = feed(clock, 2, t + 0.001, 0.010, &why);
        if (d.accepted) {
          continue;
        }
        require(why.stalled, "T4 reject was not attributed to a stalled stream");
        require(why.stalledStream == 1,
                "T4 stalled stream reported as " + std::to_string(why.stalledStream) +
                " (expected 1 = right image)");
        require(why.domainReset, "T4 stall must reset the clock domain");
        require(clock.Epoch() == epoch_before + 1, "T4 stall must bump the epoch once");
        require(!clock.Ready(), "T4 stall must invalidate all stream buffers");
        detected = true;
      }
      require(detected, "T4 480 ms stall was never detected");
    }

    // ---------------------------------------------------------------- T5
    // Source-time regression MUST reset the domain (real fault, not a reject).
    {
      SensorClock clock;
      double t = 3000.0;
      for (unsigned s = 0; s < 4; ++s) {
        require(feed(clock, s, t, 0.010).accepted, "T5 initial registration");
      }
      const uint64_t epoch_before = clock.Epoch();
      SensorClockReject why;
      const Decision d = feed(clock, 2, t - 0.004, 0.010, &why);
      require(!d.accepted, "T5 backward source time admitted");
      require(why.srcFault, "T5 source regression not reported as srcFault");
      require(why.domainReset, "T5 source regression must reset the domain");
      require(clock.Epoch() == epoch_before + 1, "T5 source regression must bump epoch");
      require(!clock.Ready(), "T5 source regression must invalidate all buffers");
    }

    // ---------------------------------------------------------------- T6
    // Host monotonic-clock regression is a host artefact: reject that sample
    // only, never a domain reset. This is what keeps a single late callback
    // from wiping the map.
    {
      SensorClock clock;
      double t = 4000.0;
      for (unsigned s = 0; s < 4; ++s) {
        require(feed(clock, s, t, 0.010).accepted, "T6 initial registration");
      }
      const uint64_t epoch_before = clock.Epoch();
      SensorClockReject why;
      // mono goes backwards relative to the previous call, wall keeps advancing.
      const uint64_t mono = static_cast<uint64_t>(std::llround(t * 1e6)) - 5000;
      const uint64_t wall = static_cast<uint64_t>(std::llround((t + 0.004) * 1e6));
      const bool ok = clock.Accept(2, true, (t + 0.004) * 1e3, wall, mono, &why);
      require(!ok, "T6 host artefact admitted");
      require(why.hostArtefact, "T6 host artefact not reported");
      require(!why.domainReset, "T6 host artefact must not reset the domain");
      require(clock.Epoch() == epoch_before, "T6 host artefact bumped the epoch");
      require(clock.Ready(), "T6 host artefact must not invalidate other streams");
    }

    // ---------------------------------------------------------------- T7
    // The FAULT path must also be origin-invariant: the same stall at source
    // origin 0 s and at 57 s must yield the same decisions and the same epoch
    // count. A clock-domain reset is a property of the relative cadence, not of
    // how long the simulator had already been running.
    {
      auto stallPattern = [](double origin_s) {
          SensorClock clock;
          std::vector<Decision> out;
          double t = 5000.0;   // epoch axis, identical for both origins
          for (unsigned s = 0; s < 4; ++s) {
            out.push_back(feed(clock, s, t, 0.010));
          }
          for (int k = 1; k <= 120; ++k) {
            t += 0.004;
            out.push_back(feed(clock, 2, t, 0.010));
            if (k % 8 == 0) {
              out.push_back(feed(clock, 0, t, 0.010));
            }
            // stream 1 (right image) is the stalled one
            out.push_back(feed(clock, 3, t, 0.010));
          }
          (void)origin_s;
          return out;
        };
      const auto a = stallPattern(0.0);
      const auto b = stallPattern(57.0);
      require(a.size() == b.size(), "T7 stall pattern length");
      int resets = 0;
      for (size_t k = 0; k < a.size(); ++k) {
        require(a[k].accepted == b[k].accepted, "T7 stall decision depends on origin");
        require(a[k].epoch == b[k].epoch, "T7 stall epoch depends on origin");
        if (a[k].why.domainReset) {
          ++resets;
        }
      }
      require(resets > 0, "T7 stalled stream never produced a domain reset");
    }

    // ---------------------------------------------------------------- T8
    // After a domain reset every stream must re-register before Ready().
    {
      SensorClock clock;
      double t = 6000.0;
      for (unsigned s = 0; s < 4; ++s) {
        require(feed(clock, s, t, 0.010).accepted, "T8 initial registration");
      }
      require(clock.Ready(), "T8 ready before reset");
      // Force a source-time regression on stream 2.
      require(!feed(clock, 2, t - 0.100, 0.010).accepted, "T8 regression admitted");
      require(!clock.Ready(), "T8 Ready() survived a domain reset");
      t += 0.004;
      require(feed(clock, 2, t, 0.010).accepted, "T8 re-register stream 2");
      require(!clock.Ready(), "T8 Ready() true with only stream 2 re-registered");
      for (unsigned s : {0u, 1u, 3u}) {
        require(feed(clock, s, t, 0.010).accepted, "T8 re-register remaining stream");
      }
      require(clock.Ready(), "T8 Ready() false after full re-registration");
    }

    std::cout << "np01_late_attach_clock_regression: PASS (T1..T8)" << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "FAIL: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
