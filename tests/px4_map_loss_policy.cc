// [NP-01 FIX5 / FIX5b] Regression for the bounded VIBA2 window decision.
// No ROS / OpenCV / Atlas dependency.
//
// Part 1: contract of the pure decision function (configuration order, clock
//         domain, map identity, bound).
// Part 2: STATE SEQUENCES driven through a caller replica of Tracking::
//         KeepViba2Map(), because a boolean-function test alone cannot show
//         that the window is bounded, bound to a map, or cleared on recovery.
#include "../include/MapLossPolicy.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using ORB_SLAM3::DecideViba2Window;
using ORB_SLAM3::Viba2WindowDecision;
namespace D = ORB_SLAM3;

static int checks = 0;
static void require(bool ok, const std::string & what) {
    ++checks;
    if (!ok) {
        std::cerr << "FAIL: " << what << std::endl;
        std::exit(1);
    }
}

static const bool S = true, N = false;
static const Viba2WindowDecision Drop = Viba2WindowDecision::Drop;
static const Viba2WindowDecision Start = Viba2WindowDecision::StartWindow;
static const Viba2WindowDecision Keep = Viba2WindowDecision::Retain;

// ---------------------------------------------------------------------------
// Caller replica: mirrors Tracking::KeepViba2Map() state handling exactly.
// `now` is a MONOTONIC host clock in seconds (steady_clock in production).
// ---------------------------------------------------------------------------
struct Window {
    double windowSeconds = 0.0;
    bool active = false;
    int mapId = -1;
    double start = 0.0;
    int starts = 0, retains = 0, drops = 0;

    Viba2WindowDecision evaluate(bool imuStereo, bool init, bool ba2, int map,
                                 double now, double elapsedOverride = -1.0) {
        const bool sameMap = active && map == mapId;
        double elapsed = sameMap ? (now - start) : -1.0;
        if (elapsedOverride != -1.0 && sameMap) elapsed = elapsedOverride;
        const Viba2WindowDecision d =
            DecideViba2Window(imuStereo, init, ba2, windowSeconds, active, sameMap, elapsed);
        if (d == Start) {
            active = true; mapId = map; start = now; ++starts;
        } else if (d == Keep) {
            ++retains;
        } else {
            if (active) { active = false; mapId = -1; }
            ++drops;
        }
        return d;
    }
};

int main() {
    // =====================================================================
    // Part 1 - pure contract
    // =====================================================================
    // ---- applicability: only the IMU_STEREO / VIBA1-done / VIBA2-pending window
    require(DecideViba2Window(N, S, N, 20.0, N, N, -1.0) == Drop,
            "applicability: non IMU_STEREO keeps native behaviour");
    require(DecideViba2Window(S, N, N, 20.0, N, N, -1.0) == Drop,
            "applicability: IMU not initialised keeps native behaviour");
    require(DecideViba2Window(S, S, S, 20.0, N, N, -1.0) == Drop,
            "applicability: VIBA2 already done needs no window");

    // ---- configuration is evaluated BEFORE any window state -----------------
    // FIX5 bug: protectedSince<0 returned "retain" even when disabled. A
    // disabled policy must not retain for ANY state combination.
    const double disabled[] = {0.0, -1.0, -20.0,
                               std::nan(""), std::numeric_limits<double>::infinity()};
    for (double w : disabled) {
        require(DecideViba2Window(S, S, N, w, N, N, -1.0) == Drop,
                "config: disabled window cannot start a window");
        require(DecideViba2Window(S, S, N, w, S, S, 0.0) == Drop,
                "config: disabled window cannot retain an open window");
        require(DecideViba2Window(S, S, N, w, S, S, 1e-9) == Drop,
                "config: disabled window cannot retain at elapsed>0");
    }

    // ---- map identity -------------------------------------------------------
    require(DecideViba2Window(S, S, N, 20.0, N, S, 0.0) == Start,
            "map: no window yet -> start for this map");
    require(DecideViba2Window(S, S, N, 20.0, S, N, 0.0) == Start,
            "map: a window from another map is never inherited -> start");
    require(DecideViba2Window(S, S, N, 20.0, S, S, 0.0) == Keep,
            "map: same map inside the bound -> retain");

    // ---- clock domain -------------------------------------------------------
    require(DecideViba2Window(S, S, N, 20.0, S, S, -0.5) == Drop,
            "clock: negative elapsed fails closed, never restarts the window");
    require(DecideViba2Window(S, S, N, 20.0, S, S, std::nan("")) == Drop,
            "clock: non-finite elapsed fails closed");

    // ---- bound --------------------------------------------------------------
    require(DecideViba2Window(S, S, N, 20.0, S, S, 19.999) == Keep,
            "bound: just before the bound still retained");
    require(DecideViba2Window(S, S, N, 20.0, S, S, 20.0) == Drop,
            "bound: exactly at the bound native behaviour resumes");
    require(DecideViba2Window(S, S, N, 20.0, S, S, 26.0) == Drop,
            "bound: after the bound native behaviour resumes");

    // =====================================================================
    // Part 2 - state sequences (the part a boolean test cannot prove)
    // =====================================================================

    // ---- S1 transient failure recovers, then fails again --------------------
    {
        Window w; w.windowSeconds = 20.0;
        require(w.evaluate(S, S, N, 1, 0.0) == Start, "S1: first loss starts the window");
        require(w.evaluate(S, S, N, 1, 0.5) == Keep, "S1: second loss retained");
        // tracking recovered (mState OK again); the window stays open but its
        // start time is NOT moved, so the total retention for this map is
        // still bounded by windowSeconds.
        require(w.evaluate(S, S, N, 1, 19.9) == Keep, "S1: later loss still inside the same bound");
        require(w.evaluate(S, S, N, 1, 20.0) == Drop, "S1: bound is measured from the FIRST loss");
        require(!w.active && w.mapId == -1, "S1: drop clears the window state");
        require(w.starts == 1, "S1: repeated transient losses do not restart the bound");
    }

    // ---- S2 continuous failure times out to native reset --------------------
    {
        Window w; w.windowSeconds = 20.0;
        int retains = 0;
        double t = 0.0;
        for (; t < 20.0; t += 0.05) {
            if (w.evaluate(S, S, N, 1, t) == Keep) ++retains;
        }
        require(retains > 300, "S2: continuous failure is retained for the whole bound");
        require(w.evaluate(S, S, N, 1, 20.0) == Drop,
                "S2: after the bound the native reset path resumes");
        require(w.drops >= 1 && !w.active, "S2: window closed after timeout");
    }

    // ---- S3 map switch: the new map never inherits the old window -----------
    {
        Window w; w.windowSeconds = 20.0;
        require(w.evaluate(S, S, N, 1, 0.0) == Start, "S3: map 1 opens a window");
        require(w.evaluate(S, S, N, 1, 10.0) == Keep, "S3: map 1 retained");
        // ResetActiveMap/CreateMapInAtlas produced map 2, also init && !ba2.
        require(w.evaluate(S, S, N, 2, 10.5) == Start,
                "S3: map 2 starts its OWN window instead of inheriting map 1's");
        require(w.mapId == 2 && w.start == 10.5, "S3: window rebound to map 2");
        require(w.evaluate(S, S, N, 2, 30.4) == Keep, "S3: map 2 still inside its own bound");
        require(w.evaluate(S, S, N, 2, 30.5) == Drop, "S3: map 2 bound measured from its own start");
    }

    // ---- S4 VIBA2 completes -> retention must stop immediately -------------
    {
        Window w; w.windowSeconds = 20.0;
        require(w.evaluate(S, S, N, 1, 0.0) == Start, "S4: window open while VIBA2 pending");
        require(w.evaluate(S, S, N, 1, 5.0) == Keep, "S4: retained while pending");
        require(w.evaluate(S, S, S, 1, 5.1) == Drop, "S4: ba2=true drops retention at once");
        require(!w.active, "S4: completed VIBA2 clears the window");
        require(w.evaluate(S, S, S, 1, 5.2) == Drop, "S4: mature map keeps native behaviour");
    }

    // ---- S5 disabled configuration behaves exactly like native ORB-SLAM3 ----
    {
        Window w; w.windowSeconds = 0.0;   // real-machine default
        for (int i = 0; i < 100; ++i)
            require(w.evaluate(S, S, N, 1, i * 0.1) == Drop,
                    "S5: disabled policy never retains a single frame");
        require(w.starts == 0 && w.retains == 0, "S5: no window is ever opened");
    }

    // ---- S6 non-finite / regressing elapsed cannot extend the bound ---------
    // With steady_clock a negative elapsed is unreachable in production; these
    // are defensive inputs. The requirement is that they fail CLOSED (drop the
    // retention) instead of opening or extending a window.
    {
        Window w; w.windowSeconds = 20.0;
        require(w.evaluate(S, S, N, 1, 0.0) == Start, "S6: window opened");
        require(w.evaluate(S, S, N, 1, 5.0, std::nan("")) == Drop,
                "S6: NaN elapsed fails closed instead of restarting");
        require(!w.active, "S6: failed-closed decision clears the window");
        // A cleared window may legitimately be reopened by a later loss; that
        // is a fresh episode, not an extension of the previous bound.
        require(w.evaluate(S, S, N, 1, 6.0) == Start, "S6: later loss opens a fresh window");
        require(w.evaluate(S, S, N, 1, 6.1, -3.0) == Drop,
                "S6: negative elapsed fails closed");
        require(!w.active, "S6: negative elapsed clears the window too");
        // FIX5 silently absorbed a sensor-time regression by moving the window
        // start forward, i.e. the bound was extendable. The caller can no longer
        // express that: elapsed comes from a monotonic clock.
    }

    // ---- S7 recovery must not fabricate validity ----------------------------
    // The policy can only say "keep the map". There is no return value that
    // could mark a pose valid, clear a latch or relax a Gate.
    require(DecideViba2Window(S, S, N, 20.0, S, S, 1.0) == Keep,
            "S7: retention is a map decision only");
    require(DecideViba2Window(S, S, N, 20.0, S, S, 1.0) != Drop,
            "S7: retention never doubles as a reset");

    std::cout << "PX4_MAP_LOSS_POLICY_OK checks=" << checks << std::endl;
    return 0;
}
