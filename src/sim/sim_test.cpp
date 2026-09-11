#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "sim/Simulator.h"
#include "sim/SimulatorInternal.h"
#include "aero/AeroEngine.h"

static int failures = 0;

static SimCommand cmd(const std::string& type, std::vector<std::pair<std::string, std::string>> params = {}) {
  SimCommand c;
  c.type = type;
  for (const auto& kv : params) c.params[kv.first] = kv.second;
  return c;
}

static void report(const char* label, bool ok, const std::string& detail) {
  std::printf("%s: %s (%s)\n", label, ok ? "PASS" : "FAIL", detail.c_str());
  if (!ok) ++failures;
}

static bool allFinite(const SimResult& r) {
  for (const SimPoint& p : r.positions) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(p.heading) || !std::isfinite(p.pitch) || !std::isfinite(p.roll)) {
      return false;
    }
  }
  return true;
}

int main() {
  const SimResult resA = simulateCommands({cmd("takeoff")});
  {
    bool ok = !resA.positions.empty();
    const double z = ok ? resA.positions.back().z : 0;
    ok = ok && std::fabs(z - 0.8) < 0.08;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "z=%.3f points=%zu", z, resA.positions.size());
    report("(a) takeoff height", ok, buf);
  }

  const SimResult resB = simulateCommands({cmd("takeoff"), cmd("land")});
  {
    bool ok = !resB.positions.empty();
    const double z = ok ? resB.positions.back().z : 1e9;
    const double heading = ok ? resB.positions.back().heading : 1e9;
    ok = ok && std::fabs(z) < 0.1 && std::isfinite(heading) && std::fabs(heading) < 0.5;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "z=%.3f heading=%.3f", z, heading);
    report("(b) takeoff+land", ok, buf);
  }

  const SimResult resC = simulateCommands({cmd("takeoff"), cmd("move_forward", {{"dist", "50"}})});
  {
    bool ok = resC.positions.size() >= 2;
    const SimPoint& first = resC.positions.front();
    const SimPoint& last = resC.positions.back();
    const double travel = std::hypot(last.x - first.x, last.y - first.y);
    ok = ok && travel >= 0.20 && travel <= 0.45 && std::fabs(last.z - 0.8) < 0.08;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "travel=%.3f z=%.3f", travel, last.z);
    report("(c) move_forward 50cm (JS parity ~0.325m)", ok, buf);
  }

  const SimResult resD = simulateCommands({cmd("takeoff"), cmd("turn_degree", {{"degree", "90"}})});
  {
    bool ok = !resD.positions.empty();
    const double heading = ok ? resD.positions.back().heading : 1e9;
    ok = ok && std::fabs(heading - 90.0) < 5.0;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "heading=%.3f", heading);
    report("(d) turn_degree 90", ok, buf);
  }

  const SimResult resE = simulateCommands({cmd("takeoff"), cmd("flip", {{"dir", "back"}})});
  {
    const int takeoffSteps = std::max(static_cast<int>(std::ceil(TAKEOFF_HEIGHT / (DEFAULT_SPEED * DT))), 10);
    const size_t flipBegin = 1 + static_cast<size_t>(takeoffSteps);
    const size_t flipEnd = flipBegin + static_cast<size_t>(FLIP_NUM_POINTS);
    bool ok = resE.positions.size() >= flipEnd;
    bool mono = true;
    double lastPitch = 0;
    double prevPitch = -1e9;
    if (ok) {
      for (size_t i = flipBegin; i < flipEnd; ++i) {
        const double p = resE.positions[i].pitch;
        if (p < prevPitch - 1e-6) mono = false;
        prevPitch = p;
      }
      lastPitch = resE.positions[flipEnd - 1].pitch;
    }
    const SimPoint& first = resE.positions.front();
    const SimPoint& last = resE.positions.back();
    const double horiz = std::hypot(last.x - first.x, last.y - first.y);
    ok = ok && mono && lastPitch > 300.0 && horiz < 0.25 && std::fabs(last.z - 0.8) < 0.08;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "mono=%d endPitch=%.1f horiz=%.3f z=%.3f", mono ? 1 : 0, lastPitch, horiz, last.z);
    report("(e) flip back", ok, buf);
  }

  const SimResult resF = simulateCommands({
      cmd("takeoff"),
      cmd("move_forward", {{"dist", "50"}}),
      cmd("move_forward", {{"dist", "50"}}),
      cmd("move_left", {{"dist", "30"}}),
      cmd("turn_degree", {{"degree", "180"}}),
      cmd("hover", {{"dur", "5"}}),
  });
  {
    bool ok = !resF.positions.empty();
    const SimPoint& last = resF.positions.back();
    ok = ok && last.hasSpeed && std::isfinite(last.speed) && last.energyUsed > 0 &&
         std::isfinite(last.batteryPercent) && last.batteryPercent < 100.0;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "hasSpeed=%d speed=%.3f energy=%.5f battery=%.3f",
                  last.hasSpeed ? 1 : 0, last.speed, last.energyUsed, last.batteryPercent);
    report("(f) battery", ok, buf);
  }

  {
    const bool ok = allFinite(resB) && allFinite(resE);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "b_points=%zu e_points=%zu", resB.positions.size(), resE.positions.size());
    report("(g) no NaN", ok, buf);
  }

  {
    const bool ok = resA.totalDuration > 0 && resB.totalDuration > 0 && resC.totalDuration > 0 &&
                    resD.totalDuration > 0 && resE.totalDuration > 0 && resF.totalDuration > 0;
    char buf[200];
    std::snprintf(buf, sizeof(buf), "a=%.2f b=%.2f c=%.2f d=%.2f e=%.2f f=%.2f",
                  resA.totalDuration, resB.totalDuration, resC.totalDuration,
                  resD.totalDuration, resE.totalDuration, resF.totalDuration);
    report("(h) totalDuration > 0", ok, buf);
  }

  if (failures == 0) {
    std::printf("ALL PASS\n");
    return 0;
  }
  std::printf("FAILED\n");
  return 1;
}
