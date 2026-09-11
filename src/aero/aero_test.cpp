#include <cmath>
#include <iostream>

#include "aero/AeroEngine.h"

static bool noNaN(const Telemetry& t) {
  return !std::isnan(t.pitch) && !std::isnan(t.roll) && !std::isnan(t.yaw) &&
         !std::isnan(t.speed_mps) && !std::isnan(t.altitude_m) &&
         !std::isnan(t.thrust) && !std::isnan(t.drag) &&
         !std::isnan(t.energyUsedWh) && !std::isnan(t.batteryPercent) &&
         !std::isnan(t.turnRadiusM) && !std::isnan(t.vx) &&
         !std::isnan(t.vy) && !std::isnan(t.vz);
}

int main() {
  bool all = true;

  {
    AeroEngine e;
    e.reset(AeroConfig{});
    const double expected = e.cfg.mass * e.cfg.gravity * 2.5;
    const bool pass = e.cfg.mass > 0.0 && e.cfg.gravity > 0.0 &&
                      e.cfg.max_thrust > 0.0 &&
                      std::fabs(e.cfg.max_thrust - expected) < 1e-9;
    std::cout << "(a) reset defaults: " << (pass ? "PASS" : "FAIL") << "\n";
    all = all && pass;
  }

  {
    AeroEngine e;
    e.reset(AeroConfig{});
    AeroControl c;
    c.hasTargetAltitude = true;
    c.targetAltitude = 0.8;
    bool pass = true;
    for (int i = 0; i < 2000; ++i) {
      e.step(c, 0.05);
      if (i >= 1800 && std::fabs(e.telemetry().altitude_m - 0.8) >= 0.02) {
        pass = false;
      }
    }
    std::cout << "(b) hover altitude hold: " << (pass ? "PASS" : "FAIL") << "\n";
    all = all && pass;
  }

  {
    AeroEngine e;
    e.reset(AeroConfig{});
    AeroControl c;
    c.hasTargetAltitude = true;
    c.targetAltitude = 0.8;
    for (int i = 0; i < 1200; ++i) {
      e.step(c, 0.05);
    }
    const Telemetry t = e.telemetry();
    const bool pass = t.energyUsedWh > 0.0 && t.batteryPercent < 100.0;
    std::cout << "(c) battery drains: " << (pass ? "PASS" : "FAIL") << "\n";
    all = all && pass;
  }

  {
    AeroEngine e;
    e.reset(AeroConfig{});
    bool pass = true;
    for (int i = 0; i < 1000; ++i) {
      AeroControl c;
      c.hasTargetAltitude = (i % 2 == 0);
      c.targetAltitude = 1.0;
        c.throttle = 0.4;
      c.pitch = 5.0 * std::sin(0.02 * static_cast<double>(i));
      c.roll = 3.0 + 2.0 * std::sin(0.03 * static_cast<double>(i));
      c.yaw = 10.0 * std::sin(0.01 * static_cast<double>(i));
      e.step(c, 0.05);
      if (!noNaN(e.telemetry())) {
        pass = false;
      }
    }
    std::cout << "(d) no NaN: " << (pass ? "PASS" : "FAIL") << "\n";
    all = all && pass;
  }

  if (all) {
    std::cout << "ALL PASS\n";
    return 0;
  }
  std::cout << "FAILED\n";
  return 1;
}
