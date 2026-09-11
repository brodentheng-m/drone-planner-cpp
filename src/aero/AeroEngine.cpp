#include "aero/AeroEngine.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double PI = 3.14159265358979323846;
}

AeroEngine::AeroEngine() : cfg{} {
  reset(cfg);
}

void AeroEngine::reset(const AeroConfig& config) {
  cfg = config;
  if (cfg.max_thrust <= 0) {
    cfg.max_thrust = cfg.mass * cfg.gravity * 2.5;
  }
  s = State{};
}

void AeroEngine::step(const AeroControl& control, double dt) {
  const double t = std::min(std::max(control.throttle, 0.0), 1.0);

  s.pitch = control.pitch;
  s.roll = control.roll;

  const double yRateCmd = control.yaw;
  s.yawRate += (yRateCmd - s.yawRate) * std::min(1.0, cfg.rotational_drag * dt);
  s.yaw += s.yawRate * dt;

  const double theta = s.pitch * PI / 180.0;
  const double phi = s.roll * PI / 180.0;
  const double yawr = s.yaw * PI / 180.0;

  double thrustTotal;
  if (control.hasTargetAltitude) {
    const double targetVel = s.hasPrevTargetZ ? (control.targetAltitude - s.prevTargetZ) / dt : 0.0;
    s.prevTargetZ = control.targetAltitude;
    s.hasPrevTargetZ = true;
    const double err = control.targetAltitude - s.z;
    const double errDot = targetVel - s.vz;
    const double requiredUp = cfg.mass * cfg.gravity + cfg.altitude_kp * err + cfg.altitude_kd * errDot;
    const double tilt = std::max(std::cos(theta) * std::cos(phi), 0.25);
    thrustTotal = std::min(std::max(requiredUp / tilt, 0.0), cfg.max_thrust);
  } else {
    thrustTotal = t * cfg.max_thrust;
  }

  const double fwdX = std::cos(yawr);
  const double fwdY = std::sin(yawr);
  const double rightX = -std::sin(yawr);
  const double rightY = std::cos(yawr);

  const double fFwd = -thrustTotal * std::sin(theta);
  const double fRight = thrustTotal * std::sin(phi);
  const double fUp = thrustTotal * std::cos(theta) * std::cos(phi);

  const double fx = fFwd * fwdX + fRight * rightX;
  const double fy = fFwd * fwdY + fRight * rightY;
  const double fz = fUp;

  const double dragX = -cfg.drag_coefficient * s.vx * std::abs(s.vx);
  const double dragY = -cfg.drag_coefficient * s.vy * std::abs(s.vy);
  const double dragZ = -cfg.drag_coefficient * s.vz * std::abs(s.vz);

  const double ax = (fx + dragX) / cfg.mass;
  const double ay = (fy + dragY) / cfg.mass;
  const double az = (fz - cfg.mass * cfg.gravity + dragZ) / cfg.mass;

  s.vx += ax * dt;
  s.vy += ay * dt;
  s.vz += az * dt;

  const double speed = std::hypot(s.vx, s.vy, s.vz);
  if (speed > cfg.max_velocity) {
    const double scale = cfg.max_velocity / speed;
    s.vx *= scale;
    s.vy *= scale;
    s.vz *= scale;
  }

  s.x += s.vx * dt;
  s.y += s.vy * dt;
  s.z += s.vz * dt;

  if (s.z < 0) {
    s.z = 0;
    if (s.vz < 0) {
      s.vz = 0;
    }
  }

  s.thrust = thrustTotal;
  s.drag = std::hypot(dragX, dragY, dragZ);
  s.energyUsedWh += cfg.energy_drain * thrustTotal * dt / cfg.efficiency;
}

Telemetry AeroEngine::telemetry() const {
  Telemetry t;

  t.pitch = s.pitch;
  t.roll = s.roll;
  t.yaw = s.yaw;
  t.speed_mps = std::hypot(s.vx, s.vy, s.vz);
  t.altitude_m = s.z;
  t.thrust = s.thrust;
  t.drag = s.drag;
  t.energyUsedWh = s.energyUsedWh;
  t.batteryPercent = std::max(0.0, 100.0 * (1.0 - s.energyUsedWh / cfg.battery_capacity_wh));
  t.vx = s.vx;
  t.vy = s.vy;
  t.vz = s.vz;

  const double horizSpeed = std::hypot(s.vx, s.vy);
  const double bankRad = s.roll * PI / 180.0;
  t.turnRadiusM = std::numeric_limits<double>::infinity();
  if (horizSpeed > 1e-4) {
    const double tanBank = std::tan(bankRad);
    if (std::abs(tanBank) > 1e-6) {
      t.turnRadiusM = (horizSpeed * horizSpeed) / (cfg.gravity * std::abs(tanBank));
    }
  }

  return t;
}
