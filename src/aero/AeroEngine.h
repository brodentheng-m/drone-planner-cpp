#ifndef AEROENGINE_H
#define AEROENGINE_H

#include <cmath>

struct AeroConfig {
  double mass = 0.5;
  double gravity = 9.81;
  double max_thrust = 0.0;
  double drag_coefficient = 0.2;
  double rotational_drag = 8.0;
  double max_velocity = 8.0;
  double battery_capacity_wh = 15.0;
  double energy_drain = 0.003;
  double efficiency = 0.85;
  double altitude_kp = 8.0;
  double altitude_kd = 3.0;
};

struct AeroControl {
  double throttle = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  double yaw = 0.0;
  bool hasTargetAltitude = false;
  double targetAltitude = 0.0;
};

struct Telemetry {
  double pitch = 0;
  double roll = 0;
  double yaw = 0;
  double speed_mps = 0;
  double altitude_m = 0;
  double thrust = 0;
  double drag = 0;
  double energyUsedWh = 0;
  double batteryPercent = 100;
  double turnRadiusM = 0;
  double vx = 0;
  double vy = 0;
  double vz = 0;
};

class AeroEngine {
public:
  AeroEngine();
  void reset(const AeroConfig& config);
  void step(const AeroControl& control, double dt);
  Telemetry telemetry() const;
  AeroConfig cfg;
  struct State {
    double x = 0;
    double y = 0;
    double z = 0;
    double vx = 0;
    double vy = 0;
    double vz = 0;
    double pitch = 0;
    double roll = 0;
    double yaw = 0;
    double yawRate = 0;
    double thrust = 0;
    double drag = 0;
    double energyUsedWh = 0;
    double prevTargetZ = 0;
    bool hasPrevTargetZ = false;
  } s;
};

#endif
