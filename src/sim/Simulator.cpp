#include "sim/SimulatorInternal.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "sim/Eval.h"

namespace {

constexpr double PI = 3.14159265358979323846;

double clampd(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

int clampi(int v, int lo, int hi) {
  return std::min(std::max(v, lo), hi);
}

std::string numToStr(double v) {
  std::string s = std::to_string(v);
  const size_t dot = s.find('.');
  if (dot != std::string::npos) {
    s.erase(s.find_last_not_of('0') + 1);
    if (s.back() == '.') s.pop_back();
  }
  return s;
}

double varNum(const std::map<std::string, std::string>& vars, const std::string& name) {
  const auto it = vars.find(name);
  if (it == vars.end()) return 0.0;
  try {
    return std::stod(it->second);
  } catch (...) {
    return 0.0;
  }
}

std::string trim(const std::string& s) {
  const size_t a = s.find_first_not_of(" \t\n\r\f\v");
  if (a == std::string::npos) return "";
  const size_t b = s.find_last_not_of(" \t\n\r\f\v");
  return s.substr(a, b - a + 1);
}

double nowSec() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::map<std::string, std::vector<SimCommand>>& funcRegistry() {
  static std::map<std::string, std::vector<SimCommand>> registry;
  return registry;
}

SimResult simulateSingleDrone(const std::vector<SimCommand>& commands, double ox, double oy, double oz) {
  funcRegistry().clear();
  SimState state;
  SimPoint start;
  start.x = ox;
  start.y = oz;
  start.z = oy;
  start.hasSpeed = true;
  start.speed = 0;
  start.energyUsed = 0;
  start.batteryPercent = 100;
  start.turnRadiusM = 1e30;
  state.positions.push_back(start);
  std::map<std::string, std::string> vars;
  processCommands(commands, vars, state, 500);
  SimResult result;
  result.positions = std::move(state.positions);
  result.totalDuration = state.totalDuration;
  result.collisions = std::move(state.collisions);
  for (SimPoint& pt : result.positions) {
    pt.x += ox;
    pt.y += oz;
    pt.z += oy;
  }
  return result;
}

}

double pnum(const std::map<std::string, std::string>& p, const std::string& k, double dflt) {
  const auto it = p.find(k);
  if (it == p.end()) return dflt;
  try {
    size_t idx = 0;
    const double v = std::stod(it->second, &idx);
    if (idx == 0 || std::isnan(v)) return dflt;
    return v;
  } catch (...) {
    return dflt;
  }
}

std::string pstr(const std::map<std::string, std::string>& p, const std::string& k, const std::string& dflt) {
  const auto it = p.find(k);
  return it == p.end() ? dflt : it->second;
}

SimControl computeControl(const AeroEngine& engine, double tx, double ty, double tz, double th, double dt) {
  const double g = engine.cfg.gravity;
  const double vx = engine.s.vx;
  const double vy = engine.s.vy;
  const double px = engine.s.x;
  const double py = engine.s.y;

  const double horizPosGain = 2.5;
  const double horizVelGain = 4;

  const double errX = tx - px;
  const double errY = ty - py;

  double desVX = horizPosGain * errX;
  double desVY = horizPosGain * errY;
  const double desSpeed = std::hypot(desVX, desVY);
  if (desSpeed > engine.cfg.max_velocity) {
    const double scale = engine.cfg.max_velocity / desSpeed;
    desVX *= scale;
    desVY *= scale;
  }

  const double ax = (desVX - vx) * horizVelGain;
  const double ay = (desVY - vy) * horizVelGain;

  const double yawRad = th * PI / 180;
  const double fwdX = std::cos(yawRad);
  const double fwdY = std::sin(yawRad);
  const double rightX = -std::sin(yawRad);
  const double rightY = std::cos(yawRad);

  const double aFwd = ax * fwdX + ay * fwdY;
  const double aRight = ax * rightX + ay * rightY;

  const double pitchDeg = clampd(-std::atan2(aFwd, g) * 180 / PI, -MAX_PITCH, MAX_PITCH);
  const double rollDeg = clampd(std::atan2(aRight, g) * 180 / PI, -MAX_ROLL, MAX_ROLL);

  const double yawRate = clampd((th - engine.s.yaw) / dt, -MAX_YAW_RATE, MAX_YAW_RATE);

  SimControl control;
  control.throttle = 0;
  control.pitch = pitchDeg;
  control.roll = rollDeg;
  control.yaw = yawRate;
  control.hasTargetAltitude = true;
  control.targetAltitude = tz;
  return control;
}

void driveStep(SimState& state, AeroEngine& engine, double tx, double ty, double tz, double th, double dt) {
  const SimControl control = computeControl(engine, tx, ty, tz, th, dt);
  AeroControl ac;
  ac.throttle = control.throttle;
  ac.pitch = control.pitch;
  ac.roll = control.roll;
  ac.yaw = control.yaw;
  ac.hasTargetAltitude = control.hasTargetAltitude;
  ac.targetAltitude = control.targetAltitude;
  engine.step(ac, dt);
  state.x = engine.s.x;
  state.y = engine.s.y;
  state.z = engine.s.z;
  state.heading = th;
}

void pushPoint(SimState& state, AeroEngine& engine, double heading, double pitch, double roll, bool hasLed, int r, int g, int b, int brightness) {
  SimPoint pt;
  pt.x = engine.s.x;
  pt.y = engine.s.y;
  pt.z = engine.s.z;
  pt.heading = heading;
  pt.pitch = pitch;
  pt.roll = roll;
  if (hasLed) {
    pt.hasLed = true;
    pt.ledR = r;
    pt.ledG = g;
    pt.ledB = b;
    pt.ledBrightness = brightness;
  }
  pt.hasSpeed = true;
  const Telemetry tel = engine.telemetry();
  pt.speed = tel.speed_mps;
  pt.energyUsed = tel.energyUsedWh;
  pt.batteryPercent = tel.batteryPercent;
  pt.turnRadiusM = std::isinf(tel.turnRadiusM) ? 1e30 : tel.turnRadiusM;
  state.positions.push_back(pt);
}

void processCommands(const std::vector<SimCommand>& commands, std::map<std::string, std::string>& vars, SimState& state, int maxIter) {
  int iter = 0;
  for (const SimCommand& cmd : commands) {
    if (iter++ > maxIter) break;
    const std::map<std::string, std::string>& p = cmd.params;
    const std::string& type = cmd.type;
    double dur = 0;
    AeroEngine& engine = state.engine;

    if (type == "takeoff") {
      const int steps = std::max(static_cast<int>(std::ceil(TAKEOFF_HEIGHT / (DEFAULT_SPEED * DT))), 10);
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        const double tz = TAKEOFF_HEIGHT * t;
        driveStep(state, engine, state.x, state.y, tz, state.heading, DT);
        const double pitch = t < 0.5 ? -MAX_PITCH * (1 - t * 2) : 0;
        pushPoint(state, engine, state.heading, pitch, 0, false, 0, 0, 0, 100);
      }
      state.flying = true;
      dur = steps * DT;
    } else if (type == "land") {
      const double startZ = state.z;
      const int steps = std::max(static_cast<int>(std::ceil(startZ / (DEFAULT_SPEED * DT))), 10);
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        const double tz = startZ * (1 - t);
        driveStep(state, engine, state.x, state.y, tz, state.heading, DT);
        const double pitch = t > 0.5 ? MAX_PITCH * ((t - 0.5) * 2) : 0;
        pushPoint(state, engine, state.heading, pitch, 0, false, 0, 0, 0, 100);
      }
      state.flying = false;
      state.z = 0;
      engine.s.z = 0;
      dur = steps * DT;
    } else if (type == "hover") {
      dur = pnum(p, "dur", 1);
      const int steps = static_cast<int>(std::ceil(dur / DT));
      for (int i = 0; i < steps; ++i) {
        driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
      }
    } else if (type == "flip") {
      const std::string dir = pstr(p, "dir", "back");
      const double hr = state.heading * PI / 180;
      const double hx = state.x, hy = state.y, hz = state.z;
      const double cosH = std::cos(hr);
      const double sinH = std::sin(hr);
      double dirX = 0, dirY = 0;
      if (dir == "forward") { dirX = cosH; dirY = sinH; }
      else if (dir == "back") { dirX = -cosH; dirY = -sinH; }
      else if (dir == "left") { dirX = sinH; dirY = -cosH; }
      else { dirX = -sinH; dirY = cosH; }
      const double Rh = FLIP_TRAVEL;
      const double Rv = FLIP_CLIMB;
      const double phi0 = -PI / 2;
      double prevTangent = 0;
      double tangent = 0;
      bool hasPrev = false;
      for (int i = 0; i < FLIP_NUM_POINTS; ++i) {
        const double s = static_cast<double>(i) / (FLIP_NUM_POINTS - 1);
        const double u = (1 - std::cos(PI * s)) / 2;
        const double phi = phi0 + 2 * PI * u;
        const double tx = hx + dirX * Rh * std::cos(phi);
        const double ty = hy + dirY * Rh * std::cos(phi);
        const double tz = hz + Rv + Rv * std::sin(phi);
        const double raw = std::atan2(Rv * std::cos(phi), -Rh * std::sin(phi));
        if (!hasPrev) {
          tangent = raw;
        } else {
          double d = raw - prevTangent;
          while (d > PI) d -= 2 * PI;
          while (d < -PI) d += 2 * PI;
          tangent += d;
        }
        prevTangent = raw;
        hasPrev = true;
        const double theta = tangent * 180 / PI;
        double pitch = 0, roll = 0;
        if (dir == "back") pitch = theta;
        else if (dir == "forward") pitch = -theta;
        else if (dir == "left") roll = theta;
        else roll = -theta;
        driveStep(state, engine, tx, ty, tz, state.heading, DT);
        pushPoint(state, engine, state.heading, pitch, roll, false, 0, 0, 0, 100);
      }
      state.x = hx;
      state.y = hy;
      state.z = hz;
      for (int i = 0; i < FLIP_SETTLE; ++i) {
        driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
      }
      dur = FLIP_NUM_POINTS * DT;
    } else if (type == "go") {
      const std::string dir = pstr(p, "dir", "forward");
      const double power = pnum(p, "power", 50);
      dur = pnum(p, "dur", 1);
      const double speed = (power / 100) * DEFAULT_SPEED * 2;
      const int steps = std::max(static_cast<int>(std::ceil(dur / DT)), 5);
      const double hr = state.heading * PI / 180;
      double dx = 0, dy = 0, dz = 0, pitch = 0, roll = 0;
      const double pf = power / 100;
      if (dir == "forward") { dx = std::cos(hr); dy = std::sin(hr); pitch = -MAX_PITCH * pf; }
      else if (dir == "backward") { dx = -std::cos(hr); dy = -std::sin(hr); pitch = MAX_PITCH * pf; }
      else if (dir == "left") { dx = std::sin(hr); dy = -std::cos(hr); roll = -MAX_ROLL * pf; }
      else if (dir == "right") { dx = -std::sin(hr); dy = std::cos(hr); roll = MAX_ROLL * pf; }
      else if (dir == "up") { dz = speed * dur; }
      else if (dir == "down") { dz = -speed * dur; }
      const double startX = state.x, startY = state.y, startZ = state.z;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        double ease = 1;
        if (t < 0.2) ease = t / 0.2;
        else if (t > 0.8) ease = (1 - t) / 0.2;
        const double newX = startX + dx * speed * dur * t;
        const double newY = startY + dy * speed * dur * t;
        const double newZ = startZ + dz * t;
        driveStep(state, engine, newX, newY, newZ, state.heading, DT);
        pushPoint(state, engine, state.heading, pitch * ease, roll * ease, false, 0, 0, 0, 100);
      }
      state.x = startX + dx * speed * dur;
      state.y = startY + dy * speed * dur;
      state.z = startZ + dz;
      engine.s.x = state.x;
      engine.s.y = state.y;
      engine.s.z = state.z;
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "move_forward" || type == "move_backward") {
      const double dist = pnum(p, "dist", 50) / 100;
      const double speed = pnum(p, "speed", 50) / 100;
      const double sign = type == "move_forward" ? 1 : -1;
      const int steps = std::max(static_cast<int>(std::ceil(dist / (speed * DEFAULT_SPEED * 2 * DT))), 5);
      const double hr = state.heading * PI / 180;
      const double pitch = sign * MAX_PITCH * speed;
      const double sx = state.x, sy = state.y, sz = state.z;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        double ease = 1;
        if (t < 0.2) ease = t / 0.2;
        else if (t > 0.8) ease = (1 - t) / 0.2;
        const double tx = sx + sign * std::cos(hr) * dist * t;
        const double ty = sy + sign * std::sin(hr) * dist * t;
        driveStep(state, engine, tx, ty, sz, state.heading, DT);
        pushPoint(state, engine, state.heading, pitch * ease, 0, false, 0, 0, 0, 100);
      }
      state.x = sx + sign * std::cos(hr) * dist;
      state.y = sy + sign * std::sin(hr) * dist;
      state.z = sz;
      dur = steps * DT;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "move_left" || type == "move_right") {
      const double dist = pnum(p, "dist", 50) / 100;
      const double speed = pnum(p, "speed", 50) / 100;
      const int steps = std::max(static_cast<int>(std::ceil(dist / (speed * DEFAULT_SPEED * 2 * DT))), 5);
      const double hr = state.heading * PI / 180;
      const double roll = type == "move_left" ? -MAX_ROLL * speed : MAX_ROLL * speed;
      const double sx = state.x, sy = state.y, sz = state.z;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        const double ease = t < 0.2 ? t / 0.2 : t > 0.8 ? (1 - t) / 0.2 : 1;
        double tx, ty;
        if (type == "move_left") { tx = sx + std::sin(hr) * dist * t; ty = sy - std::cos(hr) * dist * t; }
        else { tx = sx - std::sin(hr) * dist * t; ty = sy + std::cos(hr) * dist * t; }
        driveStep(state, engine, tx, ty, sz, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, roll * ease, false, 0, 0, 0, 100);
      }
      if (type == "move_left") { state.x = sx + std::sin(hr) * dist; state.y = sy - std::cos(hr) * dist; }
      else { state.x = sx - std::sin(hr) * dist; state.y = sy + std::cos(hr) * dist; }
      state.z = sz;
      dur = steps * DT;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "turn_left" || type == "turn_right") {
      const double deg = pnum(p, "deg", 90);
      const int steps = std::max(static_cast<int>(std::ceil(deg / 180 * 10)), 5);
      const double degFactor = std::min(deg / 360, 1.0);
      const double rollDir = type == "turn_left" ? -MAX_ROLL * degFactor : MAX_ROLL * degFactor;
      const double startHeading = state.heading;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        const double ease = t < 0.2 ? t / 0.2 : t > 0.8 ? (1 - t) / 0.2 : 1;
        state.heading = startHeading + (type == "turn_left" ? -deg : deg) * t;
        driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, rollDir * ease, false, 0, 0, 0, 100);
      }
      dur = deg / 180;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "turn_degree") {
      const double deg = pnum(p, "deg", 90);
      const double timeout = pnum(p, "timeout", 3);
      const int steps = std::max(static_cast<int>(std::ceil(timeout * 10)), 5);
      const double degFactor = std::min(std::abs(deg) / 360, 1.0);
      const double rollDir = deg > 0 ? MAX_ROLL * degFactor : -MAX_ROLL * degFactor;
      const double startHeading = state.heading;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        const double ease = t < 0.2 ? t / 0.2 : t > 0.8 ? (1 - t) / 0.2 : 1;
        state.heading = startHeading + deg * t;
        driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, rollDir * ease, false, 0, 0, 0, 100);
      }
      dur = timeout;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "circle" || type == "circle_turn") {
      const double speed = pnum(p, "speed", 75) / 100;
      const double direction = pstr(p, "dir", "") == "counter-clockwise" ? -1 : 1;
      const double radius = 0.5;
      const int steps = 60;
      const double cx = state.x, cy = state.y, cz = state.z;
      for (int i = 0; i < steps; ++i) {
        const double angle = 2 * PI * i / steps * direction;
        const double tx = cx + radius * std::cos(angle);
        const double ty = cy + radius * std::sin(angle);
        driveStep(state, engine, tx, ty, cz, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, -MAX_ROLL * speed * std::sin(angle * direction), false, 0, 0, 0, 100);
      }
      state.x = cx + radius * direction;
      state.y = cy;
      state.z = cz;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
      dur = 3 * speed;
    } else if (type == "square" || type == "triangle" || type == "square_turn" || type == "triangle_turn") {
      const double speed = pnum(p, "speed", 60) / 100;
      const double secs = pnum(p, "secs", 1);
      const double direction = pstr(p, "dir", "") == "counter-clockwise" ? -1 : 1;
      const bool isTriangle = type.find("triangle") != std::string::npos;
      const int numSides = isTriangle ? 3 : 4;
      const double side = 0.5;
      const int stepsPerSide = 15;
      const double hr = state.heading * PI / 180;
      const double sx = state.x, sy = state.y, sz = state.z;
      std::vector<double> sideAngles;
      for (int i = 0; i < numSides; ++i) {
        sideAngles.push_back(hr + (i * 2 * PI / numSides) * direction);
      }
      double accX = 0, accY = 0;
      for (int si = 0; si < numSides; ++si) {
        const double angle = sideAngles[si];
        const double ddx = std::cos(angle), ddy = std::sin(angle);
        for (int i = 0; i < stepsPerSide; ++i) {
          const double t = static_cast<double>(i + 1) / stepsPerSide;
          const double ease = t < 0.2 ? t / 0.2 : t > 0.8 ? (1 - t) / 0.2 : 1;
          const double tx = sx + accX + ddx * side * t;
          const double ty = sy + accY + ddy * side * t;
          driveStep(state, engine, tx, ty, sz, state.heading, DT);
          pushPoint(state, engine, state.heading, -MAX_PITCH * speed * ease, 0, false, 0, 0, 0, 100);
        }
        accX += ddx * side;
        accY += ddy * side;
      }
      state.x = sx + accX;
      state.y = sy + accY;
      state.z = sz;
      dur = numSides * secs * speed;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "spiral") {
      const double speed = pnum(p, "speed", 50) / 100;
      const double direction = pstr(p, "dir", "") == "counter-clockwise" ? -1 : 1;
      const int steps = 120;
      const double sx = state.x, sy = state.y, sz = state.z;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        const double angle = 4 * PI * t * direction;
        const double radius = t * 0.5;
        const double tx = sx + radius * std::cos(angle);
        const double ty = sy + radius * std::sin(angle);
        const double tz = sz + t * 0.3;
        driveStep(state, engine, tx, ty, tz, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, -MAX_ROLL * speed * std::sin(angle), false, 0, 0, 0, 100);
      }
      state.x = sx + 0.5 * std::cos(4 * PI * direction);
      state.y = sy + 0.5 * std::sin(4 * PI * direction);
      state.z = sz + 0.3;
      dur = 5 * speed;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "sway") {
      const double speed = pnum(p, "speed", 50) / 100;
      const std::string dir = pstr(p, "dir", "forward-back");
      const int steps = 40;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        const double angle = 2 * PI * t;
        double pitch = 0, roll = 0;
        if (dir == "forward-back") pitch = MAX_PITCH * speed * std::sin(angle);
        else if (dir == "left-right") roll = MAX_ROLL * speed * std::sin(angle);
        else if (dir == "pitch-forward") pitch = MAX_PITCH * speed;
        else if (dir == "pitch-backward") pitch = -MAX_PITCH * speed;
        else if (dir == "roll-left") roll = -MAX_ROLL * speed;
        else if (dir == "roll-right") roll = MAX_ROLL * speed;
        driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
        pushPoint(state, engine, state.heading, pitch, roll, false, 0, 0, 0, 100);
      }
      dur = 2 * speed;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "keep_distance" || type == "avoid_wall") {
      const double dist = pnum(p, "dist", 50) / 100;
      const double speed = pnum(p, "speed", 50) / 100;
      const int steps = std::max(static_cast<int>(std::ceil(dist / (speed * DEFAULT_SPEED * DT))), 5);
      const double hr = state.heading * PI / 180;
      const double sx = state.x, sy = state.y, sz = state.z;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i + 1) / steps;
        const double tx = sx - std::cos(hr) * dist * t;
        const double ty = sy - std::sin(hr) * dist * t;
        driveStep(state, engine, tx, ty, sz, state.heading, DT);
        pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
      }
      state.x = sx - std::cos(hr) * dist;
      state.y = sy - std::sin(hr) * dist;
      state.z = sz;
      dur = steps * DT;
      driveStep(state, engine, state.x, state.y, state.z, state.heading, DT);
      pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
    } else if (type == "detect_wall") {
      vars[pstr(p, "var", "")] = "0";
      dur = 0.1;
    } else if (type == "led") {
      const int r = clampi(static_cast<int>(pnum(p, "r", 0)), 0, 255);
      const int g = clampi(static_cast<int>(pnum(p, "g", 255)), 0, 255);
      const int b = clampi(static_cast<int>(pnum(p, "b", 0)), 0, 255);
      const int brightness = clampi(static_cast<int>(pnum(p, "brightness", 100)), 0, 255);
      state.ledOn = true;
      state.ledR = r;
      state.ledG = g;
      state.ledB = b;
      state.ledBrightness = brightness;
      pushPoint(state, engine, state.heading, 0, 0, true, r, g, b, brightness);
      dur = 0.1;
    } else if (type == "led_off") {
      state.ledOn = false;
      state.ledR = 0;
      state.ledG = 0;
      state.ledB = 0;
      state.ledBrightness = 0;
      pushPoint(state, engine, state.heading, 0, 0, true, 0, 0, 0, 0);
      dur = 0.1;
    } else if (type == "buzzer") {
      dur = pnum(p, "dur", 500) / 1000;
    } else if (type == "var_declare") {
      vars[pstr(p, "name", "")] = numToStr(eval::evaluate(pstr(p, "value", ""), vars));
      dur = 0;
    } else if (type == "set_var") {
      const double val = eval::evaluate(pstr(p, "value", ""), vars);
      const std::string name = pstr(p, "name", "");
      const std::string op = pstr(p, "op", "=");
      if (op == "=") vars[name] = numToStr(val);
      else if (op == "+=") vars[name] = numToStr(varNum(vars, name) + val);
      else if (op == "-=") vars[name] = numToStr(varNum(vars, name) - val);
      else if (op == "*=") vars[name] = numToStr(varNum(vars, name) * val);
      else if (op == "/=") vars[name] = numToStr(val != 0 ? varNum(vars, name) / val : 0);
      dur = 0;
    } else if (type == "print_var") {
      dur = 0;
    } else if (type == "if_block") {
      const double cond = eval::evaluate(pstr(p, "condition", ""), vars);
      if (cond) processCommands(cmd.children, vars, state, maxIter);
      dur = 0;
    } else if (type == "elif_block") {
      const double cond = eval::evaluate(pstr(p, "condition", ""), vars);
      if (cond) processCommands(cmd.children, vars, state, maxIter);
      dur = 0;
    } else if (type == "else_block") {
      processCommands(cmd.children, vars, state, maxIter);
      dur = 0;
    } else if (type == "end_block") {
      dur = 0;
    } else if (type == "while_block") {
      int loops = 0;
      while (eval::evaluate(pstr(p, "condition", ""), vars) && loops < 500) {
        processCommands(cmd.children, vars, state, maxIter);
        ++loops;
      }
      dur = 0;
    } else if (type == "for_block") {
      const std::string varName = pstr(p, "var", "i");
      const int start = static_cast<int>(pnum(p, "start", 0));
      const int endVal = static_cast<int>(pnum(p, "end_val", 5));
      const int step = static_cast<int>(pnum(p, "step", 1));
      if (step != 0) {
        for (int i = start; step > 0 ? i < endVal : i > endVal; i += step) {
          vars[varName] = numToStr(i);
          processCommands(cmd.children, vars, state, maxIter);
        }
      }
      dur = 0;
    } else if (type == "break_cmd") {
      dur = 0;
    } else if (type == "emergency_stop" || type == "stop_motors") {
      const int steps = 10;
      for (int i = 0; i < steps; ++i) {
        pushPoint(state, engine, state.heading, 0, 0, false, 0, 0, 0, 100);
      }
      state.flying = false;
      dur = 0.5;
    } else if (type == "get_battery") {
      vars[pstr(p, "var", "")] = "80";
      dur = 0;
    } else if (type == "get_height") {
      vars[pstr(p, "var", "")] = numToStr(state.z * 100);
      dur = 0;
    } else if (type == "get_front_range") {
      vars[pstr(p, "var", "")] = "100";
      dur = 0;
    } else if (type == "get_bottom_range") {
      vars[pstr(p, "var", "")] = numToStr(state.z * 100);
      dur = 0;
    } else if (type == "get_front_color") {
      vars[pstr(p, "var", "")] = "green";
      dur = 0;
    } else if (type == "get_back_color") {
      vars[pstr(p, "var", "")] = "blue";
      dur = 0;
    } else if (type == "get_temperature") {
      vars[pstr(p, "var", "")] = "22";
      dur = 0;
    } else if (type == "func_def") {
      funcRegistry()[pstr(p, "name", "")] = cmd.children;
      dur = 0;
    } else if (type == "func_call") {
      const auto it = funcRegistry().find(pstr(p, "name", ""));
      if (it != funcRegistry().end()) processCommands(it->second, vars, state, maxIter);
      dur = 0;
    } else if (type == "return_val") {
      dur = 0;
    } else if (type == "list_declare") {
      const std::string values = pstr(p, "values", "");
      std::string out;
      size_t pos = 0;
      bool first = true;
      while (true) {
        const size_t comma = values.find(',', pos);
        const std::string item = trim(values.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos));
        if (!first) out += ",";
        out += numToStr(eval::evaluate(item, vars));
        first = false;
        if (comma == std::string::npos) break;
        pos = comma + 1;
      }
      vars[pstr(p, "name", "")] = out;
      dur = 0;
    } else if (type == "list_append") {
      const std::string name = pstr(p, "name", "");
      const double val = eval::evaluate(pstr(p, "value", ""), vars);
      const auto it = vars.find(name);
      if (it == vars.end()) vars[name] = numToStr(val);
      else it->second += "," + numToStr(val);
      dur = 0;
    } else if (type == "list_get") {
      const auto it = vars.find(pstr(p, "list_name", ""));
      const int index = static_cast<int>(pnum(p, "index", 0));
      std::string val = "0";
      if (it != vars.end()) {
        std::vector<std::string> items;
        size_t pos = 0;
        while (true) {
          const size_t comma = it->second.find(',', pos);
          items.push_back(it->second.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos));
          if (comma == std::string::npos) break;
          pos = comma + 1;
        }
        if (index >= 0 && index < static_cast<int>(items.size())) val = items[index];
      }
      vars[pstr(p, "var", "")] = val;
      dur = 0;
    } else if (type == "user_input") {
      vars[pstr(p, "var", "")] = "0";
      dur = 0;
    } else if (type == "timer_start") {
      vars[pstr(p, "name", "")] = numToStr(nowSec());
      dur = 0;
    } else if (type == "timer_elapsed") {
      vars[pstr(p, "var", "")] = numToStr(nowSec() - varNum(vars, pstr(p, "name", "")));
      dur = 0;
    } else if (type == "time_sleep") {
      dur = pnum(p, "dur", 1);
    } else if (type == "drone_sleep") {
      dur = pnum(p, "dur", 1);
    }

    state.totalDuration += dur;
  }
}

SimResult simulateCommands(const std::vector<SimCommand>& commands) {
  return simulateSingleDrone(commands, 0, 0, 0);
}

std::map<std::string, SimResult> simulateSwarm(const std::vector<DroneSpec>& drones) {
  std::map<std::string, SimResult> results;
  double maxDuration = 0;
  for (const DroneSpec& drone : drones) {
    SimResult result = simulateSingleDrone(drone.commands, drone.offsetX, drone.offsetY, drone.offsetZ);
    if (result.totalDuration > maxDuration) maxDuration = result.totalDuration;
    results[drone.id] = std::move(result);
  }
  for (auto& kv : results) {
    kv.second.totalDuration = maxDuration;
  }
  return results;
}
