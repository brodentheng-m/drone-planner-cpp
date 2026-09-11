#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <string>
#include <vector>
#include <map>

inline constexpr double DT = 0.05;
inline constexpr double TAKEOFF_HEIGHT = 0.8;
inline constexpr double DEFAULT_SPEED = 0.5;
inline constexpr double MAX_PITCH = 25;
inline constexpr double MAX_ROLL = 25;
inline constexpr double MAX_YAW_RATE = 180;
inline constexpr double FLIP_TRAVEL = 0.35;
inline constexpr double FLIP_CLIMB = 0.45;
inline constexpr int FLIP_NUM_POINTS = 200;
inline constexpr int FLIP_SETTLE = 8;

struct SimPoint {
  double x = 0;
  double y = 0;
  double z = 0;
  double heading = 0;
  double pitch = 0;
  double roll = 0;
  bool hasLed = false;
  int ledR = 0;
  int ledG = 0;
  int ledB = 0;
  int ledBrightness = 100;
  bool hasSpeed = false;
  double speed = 0;
  double energyUsed = 0;
  double batteryPercent = 100;
  double turnRadiusM = 0;
};

struct Collision {
  double x = 0;
  double y = 0;
  double z = 0;
  int obstacleId = 0;
};

struct SimResult {
  std::vector<SimPoint> positions;
  double totalDuration = 0;
  std::vector<Collision> collisions;
};

struct SimCommand {
  std::string type;
  std::map<std::string, std::string> params;
  std::vector<SimCommand> children;
};

struct DroneSpec {
  std::string id;
  std::string name;
  std::string color;
  std::vector<SimCommand> commands;
  double offsetX = 0;
  double offsetY = 0;
  double offsetZ = 0;
};

SimResult simulateCommands(const std::vector<SimCommand>& commands);

std::map<std::string, SimResult> simulateSwarm(const std::vector<DroneSpec>& drones);

#endif
