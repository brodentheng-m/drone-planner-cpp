#ifndef SIMULATORINTERNAL_H
#define SIMULATORINTERNAL_H

#include <string>
#include <vector>
#include <map>
#include "aero/AeroEngine.h"
#include "sim/Simulator.h"

struct SimState {
  double x = 0;
  double y = 0;
  double z = 0;
  double heading = 0;
  bool flying = false;
  bool ledOn = false;
  int ledR = 0;
  int ledG = 0;
  int ledB = 0;
  int ledBrightness = 100;
  std::vector<SimPoint> positions;
  double totalDuration = 0;
  std::vector<Collision> collisions;
  AeroEngine engine;
};

double pnum(const std::map<std::string, std::string>& p, const std::string& k, double dflt);
std::string pstr(const std::map<std::string, std::string>& p, const std::string& k, const std::string& dflt);

struct SimControl {
  double throttle = 0;
  double pitch = 0;
  double roll = 0;
  double yaw = 0;
  bool hasTargetAltitude = false;
  double targetAltitude = 0;
  double targetVertVel = 0;
};

SimControl computeControl(const AeroEngine& engine, double tx, double ty, double tz, double th, double dt);
void driveStep(SimState& state, AeroEngine& engine, double tx, double ty, double tz, double th, double dt);
void pushPoint(SimState& state, AeroEngine& engine, double heading, double pitch, double roll, bool hasLed, int r, int g, int b, int brightness);
void processCommands(const std::vector<SimCommand>& commands, std::map<std::string, std::string>& vars, SimState& state, int maxIter);

#endif
