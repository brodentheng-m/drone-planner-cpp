#ifndef CODEGENERATOR_H
#define CODEGENERATOR_H

#include <string>
#include <vector>
#include <map>
#include "sim/Simulator.h"

std::string generateDroneScript(const std::vector<SimCommand>& commands);

std::string generateSwarmScript(const std::vector<std::pair<std::string, std::vector<SimCommand>>>& drones);

#endif
