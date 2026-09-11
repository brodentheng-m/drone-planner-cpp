#include "codegen/CodeGenerator.h"
#include "sim/Simulator.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void report(bool ok, const std::string& name) {
  std::cout << (ok ? "PASS" : "FAIL") << ": " << name << "\n";
  if (!ok) ++g_failures;
}

SimCommand makeCmd(const std::string& type, const std::map<std::string, std::string>& params) {
  SimCommand cmd;
  cmd.type = type;
  cmd.params = params;
  return cmd;
}

bool contains(const std::string& hay, const std::string& needle) {
  return hay.find(needle) != std::string::npos;
}

}

int main() {
  std::vector<SimCommand> commands;
  commands.push_back(makeCmd("takeoff", {}));
  commands.push_back(makeCmd("move_forward", {{"dist", "50"}}));
  commands.push_back(makeCmd("turn_degree", {{"deg", "90"}, {"degree", "90"}, {"timeout", "3"}, {"p_value", "10"}}));
  commands.push_back(makeCmd("led", {{"color", "green"}}));
  commands.push_back(makeCmd("led_off", {}));
  commands.push_back(makeCmd("buzzer", {{"freq", "440"}, {"dur", "0.5"}}));
  commands.push_back(makeCmd("keep_distance", {{"dist", "50"}}));
  commands.push_back(makeCmd("avoid_wall", {{"dist", "70"}}));
  commands.push_back(makeCmd("square_turn", {{"speed", "50"}, {"secs", "1"}, {"dir", "1"}}));
  commands.push_back(makeCmd("flip", {{"dir", "back"}}));
  commands.push_back(makeCmd("spiral", {{"speed", "50"}, {"dir", "1"}}));
  commands.push_back(makeCmd("sway", {{"speed", "50"}, {"secs", "2"}, {"dir", "left-right"}}));
  commands.push_back(makeCmd("hover", {{"dur", "1"}}));
  commands.push_back(makeCmd("land", {}));

  const std::string script = generateDroneScript(commands);

  std::cout << "--- generated script (first 6 lines) ---\n";
  {
    std::istringstream in(script);
    std::string line;
    int shown = 0;
    while (shown < 6 && std::getline(in, line)) {
      std::cout << line << "\n";
      ++shown;
    }
  }
  std::cout << "----------------------------------------\n";

  report(script.find("from codrone_edu.drone import *") == 0,
         "script starts with \"from codrone_edu.drone import *\"");
  report(contains(script, "drone.set_drone_LED(0, 255, 0, 100)"),
         "led green maps to drone.set_drone_LED(0, 255, 0, 100)");
  report(contains(script, "drone.keep_distance(2, 50)"),
         "keep_distance maps to drone.keep_distance(2, 50)");
  report(contains(script, "turn_degree") && contains(script, "90"),
         "turn_degree command present with degree 90");

  const char* badTokens[] = {"set_led(", "random_color(", "set_buzzer(", "square_turn(", "from drone import"};
  bool noBad = true;
  for (const char* tok : badTokens) {
    if (contains(script, tok)) {
      std::cout << "  found invalid token: " << tok << "\n";
      noBad = false;
    }
  }
  size_t pos = 0;
  while ((pos = script.find("import codrone", pos)) != std::string::npos) {
    if (script.compare(pos + 14, 4, "_edu") != 0) noBad = false;
    pos += 14;
  }
  report(noBad, "no invalid tokens present");

  {
    std::ofstream out("/tmp/gen_cg.py");
    out << script << "\n";
  }
  if (std::system("python3 --version >/dev/null 2>&1") != 0) {
    std::cout << "WARN: python3 not available, skipping Python syntax validation\n";
  } else {
    const int rc = std::system("python3 -c \"import ast; ast.parse(open('/tmp/gen_cg.py').read())\"");
    report(rc == 0, "generated script parses as valid Python");
  }

  if (g_failures == 0) {
    std::cout << "ALL CHECKS PASSED\n";
    return 0;
  }
  std::cout << g_failures << " CHECK(S) FAILED\n";
  return 1;
}
