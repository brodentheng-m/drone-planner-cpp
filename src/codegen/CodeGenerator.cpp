#include "codegen/CodeGenerator.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace {

std::string praw(const SimCommand& cmd, const std::string& key) {
  const auto it = cmd.params.find(key);
  return it == cmd.params.end() ? std::string() : it->second;
}

std::string pdef(const SimCommand& cmd, const std::string& key, const std::string& dflt) {
  const std::string v = praw(cmd, key);
  return v.empty() ? dflt : v;
}

double jsNumber(const std::string& s) {
  if (s.empty()) return 0.0;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) return std::nan("");
  while (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r' || *end == '\f' || *end == '\v') ++end;
  if (*end != '\0') return std::nan("");
  return v;
}

std::string moveSpeedStr(const SimCommand& cmd) {
  const double raw = jsNumber(praw(cmd, "speed"));
  const double speed = std::round((raw / 100.0) * 2.0 * 100.0) / 100.0;
  if (std::isnan(speed)) return "NaN";
  char buf[64];
  if (speed == std::trunc(speed)) {
    std::snprintf(buf, sizeof(buf), "%.1f", speed);
    return buf;
  }
  std::snprintf(buf, sizeof(buf), "%.2f", speed);
  std::string s = buf;
  while (!s.empty() && s.back() == '0') s.pop_back();
  if (!s.empty() && s.back() == '.') s.pop_back();
  return s;
}

std::string dirSign(const SimCommand& cmd) {
  return praw(cmd, "dir") == "clockwise" ? "1" : "-1";
}

struct Rgb {
  int r;
  int g;
  int b;
};

Rgb ledRgb(const std::string& color) {
  if (color == "red") return {255, 0, 0};
  if (color == "green") return {0, 255, 0};
  if (color == "blue") return {0, 0, 255};
  if (color == "yellow") return {255, 255, 0};
  if (color == "cyan") return {0, 255, 255};
  if (color == "magenta") return {255, 0, 255};
  if (color == "white") return {255, 255, 255};
  if (color == "purple") return {128, 0, 255};
  if (color == "orange") return {255, 165, 0};
  if (color == "pink") return {255, 192, 203};
  return {0, 255, 0};
}

const std::pair<int, const char*> kNotes[] = {
  {261, "C4"}, {294, "D4"}, {329, "E4"}, {349, "F4"}, {392, "G4"},
  {440, "A4"}, {494, "B4"}, {523, "C5"}, {659, "E5"}, {880, "A5"}
};

std::string buzzerNote(double freq) {
  int nearest = kNotes[0].first;
  const char* name = kNotes[0].second;
  double best = std::fabs(freq - nearest);
  for (const auto& n : kNotes) {
    const double d = std::fabs(freq - n.first);
    if (d < best) {
      best = d;
      nearest = n.first;
      name = n.second;
    }
  }
  return name;
}

int swayDir(const std::string& dir) {
  if (dir == "turn-right" || dir == "pitch-backward" || dir == "roll-right") return -1;
  return 1;
}

std::string replaceAll(std::string s, const std::string& from, const std::string& to) {
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
  return s;
}

std::string commandCode(const SimCommand& cmd) {
  const std::string& t = cmd.type;
  if (t == "takeoff") return "drone.takeoff()";
  if (t == "land") return "drone.land()";
  if (t == "emergency_stop") return "drone.emergency_stop()";
  if (t == "stop_motors") return "drone.stop_motors()";
  if (t == "hover") return "drone.hover(" + praw(cmd, "dur") + ")";
  if (t == "flip") return "drone.flip(\"" + praw(cmd, "dir") + "\")";
  if (t == "go") return "drone.go(\"" + praw(cmd, "dir") + "\", " + praw(cmd, "power") + ", " + praw(cmd, "dur") + ")";
  if (t == "move_forward" || t == "move_backward" || t == "move_left" || t == "move_right") {
    return "drone." + t + "(" + praw(cmd, "dist") + ", speed=" + moveSpeedStr(cmd) + ")";
  }
  if (t == "turn_left" || t == "turn_right") return "drone." + t + "(" + praw(cmd, "deg") + ")";
  if (t == "turn_degree") return "drone.turn_degree(" + praw(cmd, "deg") + ", timeout=" + praw(cmd, "timeout") + ", p_value=" + praw(cmd, "p_value") + ")";
  if (t == "circle") return "drone.circle(speed=" + praw(cmd, "speed") + ", direction=" + dirSign(cmd) + ")";
  if (t == "circle_turn") return "drone.circle_turn(speed=" + praw(cmd, "speed") + ", seconds=" + praw(cmd, "secs") + ", direction=" + dirSign(cmd) + ")";
  if (t == "square" || t == "square_turn") return "drone.square(speed=" + praw(cmd, "speed") + ", seconds=" + praw(cmd, "secs") + ", direction=" + dirSign(cmd) + ")";
  if (t == "triangle") return "drone.triangle(speed=" + praw(cmd, "speed") + ", seconds=" + praw(cmd, "secs") + ", direction=" + dirSign(cmd) + ")";
  if (t == "triangle_turn") return "drone.triangle_turn(speed=" + praw(cmd, "speed") + ", seconds=" + praw(cmd, "secs") + ", direction=" + dirSign(cmd) + ")";
  if (t == "spiral") return "drone.spiral(speed=" + praw(cmd, "speed") + ", seconds=3, direction=" + dirSign(cmd) + ")";
  if (t == "sway") return "drone.sway(speed=" + praw(cmd, "speed") + ", seconds=" + praw(cmd, "secs") + ", direction=" + std::to_string(swayDir(praw(cmd, "dir"))) + ")";
  if (t == "keep_distance") return "drone.keep_distance(2, " + praw(cmd, "dist") + ")";
  if (t == "avoid_wall") return "drone.avoid_wall(2, " + praw(cmd, "dist") + ")";
  if (t == "detect_wall") return praw(cmd, "var") + " = drone.detect_wall()";
  if (t == "led") {
    const std::string color = praw(cmd, "color");
    if (color == "off") return "drone.drone_LED_off()";
    const Rgb c = ledRgb(color);
    return "drone.set_drone_LED(" + std::to_string(c.r) + ", " + std::to_string(c.g) + ", " + std::to_string(c.b) + ", 100)";
  }
  if (t == "led_off") return "drone.drone_LED_off()";
  if (t == "random_led") return "drone.set_drone_LED(0, 255, 128, 100)";
  if (t == "buzzer") return "drone.drone_buzzer(\"" + buzzerNote(jsNumber(praw(cmd, "freq"))) + "\", " + praw(cmd, "dur") + ")";
  if (t == "drone_sleep") return "time.sleep(" + praw(cmd, "dur") + ")";
  if (t == "var_declare") return praw(cmd, "name") + " = " + praw(cmd, "value");
  if (t == "set_var") return praw(cmd, "name") + " " + praw(cmd, "op") + " " + praw(cmd, "value");
  if (t == "print_var") return "print(" + praw(cmd, "value") + ")";
  if (t == "break_cmd") return "break";
  if (t == "get_battery") return praw(cmd, "var") + " = drone.get_battery()";
  if (t == "get_height") return praw(cmd, "var") + " = drone.get_height(unit=\"" + praw(cmd, "unit") + "\")";
  if (t == "get_front_range") return praw(cmd, "var") + " = drone.get_front_range(unit=\"" + praw(cmd, "unit") + "\")";
  if (t == "get_bottom_range") return praw(cmd, "var") + " = drone.get_bottom_range(unit=\"" + praw(cmd, "unit") + "\")";
  if (t == "get_front_color") return praw(cmd, "var") + " = drone.get_front_color(kind=\"" + praw(cmd, "kind") + "\")";
  if (t == "get_back_color") return praw(cmd, "var") + " = drone.get_back_color(kind=\"" + praw(cmd, "kind") + "\")";
  if (t == "get_temperature") return praw(cmd, "var") + " = drone.get_temperature(unit=\"" + praw(cmd, "unit") + "\")";
  if (t == "get_distance") return praw(cmd, "var") + " = drone.get_front_range()";
  if (t == "func_call") return praw(cmd, "name") + "()";
  if (t == "return_val") return "return " + praw(cmd, "value");
  if (t == "list_declare") return praw(cmd, "name") + " = [" + praw(cmd, "values") + "]";
  if (t == "list_append") return praw(cmd, "name") + ".append(" + praw(cmd, "value") + ")";
  if (t == "list_get") return praw(cmd, "var") + " = " + praw(cmd, "list_name") + "[" + praw(cmd, "index") + "]";
  if (t == "user_input") return praw(cmd, "var") + " = input(\"" + praw(cmd, "prompt") + "\")";
  if (t == "timer_start") return praw(cmd, "name") + " = time.time()";
  if (t == "timer_elapsed") return praw(cmd, "var") + " = time.time() - " + praw(cmd, "name");
  return "";
}

void generateBlockCode(const std::vector<SimCommand>& commands, std::vector<std::string>& lines, int indent, const std::string& droneVar) {
  const std::string pad(static_cast<size_t>(indent) * 4, ' ');
  for (const SimCommand& cmd : commands) {
    const std::string& t = cmd.type;
    if (t == "if_block") {
      lines.push_back(pad + "if " + praw(cmd, "condition") + ":");
      if (!cmd.children.empty()) generateBlockCode(cmd.children, lines, indent + 1, droneVar);
      else lines.push_back(pad + "    pass");
    } else if (t == "elif_block") {
      lines.push_back(pad + "elif " + praw(cmd, "condition") + ":");
      if (!cmd.children.empty()) generateBlockCode(cmd.children, lines, indent + 1, droneVar);
      else lines.push_back(pad + "    pass");
    } else if (t == "else_block") {
      lines.push_back(pad + "else:");
      if (!cmd.children.empty()) generateBlockCode(cmd.children, lines, indent + 1, droneVar);
      else lines.push_back(pad + "    pass");
    } else if (t == "end_block") {
    } else if (t == "while_block") {
      lines.push_back(pad + "while " + praw(cmd, "condition") + ":");
      if (!cmd.children.empty()) generateBlockCode(cmd.children, lines, indent + 1, droneVar);
      else lines.push_back(pad + "    pass");
    } else if (t == "for_block") {
      lines.push_back(pad + "for " + pdef(cmd, "var", "i") + " in range(" + pdef(cmd, "start", "0") + ", " + pdef(cmd, "end_val", "5") + ", " + pdef(cmd, "step", "1") + "):");
      if (!cmd.children.empty()) generateBlockCode(cmd.children, lines, indent + 1, droneVar);
      else lines.push_back(pad + "    pass");
    } else if (t == "func_def") {
      lines.push_back(pad + "def " + praw(cmd, "name") + "():");
      if (!cmd.children.empty()) generateBlockCode(cmd.children, lines, indent + 1, droneVar);
      else lines.push_back(pad + "    pass");
    } else {
      std::string code = commandCode(cmd);
      if (!code.empty()) {
        if (droneVar != "drone") code = replaceAll(code, "drone.", droneVar + ".");
        lines.push_back(pad + code);
      }
    }
  }
}

std::string joinLines(const std::vector<std::string>& lines) {
  std::string out;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (i > 0) out += '\n';
    out += lines[i];
  }
  return out;
}

}

std::string generateDroneScript(const std::vector<SimCommand>& commands) {
  std::vector<std::string> lines = {
    "from codrone_edu.drone import *",
    "import time",
    "",
    "drone = Drone()",
    "drone.pair()",
    ""
  };
  generateBlockCode(commands, lines, 0, "drone");
  lines.push_back("");
  lines.push_back("drone.close()");
  return joinLines(lines);
}

std::string generateSwarmScript(const std::vector<std::pair<std::string, std::vector<SimCommand>>>& drones) {
  if (drones.size() == 1) return generateDroneScript(drones[0].second);
  std::vector<std::string> lines = {
    "from codrone_edu.drone import *",
    "import time",
    "import threading",
    ""
  };
  for (size_t i = 0; i < drones.size(); ++i) {
    lines.push_back("drone" + std::to_string(i + 1) + " = Drone()");
  }
  lines.push_back("");
  for (size_t i = 0; i < drones.size(); ++i) {
    lines.push_back("drone" + std::to_string(i + 1) + ".pair()");
  }
  lines.push_back("");
  lines.push_back("time.sleep(2)");
  lines.push_back("");
  for (size_t i = 0; i < drones.size(); ++i) {
    const std::string n = std::to_string(i + 1);
    lines.push_back("def fly_drone" + n + "():");
    lines.push_back("    d = drone" + n);
    generateBlockCode(drones[i].second, lines, 1, "d");
    lines.push_back("");
  }
  lines.push_back("threads = []");
  for (size_t i = 0; i < drones.size(); ++i) {
    const std::string n = std::to_string(i + 1);
    lines.push_back("t" + n + " = threading.Thread(target=fly_drone" + n + ")");
    lines.push_back("threads.append(t" + n + ")");
  }
  lines.push_back("");
  lines.push_back("for t in threads:");
  lines.push_back("    t.start()");
  lines.push_back("for t in threads:");
  lines.push_back("    t.join()");
  lines.push_back("");
  for (size_t i = 0; i < drones.size(); ++i) {
    lines.push_back("drone" + std::to_string(i + 1) + ".close()");
  }
  return joinLines(lines);
}
