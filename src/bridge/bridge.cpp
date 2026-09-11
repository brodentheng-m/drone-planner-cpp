#include "sim/Simulator.h"
#include "codegen/CodeGenerator.h"
#include "bridge/Json.h"

#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define BRIDGE_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define BRIDGE_EXPORT
#endif

namespace {

const JsonValue* jsonMember(const JsonValue& v, const std::string& key) {
  if (v.type != JsonValue::Object) return nullptr;
  for (const auto& kv : v.obj) {
    if (kv.first == key) return &kv.second;
  }
  return nullptr;
}

JsonValue jnum(double d) {
  JsonValue v;
  v.type = JsonValue::Number;
  v.num = d;
  return v;
}

JsonValue jstr(const std::string& s) {
  JsonValue v;
  v.type = JsonValue::String;
  v.str = s;
  return v;
}

JsonValue jbool(bool b) {
  JsonValue v;
  v.type = JsonValue::Bool;
  v.b = b;
  return v;
}

std::string paramToString(const JsonValue& v) {
  if (v.type == JsonValue::String) return v.str;
  if (v.type == JsonValue::Number) return jsonSerialize(v);
  if (v.type == JsonValue::Bool) return v.b ? "true" : "false";
  if (v.type == JsonValue::Null) return "";
  return jsonSerialize(v);
}

SimCommand toCommand(const JsonValue& v) {
  SimCommand cmd;
  if (v.type != JsonValue::Object) return cmd;
  cmd.type = jsonStr(v, "type", "");
  const JsonValue* params = jsonMember(v, "params");
  if (params && params->type == JsonValue::Object) {
    for (const auto& kv : params->obj) {
      cmd.params[kv.first] = paramToString(kv.second);
    }
  }
  const JsonValue* children = jsonMember(v, "children");
  if (children && children->type == JsonValue::Array) {
    for (const auto& c : children->arr) {
      cmd.children.push_back(toCommand(c));
    }
  }
  return cmd;
}

struct ParsedInput {
  bool swarm = false;
  std::vector<DroneSpec> drones;
  std::vector<SimCommand> commands;
};

std::vector<SimCommand> toCommandList(const JsonValue* arr) {
  std::vector<SimCommand> out;
  if (arr && arr->type == JsonValue::Array) {
    for (const auto& c : arr->arr) {
      out.push_back(toCommand(c));
    }
  }
  return out;
}

ParsedInput parseInput(const JsonValue& root) {
  ParsedInput pi;
  const JsonValue* drones = jsonMember(root, "drones");
  if (drones && drones->type == JsonValue::Array && drones->arr.size() > 1) {
    pi.swarm = true;
    int idx = 0;
    for (const auto& d : drones->arr) {
      DroneSpec spec;
      spec.id = jsonStr(d, "id", "");
      if (spec.id.empty()) spec.id = "drone" + std::to_string(idx + 1);
      spec.name = jsonStr(d, "name", "");
      spec.commands = toCommandList(jsonMember(d, "order"));
      pi.drones.push_back(std::move(spec));
      ++idx;
    }
    return pi;
  }
  pi.commands = toCommandList(jsonMember(root, "commands"));
  if (pi.commands.empty() && drones && drones->type == JsonValue::Array && drones->arr.size() == 1) {
    pi.commands = toCommandList(jsonMember(drones->arr[0], "order"));
  }
  return pi;
}

JsonValue pointJson(const SimPoint& p) {
  JsonValue o;
  o.type = JsonValue::Object;
  o.obj.emplace_back("x", jnum(p.x));
  o.obj.emplace_back("y", jnum(p.y));
  o.obj.emplace_back("z", jnum(p.z));
  o.obj.emplace_back("heading", jnum(p.heading));
  o.obj.emplace_back("pitch", jnum(p.pitch));
  o.obj.emplace_back("roll", jnum(p.roll));
  if (p.hasLed) {
    JsonValue led;
    led.type = JsonValue::Object;
    led.obj.emplace_back("r", jnum(p.ledR));
    led.obj.emplace_back("g", jnum(p.ledG));
    led.obj.emplace_back("b", jnum(p.ledB));
    led.obj.emplace_back("brightness", jnum(p.ledBrightness));
    o.obj.emplace_back("led", std::move(led));
  }
  o.obj.emplace_back("speed", jnum(p.speed));
  o.obj.emplace_back("energyUsed", jnum(p.energyUsed));
  o.obj.emplace_back("batteryPercent", jnum(p.batteryPercent));
  o.obj.emplace_back("turnRadiusM", jnum(p.turnRadiusM));
  return o;
}

JsonValue collisionJson(const Collision& c) {
  JsonValue o;
  o.type = JsonValue::Object;
  o.obj.emplace_back("x", jnum(c.x));
  o.obj.emplace_back("y", jnum(c.y));
  o.obj.emplace_back("z", jnum(c.z));
  o.obj.emplace_back("obstacleId", jnum(c.obstacleId));
  return o;
}

JsonValue resultJson(const SimResult& r) {
  JsonValue o;
  o.type = JsonValue::Object;
  JsonValue positions;
  positions.type = JsonValue::Array;
  for (const auto& p : r.positions) {
    positions.arr.push_back(pointJson(p));
  }
  o.obj.emplace_back("positions", std::move(positions));
  o.obj.emplace_back("totalDuration", jnum(r.totalDuration));
  JsonValue collisions;
  collisions.type = JsonValue::Array;
  for (const auto& c : r.collisions) {
    collisions.arr.push_back(collisionJson(c));
  }
  o.obj.emplace_back("collisions", std::move(collisions));
  return o;
}

std::string errJson(const std::string& msg) {
  JsonValue o;
  o.type = JsonValue::Object;
  o.obj.emplace_back("ok", jbool(false));
  o.obj.emplace_back("error", jstr(msg));
  return jsonSerialize(o);
}

char* toCString(const std::string& s) {
  char* p = static_cast<char*>(std::malloc(s.size() + 1));
  if (!p) return nullptr;
  std::memcpy(p, s.c_str(), s.size() + 1);
  return p;
}

}

BRIDGE_EXPORT
std::string engineSimulate(const std::string& jsonIn) {
  try {
    JsonValue root;
    std::string err;
    if (!jsonParse(jsonIn, root, &err)) return errJson(err);
    if (root.type != JsonValue::Object) return errJson("input must be a JSON object");
    ParsedInput pi = parseInput(root);
    if (pi.swarm) {
      std::map<std::string, SimResult> results = simulateSwarm(pi.drones);
      JsonValue out;
      out.type = JsonValue::Object;
      JsonValue res;
      res.type = JsonValue::Object;
      for (auto& kv : results) {
        res.obj.emplace_back(kv.first, resultJson(kv.second));
      }
      out.obj.emplace_back("results", std::move(res));
      out.obj.emplace_back("ok", jbool(true));
      return jsonSerialize(out);
    }
    SimResult result = simulateCommands(pi.commands);
    JsonValue out = resultJson(result);
    out.obj.emplace_back("ok", jbool(true));
    return jsonSerialize(out);
  } catch (const std::exception& e) {
    return errJson(e.what());
  } catch (...) {
    return errJson("unknown error");
  }
}

BRIDGE_EXPORT
std::string engineGenerateCode(const std::string& jsonIn) {
  try {
    JsonValue root;
    std::string err;
    if (!jsonParse(jsonIn, root, &err)) return errJson(err);
    if (root.type != JsonValue::Object) return errJson("input must be a JSON object");
    ParsedInput pi = parseInput(root);
    std::string code;
    if (pi.swarm) {
      std::vector<std::pair<std::string, std::vector<SimCommand>>> drones;
      for (size_t i = 0; i < pi.drones.size(); ++i) {
        std::string name = pi.drones[i].name.empty()
          ? "drone" + std::to_string(i + 1)
          : pi.drones[i].name;
        drones.emplace_back(name, pi.drones[i].commands);
      }
      code = generateSwarmScript(drones);
    } else {
      code = generateDroneScript(pi.commands);
    }
    JsonValue out;
    out.type = JsonValue::Object;
    out.obj.emplace_back("code", jstr(code));
    out.obj.emplace_back("ok", jbool(true));
    return jsonSerialize(out);
  } catch (const std::exception& e) {
    return errJson(e.what());
  } catch (...) {
    return errJson("unknown error");
  }
}

extern "C" BRIDGE_EXPORT char* engineSimulateC(const char* jsonIn) {
  return toCString(engineSimulate(jsonIn ? std::string(jsonIn) : std::string()));
}

extern "C" BRIDGE_EXPORT char* engineGenerateCodeC(const char* jsonIn) {
  return toCString(engineGenerateCode(jsonIn ? std::string(jsonIn) : std::string()));
}
