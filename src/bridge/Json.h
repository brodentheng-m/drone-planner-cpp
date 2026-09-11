#ifndef DRONEPLANNERCPP_BRIDGE_JSON_H
#define DRONEPLANNERCPP_BRIDGE_JSON_H

#include <string>
#include <utility>
#include <vector>

struct JsonValue {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double num = 0;
    std::string str;
    std::vector<JsonValue> arr;
    std::vector<std::pair<std::string, JsonValue>> obj;
};

bool jsonParse(const std::string& text, JsonValue& out, std::string* err = nullptr);
std::string jsonSerialize(const JsonValue& v);
double jsonNum(const JsonValue& v, const std::string& key, double d = 0);
std::string jsonStr(const JsonValue& v, const std::string& key, const std::string& d = "");
bool jsonHas(const JsonValue& v, const std::string& key);

#endif
