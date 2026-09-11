#include "bridge/Json.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

struct Parser {
    const std::string& s;
    size_t i = 0;
    std::string* err;

    Parser(const std::string& s, std::string* err) : s(s), err(err) {}

    bool fail(const char* m) {
        if (err) *err = m;
        return false;
    }

    void ws() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
    }

    bool parseValue(JsonValue& v) {
        ws();
        if (i >= s.size()) return fail("unexpected end of input");
        char c = s[i];
        if (c == '{') return parseObject(v);
        if (c == '[') return parseArray(v);
        if (c == '"') {
            v.type = JsonValue::String;
            return parseString(v.str);
        }
        if (c == 't') return parseLiteral("true", JsonValue::Bool, v, true);
        if (c == 'f') return parseLiteral("false", JsonValue::Bool, v, false);
        if (c == 'n') return parseLiteral("null", JsonValue::Null, v, false);
        if (c == '-' || (c >= '0' && c <= '9')) return parseNumber(v);
        return fail("unexpected character");
    }

    bool parseLiteral(const char* lit, JsonValue::Type t, JsonValue& v, bool bval) {
        size_t n = std::strlen(lit);
        if (s.compare(i, n, lit) != 0) return fail("invalid literal");
        i += n;
        v.type = t;
        v.b = bval;
        return true;
    }

    bool parseNumber(JsonValue& v) {
        size_t start = i;
        if (i < s.size() && s[i] == '-') ++i;
        while (i < s.size() && std::isdigit((unsigned char)s[i])) ++i;
        if (i < s.size() && s[i] == '.') {
            ++i;
            while (i < s.size() && std::isdigit((unsigned char)s[i])) ++i;
        }
        if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
            ++i;
            if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
            while (i < s.size() && std::isdigit((unsigned char)s[i])) ++i;
        }
        if (i == start || (i == start + 1 && s[start] == '-')) return fail("invalid number");
        v.type = JsonValue::Number;
        v.num = std::strtod(s.c_str() + start, nullptr);
        return true;
    }

    static void appendUtf8(std::string& out, unsigned cp) {
        if (cp < 0x80) {
            out += (char)cp;
        } else if (cp < 0x800) {
            out += (char)(0xC0 | (cp >> 6));
            out += (char)(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += (char)(0xE0 | (cp >> 12));
            out += (char)(0x80 | ((cp >> 6) & 0x3F));
            out += (char)(0x80 | (cp & 0x3F));
        } else {
            out += (char)(0xF0 | (cp >> 18));
            out += (char)(0x80 | ((cp >> 12) & 0x3F));
            out += (char)(0x80 | ((cp >> 6) & 0x3F));
            out += (char)(0x80 | (cp & 0x3F));
        }
    }

    bool parseHex4(unsigned& cp) {
        if (i + 4 > s.size()) return fail("bad unicode escape");
        cp = 0;
        for (int k = 0; k < 4; ++k) {
            char c = s[i++];
            cp <<= 4;
            if (c >= '0' && c <= '9') cp |= (unsigned)(c - '0');
            else if (c >= 'a' && c <= 'f') cp |= (unsigned)(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') cp |= (unsigned)(c - 'A' + 10);
            else return fail("bad unicode escape");
        }
        return true;
    }

    bool parseString(std::string& out) {
        ++i;
        out.clear();
        while (i < s.size()) {
            char c = s[i++];
            if (c == '"') return true;
            if (c == '\\') {
                if (i >= s.size()) return fail("bad escape");
                char e = s[i++];
                switch (e) {
                    case '"': out += '"'; break;
                    case '\\': out += '\\'; break;
                    case '/': out += '/'; break;
                    case 'b': out += '\b'; break;
                    case 'f': out += '\f'; break;
                    case 'n': out += '\n'; break;
                    case 'r': out += '\r'; break;
                    case 't': out += '\t'; break;
                    case 'u': {
                        unsigned cp;
                        if (!parseHex4(cp)) return false;
                        if (cp >= 0xD800 && cp <= 0xDBFF && i + 1 < s.size() && s[i] == '\\' && s[i + 1] == 'u') {
                            size_t save = i;
                            i += 2;
                            unsigned lo;
                            if (!parseHex4(lo)) return false;
                            if (lo >= 0xDC00 && lo <= 0xDFFF) {
                                cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                            } else {
                                i = save;
                            }
                        }
                        appendUtf8(out, cp);
                        break;
                    }
                    default: return fail("bad escape");
                }
            } else {
                out += c;
            }
        }
        return fail("unterminated string");
    }

    bool parseArray(JsonValue& v) {
        ++i;
        v.type = JsonValue::Array;
        v.arr.clear();
        ws();
        if (i < s.size() && s[i] == ']') {
            ++i;
            return true;
        }
        while (true) {
            JsonValue el;
            if (!parseValue(el)) return false;
            v.arr.push_back(std::move(el));
            ws();
            if (i >= s.size()) return fail("unterminated array");
            if (s[i] == ',') {
                ++i;
                continue;
            }
            if (s[i] == ']') {
                ++i;
                return true;
            }
            return fail("expected , or ]");
        }
    }

    bool parseObject(JsonValue& v) {
        ++i;
        v.type = JsonValue::Object;
        v.obj.clear();
        ws();
        if (i < s.size() && s[i] == '}') {
            ++i;
            return true;
        }
        while (true) {
            ws();
            if (i >= s.size() || s[i] != '"') return fail("expected object key");
            std::string key;
            if (!parseString(key)) return false;
            ws();
            if (i >= s.size() || s[i] != ':') return fail("expected :");
            ++i;
            JsonValue val;
            if (!parseValue(val)) return false;
            v.obj.emplace_back(std::move(key), std::move(val));
            ws();
            if (i >= s.size()) return fail("unterminated object");
            if (s[i] == ',') {
                ++i;
                continue;
            }
            if (s[i] == '}') {
                ++i;
                return true;
            }
            return fail("expected , or }");
        }
    }
};

void serializeInto(const JsonValue& v, std::string& out) {
    switch (v.type) {
        case JsonValue::Null:
            out += "null";
            break;
        case JsonValue::Bool:
            out += v.b ? "true" : "false";
            break;
        case JsonValue::Number: {
            char buf[64];
            if (std::isfinite(v.num) && v.num == std::floor(v.num) && std::fabs(v.num) < 1e15) {
                std::snprintf(buf, sizeof(buf), "%lld", (long long)v.num);
            } else {
                std::snprintf(buf, sizeof(buf), "%.17g", v.num);
            }
            out += buf;
            break;
        }
        case JsonValue::String: {
            out += '"';
            for (char c : v.str) {
                unsigned char uc = (unsigned char)c;
                switch (c) {
                    case '"': out += "\\\""; break;
                    case '\\': out += "\\\\"; break;
                    case '\b': out += "\\b"; break;
                    case '\f': out += "\\f"; break;
                    case '\n': out += "\\n"; break;
                    case '\r': out += "\\r"; break;
                    case '\t': out += "\\t"; break;
                    default:
                        if (uc < 0x20) {
                            char buf[8];
                            std::snprintf(buf, sizeof(buf), "\\u%04x", (unsigned)uc);
                            out += buf;
                        } else {
                            out += c;
                        }
                }
            }
            out += '"';
            break;
        }
        case JsonValue::Array: {
            out += '[';
            for (size_t k = 0; k < v.arr.size(); ++k) {
                if (k) out += ',';
                serializeInto(v.arr[k], out);
            }
            out += ']';
            break;
        }
        case JsonValue::Object: {
            out += '{';
            for (size_t k = 0; k < v.obj.size(); ++k) {
                if (k) out += ',';
                JsonValue key;
                key.type = JsonValue::String;
                key.str = v.obj[k].first;
                serializeInto(key, out);
                out += ':';
                serializeInto(v.obj[k].second, out);
            }
            out += '}';
            break;
        }
    }
}

}

bool jsonParse(const std::string& text, JsonValue& out, std::string* err) {
    Parser p(text, err);
    if (!p.parseValue(out)) return false;
    p.ws();
    if (p.i != text.size()) {
        if (err) *err = "trailing characters";
        return false;
    }
    return true;
}

std::string jsonSerialize(const JsonValue& v) {
    std::string out;
    serializeInto(v, out);
    return out;
}

double jsonNum(const JsonValue& v, const std::string& key, double d) {
    if (v.type != JsonValue::Object) return d;
    for (const auto& kv : v.obj) {
        if (kv.first == key) {
            if (kv.second.type == JsonValue::Number) return kv.second.num;
            return d;
        }
    }
    return d;
}

std::string jsonStr(const JsonValue& v, const std::string& key, const std::string& d) {
    if (v.type != JsonValue::Object) return d;
    for (const auto& kv : v.obj) {
        if (kv.first == key) {
            if (kv.second.type == JsonValue::String) return kv.second.str;
            return d;
        }
    }
    return d;
}

bool jsonHas(const JsonValue& v, const std::string& key) {
    if (v.type != JsonValue::Object) return false;
    for (const auto& kv : v.obj) {
        if (kv.first == key) return true;
    }
    return false;
}
