#include "Eval.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace eval {

namespace {

bool isDigit(char c) {
  return c >= '0' && c <= '9';
}

bool isAlpha(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

bool isWordChar(char c) {
  return isDigit(c) || isAlpha(c) || c == '_';
}

bool isSpace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
}

bool isNumber(const std::string& s) {
  if (s.empty()) return false;
  size_t i = 0;
  if (s[i] == '+' || s[i] == '-') {
    ++i;
    if (i == s.size()) return false;
  }
  bool digit = false;
  while (i < s.size() && isDigit(s[i])) {
    digit = true;
    ++i;
  }
  if (i < s.size() && s[i] == '.') {
    ++i;
    while (i < s.size() && isDigit(s[i])) {
      digit = true;
      ++i;
    }
  }
  return digit && i == s.size();
}

std::string jsonStringify(const std::string& value) {
  if (isNumber(value)) return value;
  std::string out = "\"";
  for (char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: {
        unsigned char uc = static_cast<unsigned char>(c);
        if (uc < 0x20) {
          const char* hex = "0123456789abcdef";
          out += "\\u00";
          out += hex[uc >> 4];
          out += hex[uc & 0x0F];
        } else {
          out += c;
        }
      }
    }
  }
  out += "\"";
  return out;
}

std::string replaceAllWholeWords(const std::string& text, const std::string& name, const std::string& replacement) {
  std::string result;
  const size_t n = text.size();
  const size_t m = name.size();
  size_t i = 0;
  while (i < n) {
    if (text.compare(i, m, name) == 0 &&
        (i == 0 || !isWordChar(text[i - 1])) &&
        (i + m == n || !isWordChar(text[i + m]))) {
      result += replacement;
      i += m;
    } else {
      result += text[i];
      ++i;
    }
  }
  return result;
}

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s), pos_(0) {}

  double parse() {
    double v = parseOr();
    skipWs();
    if (pos_ != s_.size()) throw std::runtime_error("trailing");
    return v;
  }

 private:
  const std::string& s_;
  size_t pos_;

  void skipWs() {
    while (pos_ < s_.size() && isSpace(s_[pos_])) ++pos_;
  }

  bool match(const char* op) {
    skipWs();
    size_t len = std::strlen(op);
    if (pos_ + len <= s_.size() && s_.compare(pos_, len, op) == 0) {
      pos_ += len;
      return true;
    }
    return false;
  }

  double parseOr() {
    double v = parseAnd();
    while (match("||")) {
      double r = parseAnd();
      v = (v != 0.0 || r != 0.0) ? 1.0 : 0.0;
    }
    return v;
  }

  double parseAnd() {
    double v = parseCompare();
    while (match("&&")) {
      double r = parseCompare();
      v = (v != 0.0 && r != 0.0) ? 1.0 : 0.0;
    }
    return v;
  }

  double parseCompare() {
    static const char* const kOps[] = {">=", "<=", "==", "!=", ">", "<"};
    double v = parseExpr();
    while (true) {
      const char* op = nullptr;
      for (const char* candidate : kOps) {
        if (match(candidate)) {
          op = candidate;
          break;
        }
      }
      if (op == nullptr) break;
      double r = parseExpr();
      v = compareValues(v, r, op);
    }
    return v;
  }

  static double compareValues(double a, double b, const char* op) {
    if (std::strcmp(op, ">=") == 0) return a >= b ? 1.0 : 0.0;
    if (std::strcmp(op, "<=") == 0) return a <= b ? 1.0 : 0.0;
    if (std::strcmp(op, "==") == 0) return a == b ? 1.0 : 0.0;
    if (std::strcmp(op, "!=") == 0) return a != b ? 1.0 : 0.0;
    if (std::strcmp(op, ">") == 0) return a > b ? 1.0 : 0.0;
    return a < b ? 1.0 : 0.0;
  }

  double parseExpr() {
    double v = parseTerm();
    while (true) {
      skipWs();
      if (pos_ < s_.size() && (s_[pos_] == '+' || s_[pos_] == '-')) {
        char op = s_[pos_++];
        double rhs = parseTerm();
        v = (op == '+') ? v + rhs : v - rhs;
      } else {
        break;
      }
    }
    return v;
  }

  double parseTerm() {
    double v = parseFactor();
    while (true) {
      skipWs();
      if (pos_ < s_.size() && (s_[pos_] == '*' || s_[pos_] == '/')) {
        char op = s_[pos_++];
        double rhs = parseFactor();
        if (op == '/') {
          if (rhs == 0.0) throw std::runtime_error("div0");
          v = v / rhs;
        } else {
          v = v * rhs;
        }
      } else {
        break;
      }
    }
    return v;
  }

  double parseFactor() {
    skipWs();
    if (pos_ >= s_.size()) throw std::runtime_error("eof");
    char c = s_[pos_];
    if (c == '+') {
      ++pos_;
      return parseFactor();
    }
    if (c == '-') {
      ++pos_;
      return -parseFactor();
    }
    if (c == '!') {
      ++pos_;
      return parseFactor() == 0.0 ? 1.0 : 0.0;
    }
    if (c == '(') {
      ++pos_;
      double v = parseOr();
      skipWs();
      if (pos_ >= s_.size() || s_[pos_] != ')') throw std::runtime_error("paren");
      ++pos_;
      return v;
    }
    if (isDigit(c) || c == '.') return parseNumber();
    throw std::runtime_error("char");
  }

  double parseNumber() {
    size_t start = pos_;
    while (pos_ < s_.size() && isDigit(s_[pos_])) ++pos_;
    if (pos_ < s_.size() && s_[pos_] == '.') {
      ++pos_;
      while (pos_ < s_.size() && isDigit(s_[pos_])) ++pos_;
    }
    double intPart = 0.0;
    size_t i = start;
    while (i < pos_ && s_[i] != '.') {
      intPart = intPart * 10.0 + (s_[i] - '0');
      ++i;
    }
    if (i < pos_ && s_[i] == '.') {
      ++i;
      double frac = 0.0;
      double place = 0.1;
      while (i < pos_) {
        frac += (s_[i] - '0') * place;
        place *= 0.1;
        ++i;
      }
      return intPart + frac;
    }
    return intPart;
  }
};

}

std::string replaceVars(const std::string& expr, const std::map<std::string, std::string>& vars) {
  std::vector<std::string> names;
  names.reserve(vars.size());
  for (const auto& kv : vars) names.push_back(kv.first);
  std::sort(names.begin(), names.end(), [](const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return a.size() > b.size();
    return a < b;
  });
  std::string result = expr;
  for (const std::string& name : names) {
    result = replaceAllWholeWords(result, name, jsonStringify(vars.at(name)));
  }
  return result;
}

double evaluate(const std::string& expr, const std::map<std::string, std::string>& vars) {
  try {
    std::string replaced = replaceVars(expr, vars);
    Parser p(replaced);
    return p.parse();
  } catch (...) {
    return 0.0;
  }
}

}
