#ifndef EVAL_H
#define EVAL_H

#include <string>
#include <map>

namespace eval {
std::string replaceVars(const std::string& expr, const std::map<std::string, std::string>& vars);
double evaluate(const std::string& expr, const std::map<std::string, std::string>& vars);
}

#endif
