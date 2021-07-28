/**
* @file csv_input.h
* @brief input csv header file
* @author Michikuni Eguchi
* @date 2021.7.28
* @details
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace csv{

class csv_input
{
public:
    csv_input(const std::string filePath);
    double readCSV(int line, int col);

private:
    std::string line;
    std::vector<std::vector<std::string>> strcon;
    std::vector<std::string> split(const std::string& s, const std::string separator, bool ignore_empty = 0, bool split_empty = 0);

};

std::vector<std::string> split(const std::string& s, const std::string separator, bool ignore_empty = 0, bool split_empty = 0) {
  struct {
    auto len(const std::string&             s) { return s.length(); }
    auto len(const std::string::value_type* p) { return p ? std::char_traits<std::string::value_type>::length(p) : 0; }
    auto len(const std::string::value_type  c) { return c == std::string::value_type() ? 0 : 1; /*return 1;*/ }
  } util;
  
  if (s.empty()) { /// empty string ///
    if (!split_empty || util.len(separator)) return {""};
    return {};
  }
  
  auto v = std::vector<std::string>();
  auto n = static_cast<std::string::size_type>(util.len(separator));
  if (n == 0) {    /// empty separator ///
    if (!split_empty) return {s};
    for (auto&& c : s) v.emplace_back(1, c);
    return v;
  }
  
  auto p = std::string::size_type(0);
  while (1) {      /// split with separator ///
    auto pos = s.find(separator, p);
    if (pos == std::string::npos) {
      if (ignore_empty && p - n + 1 == s.size()) break;
      v.emplace_back(s.begin() + p, s.end());
      break;
    }
    if (!ignore_empty || p != pos)
      v.emplace_back(s.begin() + p, s.begin() + pos);
    p = pos + n;
  }
  return v;
}

csv_input::csv_input(const std::string filePath)
{
    int i=0;

    std::ifstream ifs(filePath);
    while (getline(ifs, line)) {
        strcon[i] = split(line, ",");
        i++;
    }
}

double csv_input::readCSV(int line, int col)
{
    return std::stod(strcon[line][col]);
}

}//namespace csv
