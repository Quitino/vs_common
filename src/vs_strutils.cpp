#include "vs_strutils.h"

namespace vs
{

bool match(const std::string& line, const std::string& target)
{
    return line.substr(0, target.length()) == target;
}

bool has(const std::string & line, const std::string& substr)
{
    return line.find(substr.c_str()) != line.npos;
}

std::string cut(const std::string& line, const std::string& target)
{
    size_t i = line.find(target.c_str());
    if (i == line.npos) return "";
    else return line.substr(i + target.length());
}

void cut(const std::string& line, const std::string& target, std::string& front, std::string& back)
{
    size_t i = line.find(target.c_str());
    if (i == line.npos) {
        front = line;
        back = "";
    } else {
        front = line.substr(0, i);
        back = line.substr(i + target.length());
    }
}

void strim(std::string &str)
{
    if (str.empty())
    {
        return;
    }
    str.erase(0, str.find_first_not_of(' '));
    str.erase(str.find_last_not_of("\r ") + 1);
}


std::vector<double> str2vec(const std::string& s)
{
    std::string a = s;
    strim(a);
    std::vector<double> data; data.reserve(500);
    std::stringstream ss(a.c_str());
    std::string t;
    while (getline(ss, t, ' '))
    {
        if (t.length() == 0) continue;
        data.push_back(atof(t.c_str()));
    }

    return data;
}

} /* namespace vs */