#ifndef __VS_STRUTILS_H__
#define __VS_STRUTILS_H__
#include <string>
#include <sstream>
#include <vector>

namespace vs
{

/** \brief judge if line's header is target*/
bool match(const std::string& line, const std::string& target);

/** \brief judge if line contains substr*/
bool has(const std::string & line, const std::string& substr);

/** \brief cut the front part of line until the first occur of target end*/
std::string cut(const std::string& line, const std::string& target);

/** \brief cut line to two part at the first occur of target end*/
void cut(const std::string& line, const std::string& target, std::string& front, std::string& back);

/** \brief delete the space in head/tail of str*/
void strim(std::string &str);

/** \brief Convert string to vector of double numbers */
std::vector<double> str2vec(const std::string& s);

/** \brief convert a number or a object which overload operate << to string type*/
template <typename T>
std::string num2str(T a)
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

/** \brief convert a string to number of object which overload operate >> */
template <typename T>
T str2num(const std::string &a)
{
    T res;
    std::stringstream ss;
    ss << a;
    ss >> res;
    return res;
}

} /* namespace vs */
#endif//__VS_STRUTILS_H__