#ifndef BIORBD_UTILS_STRING_H
#define BIORBD_UTILS_STRING_H

#include <iostream>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {

class BIORBD_API String : public std::string
{
public:
    String();
    String(const char *c);
    String(const biorbd::utils::String &s);
    String(const std::basic_string<char> &c);
    String operator+(const unsigned int);
    String operator+(const int);
    String operator+(const double);
    String operator+(const char *c);
    String operator()(const unsigned int) const;
    String operator()(const unsigned int, const unsigned int) const;
    virtual ~String();

    static biorbd::utils::String tolower(const biorbd::utils::String &str); // convert a string to a lower case string
    biorbd::utils::String tolower() const;
    static biorbd::utils::String toupper(const biorbd::utils::String &str); // convert a string to a lower case string
    biorbd::utils::String toupper() const;

};

}}

#endif // BIORBD_UTILS_STRING_H
