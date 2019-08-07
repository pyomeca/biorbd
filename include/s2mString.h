#ifndef S2MSTRING_H
#define S2MSTRING_H

#include <iostream>
#include "biorbdConfig.h"

class BIORBD_API s2mString : public std::string
{
public:
    s2mString();
    s2mString(const char *c);
    s2mString(const s2mString &s);
    s2mString(const std::basic_string<char> &c);
    s2mString operator+(const unsigned int);
    s2mString operator+(const int);
    s2mString operator+(const double);
    s2mString operator+(const char *c);
    s2mString operator()(const unsigned int) const;
    s2mString operator()(const unsigned int, const unsigned int) const;
    ~s2mString();

    static s2mString tolower(const s2mString &str); // convert a string to a lower case string
    s2mString tolower() const;
    static s2mString toupper(const s2mString &str); // convert a string to a lower case string
    s2mString toupper() const;

};

#endif // S2MSTRING_H
