#define BIORBD_API_EXPORTS
#include "Utils/String.h"

#include <sstream>
#include <map>
#include <algorithm>
#include <vector>
#include "Utils/Error.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

utils::String::String()
    : std::string("")
{

}

utils::String::String(
    const char *text)
    : std::string(text)
{

}

utils::String::String(
    const String &text)
    : std::string(text)
{

}

utils::String::String(
    const std::basic_string<char> &text)
    : std::string(text)
{

}

utils::String &utils::String::operator=(
    const utils::String &other)
{
    if (this==&other) { // check for self-assigment
        return *this;
    }

    this->std::string::operator=(other);
    return *this;
}

utils::String utils::String::operator+(
    const char *text)
{
    String tp = *this;
    tp.append(text);
    return tp;
}

utils::String utils::String::operator+(
    double val)
{
    return *this + to_string(val);
}

utils::String utils::String::operator+(
    unsigned int val)
{
    return *this + std::to_string(val);
}

utils::String utils::String::operator+(
    int val)
{
    return *this + std::to_string(val);
}

utils::String utils::String::operator()(
    unsigned int idx) const
{
    utils::Error::check(idx<this->length(),
                                "Index for string out of range");
    char out[2];
    out[0] = (*this)[idx];
    out[1] = '\0';
    return out;
}

utils::String utils::String::operator()(
    unsigned int startIdx,
    unsigned int lastIdx) const
{
    utils::Error::check((startIdx<this->length()
                                 || lastIdx<this->length()), "Index for string out of range");
    utils::Error::check(lastIdx>startIdx,
                                "Second argument should be higher than first!");
    char *out = static_cast<char*>(malloc(lastIdx-startIdx+2*sizeof(char)));
    for (unsigned int k=0; k<lastIdx-startIdx+1; ++k) {
        out[k] = (*this)[startIdx+k];
    }
    out[lastIdx-startIdx+1] = '\0';
    utils::String Out(out);
    free(out);
    return Out;
}

utils::String::~String()
{

}

utils::String utils::String::tolower(const utils::String
        &str)
{
    utils::String new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::tolower);
    return new_str;
}

utils::String utils::String::tolower() const
{
    return tolower(*this);
}

utils::String utils::String::toupper(const utils::String
        &str)
{
    utils::String new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::toupper);
    return new_str;
}

utils::String utils::String::toupper() const
{
    return toupper(*this);
}

utils::String utils::String::to_string(
    double val)
{
    std::ostringstream out;
    out.precision(20);
    out << std::fixed << val;
    return removeTrailing(out.str(), "0");
}

utils::String utils::String::to_string(
    float val)
{
    std::ostringstream out;
    out.precision(20);
    out << std::fixed << val;
    return removeTrailing(out.str(), "0");
}

utils::String utils::String::removeTrailing(
    const utils::String &origin,
    const utils::String &trailTag)
{
    utils::Error::check(trailTag.length() == 1,
                                "Tag should be of length 1");
    utils::String out(origin);

    while(out.length() > 0 && out.back() == trailTag[0]) {
        out.pop_back();
    }
    return out;
}

std::ostream &operator<<(std::ostream &os, const utils::String &a)
{
    os << a.c_str();
    return os;
}
