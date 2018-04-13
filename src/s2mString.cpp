#include "../include/s2mString.h"

s2mString::s2mString()
    : std::string("")
{

}

s2mString::s2mString(const char *c)
    : std::string(c)
{

}

s2mString::s2mString(const s2mString &s)
    : std::string(s)
{

}

s2mString::s2mString(const std::basic_string<char> &c)
    : std::string(c)
{

}

s2mString s2mString::operator+(const char *c){
    s2mString tp = *this;
    tp.append(c);
    return tp;
}
s2mString s2mString::operator+(const double d){
    return *this + boost::lexical_cast<std::string>(d);
}
s2mString s2mString::operator+(const unsigned int d){
    return *this + boost::lexical_cast<std::string>(d);
}
s2mString s2mString::operator+(const int d){
    return *this + boost::lexical_cast<std::string>(d);
}

s2mString s2mString::operator()(const unsigned int i) const{
    s2mError::s2mAssert(i<this->length(), "Index for string out of range");
    char out[2];
    out[0] = (*this)[i];
    out[1] = '\0';
    return out;
}

s2mString s2mString::operator()(const unsigned int i, const unsigned int j) const{
    s2mError::s2mAssert((i<this->length() || j<this->length()), "Index for string out of range");
    s2mError::s2mAssert(j>i, "Second argument should be higher than first!");
    char *out = (char*)malloc(j-i+2*sizeof(char));
    for (unsigned int k=0; k<j-i+1; ++k)
        out[k] = (*this)[i+k];
    out[j-i+1] = '\0';
    s2mString Out(out);
    free(out);
    return Out;
}

s2mString::~s2mString()
{

}

s2mString s2mString::tolower(const s2mString &str){
    s2mString new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::tolower);
    return new_str;
}

s2mString s2mString::tolower() const{
    return tolower(*this);
}

s2mString s2mString::toupper(const s2mString &str){
    s2mString new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::toupper);
    return new_str;
}

s2mString s2mString::toupper() const{
    return toupper(*this);
}

