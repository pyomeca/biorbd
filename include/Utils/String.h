#ifndef BIORBD_UTILS_STRING_H
#define BIORBD_UTILS_STRING_H

#include <iostream>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
///
/// \brief Class String
///
class BIORBD_API String : public std::string
{
public:
    ///
    /// \brief Construct string
    ///
    String();
    ///
    /// \brief Construct string
    /// \param c String in char
    ///
    String(const char *c);

    ///
    /// \brief Construct string
    /// \param s String in string
    ///
    String(const biorbd::utils::String &s);
    ///
    /// \brief Construct string
    /// \param c String in string char
    ///
    String(const std::basic_string<char> &c);
    ///
    /// \brief To use operator "=" 
    /// \param other The other string
    ///
    String& operator=(const biorbd::utils::String& other);

    ///
    /// \brief To use operator "+" 
    /// \param d The item to add (unsigned int)
    ///
    String operator+(unsigned int d);
    ///
    /// \brief To use operator "+" 
    /// \param d The item to add (int)
    ///
    String operator+(int d);
    ///
    /// \brief To use operator "+" 
    /// \param d The item to add (double)
    ///
    String operator+(double d);
    ///
    /// \brief To use operator "+" 
    /// \param c The item to add (char)
    ///
    String operator+(const char *c);
    ///
    /// \brief TODO:
    /// \param i TODO:
    ///
    String operator()(unsigned int i) const;
    ///
    /// \brief TODO:
    /// \param i TODO:
    /// \param j TODO:
    ///
    String operator()(unsigned int i , unsigned int j) const;

    ///
    /// \brief Destroy class properly
    ///
    virtual ~String();

    ///
    /// \brief Convert a string to a lower case string
    /// \param str The string to convert
    /// \return The new string
    ///
    static biorbd::utils::String tolower(const biorbd::utils::String &str); 
    ///
    /// \brief Return a lower case string
    /// \return The lower case string
    ///
    biorbd::utils::String tolower() const;

    ///
    /// \brief Convert a string to a upper case string
    /// \param str The string to convert
    /// \return The new string
    ///
    static biorbd::utils::String toupper(const biorbd::utils::String &str); 
    ///
    /// \brief Return an upper case string
    /// \return The upper case string
    ///
    biorbd::utils::String toupper() const;

};

}}
///
/// \brief To use operator "<<"
/// \param os TODO:
/// \param a TODO:
///
std::ostream& operator<<(std::ostream& os, const biorbd::utils::String &a);

#endif // BIORBD_UTILS_STRING_H
