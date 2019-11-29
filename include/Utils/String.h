#ifndef BIORBD_UTILS_STRING_H
#define BIORBD_UTILS_STRING_H

#include <iostream>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
///
/// \brief Wrapper around the std::string class with augmented functionality
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
    /// \param text The string in char format
    ///
    String(
            const char *text);

    ///
    /// \brief Construct string
    /// \param text The string in string format
    ///
    String(
            const biorbd::utils::String& text);

    ///
    /// \brief Construct string
    /// \param text The string in vector char format
    ///
    String(
            const std::basic_string<char>& text);

    ///
    /// \brief Allow to use operator=
    /// \param other The other string
    ///
    String& operator=(
            const biorbd::utils::String& other);

    ///
    /// \brief Append an unsigned int to the string
    /// \param val The unsinged int to add
    ///
    String operator+(
            unsigned int val);

    ///
    /// \brief Append an int to the string
    /// \param val The int to add
    ///
    String operator+(
            int val);

    ///
    /// \brief Append a double to the string
    /// \param val The double to add
    ///
    String operator+(
            double val);

    ///
    /// \brief Append a char to the string
    /// \param text The char to add
    ///
    String operator+(
            const char* text);

    ///
    /// \brief Get a specific character in the string
    /// \param idx The index of the character
    ///
    String operator()(
            unsigned int idx) const;

    ///
    /// \brief Extract a portion of the string
    /// \param startIdx The index of the first character
    /// \param lastIdx The index of the last character
    ///
    String operator()(
            unsigned int startIdx,
            unsigned int lastIdx) const;

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
/// \brief To use operator<<
/// \param os The ostream
/// \param a The string to operate on
///
std::ostream& operator<<(std::ostream& os, const biorbd::utils::String &a);

#endif // BIORBD_UTILS_STRING_H
