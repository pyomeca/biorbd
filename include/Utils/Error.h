#ifndef BIORBD_UTILS_ERROR_H
#define BIORBD_UTILS_ERROR_H

#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace utils {
class String;

class BIORBD_API Error
{
public:
    /// 
    /// \brief Throw a error message
    /// \param The error message to display
    ///
    [[noreturn]] static void raise(
            const biorbd::utils::String &message);
    ///
    /// \brief Check condition, if false, throw an error message
    /// \param cond The condition (true or false)
    /// \param message The message to display
    ///
    static void check(
            bool cond,
            const biorbd::utils::String &message);

    ///
    /// \brief Display a warning message if condition is false
    /// \param cond The condition (true of false)
    /// \message The warning message to display
    ///
        static void warning(
            bool cond,
            const biorbd::utils::String &message);
};

}}

#endif // BIORBD_UTILS_ERROR_H


