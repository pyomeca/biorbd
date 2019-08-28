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
    static void error(
            bool cond,
            const biorbd::utils::String &message);
    static void warning(
            bool cond,
            const biorbd::utils::String &message);
};

}}

#endif // BIORBD_UTILS_ERROR_H


