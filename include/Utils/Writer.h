#ifndef BIORBD_UTILS_WRITER_H
#define BIORBD_UTILS_WRITER_H

#include "biorbdConfig.h"

namespace biorbd {
class Model;

namespace utils {
class Path;

class BIORBD_API Writer
{
public:
    static void writeModel(
            biorbd::Model &,
            const biorbd::utils::Path& pathToWrite);

};

}}

#endif // BIORBD_UTILS_WRITER_H
