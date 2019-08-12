#ifndef BIORBD_UTILS_WRITER_H
#define BIORBD_UTILS_WRITER_H

#include "biorbdConfig.h"

class s2mMusculoSkeletalModel;

namespace biorbd { namespace utils {
class Path;

class BIORBD_API Writer
{
public:
    static void writeModel(
            s2mMusculoSkeletalModel &,
            const biorbd::utils::Path& pathToWrite);

};

}}

#endif // BIORBD_UTILS_WRITER_H
