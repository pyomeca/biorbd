#ifndef S2MWRITER_H
#define S2MWRITER_H

#include "biorbdConfig.h"

class s2mMusculoSkeletalModel;
class s2mPath;
class BIORBD_API s2mWriter 
{
    public:
        static void writeModel(s2mMusculoSkeletalModel &, const s2mPath& pathToWrite);

};

#endif // S2MWRITER_H
