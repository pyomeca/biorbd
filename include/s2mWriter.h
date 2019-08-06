#ifndef S2MWRITER_H
#define S2MWRITER_H

#include <iostream>
#include <fstream>
#include "biorbdConfig.h"
#include "s2mPath.h"

class s2mMusculoSkeletalModel;
class BIORBD_API s2mWriter 
{
    public:
        static void writeModel(s2mMusculoSkeletalModel &, const s2mPath& pathToWrite);

    protected:
};

#include "s2mMusculoSkeletalModel.h"

#endif // S2MWRITER_H
