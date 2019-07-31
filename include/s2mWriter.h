#ifndef S2M_WRITER_H
#define S2M_WRITER_H

#include "biorbdConfig.h"
#include "s2mMusculoSkeletalModel.h"
#include <iostream>
#include <fstream>

class BIORBD_API s2mWriter 
{
    public:
        static void writeModel(s2mMusculoSkeletalModel &, const s2mPath& pathToWrite);

    protected:
};

#endif // S2M_WRITER_H
