#ifndef S2MWRITER_H
#define S2MWRITER_H
#include "s2mMusculoSkeletalModel.h"
#include <iostream>
#include <fstream>
//#include <boost/filesystem.hpp>

class s2mWriter 
{
    public:
        static void writeModel(s2mMusculoSkeletalModel &, const s2mPath& pathToWrite);

    protected:
};

#endif // S2MWRITER_H
