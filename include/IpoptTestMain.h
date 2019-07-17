#ifndef IPOPTTESTMAIN_H
#define IPOPTTESTMAIN_H

#include <IpIpoptApplication.hpp>
#include "IpTNLP.hpp"
#include "biorbdConfig.h"
#include "s2mGenCoord.h"
#include "s2mMusculoSkeletalModel.h"
#include "s2mMuscles.h"
#include "s2mVector.h"
#include "IpoptTest.h"

#include <Eigen/Dense>


    
class BIORBD_API mainTest
{
    public:
    mainTest(
            s2mMusculoSkeletalModel &m
            );
    ~mainTest();

    int main();


    protected:
    s2mMusculoSkeletalModel &m_model;

    private:

};
#endif // IPOPTTESTMAIN_H
