#ifndef S2MMUSCULOSKELETALMODEL_H
#define S2MMUSCULOSKELETALMODEL_H
//#define S2M_MUSCLE_OPTIMIZATION

	#include "s2mOptions.h"
    #include "s2mJoints.h"
    #include "s2mMarkers.h"
    #include "s2mIMUs.h"
    #include "s2mContacts.h"
    #include "s2mMuscles.h"
    #ifdef S2M_MUSCLE_OPTIMIZATION
        #include "s2mMuscleOptimisation.h"
    #endif
    #include "s2mActuators.h"
    #include "s2mBenchmark.h"

class s2mMusculoSkeletalModel :
        public s2mJoints
        ,public s2mMarkers
        ,public s2mIMUs
        ,public s2mMuscles
        ,public s2mContacts
        ,public s2mActuators
        #ifdef S2M_MUSCLE_OPTIMIZATION
            ,public s2mMuscleOptimisation
        #endif
        
{
    public:
        s2mMusculoSkeletalModel(){}
        ~s2mMusculoSkeletalModel(){}



        bool InverseKinematics(const std::vector<Eigen::Vector3d>& Mark, const s2mGenCoord& Qinit, s2mGenCoord &Q, bool removeAxes=true);

        // Muscles

        // Set and Get
    protected:

    private:

};

#include "s2mRead.h"
#include "s2mWriter.h"

#endif // S2MMUSCULOSKELETALMODEL_H
