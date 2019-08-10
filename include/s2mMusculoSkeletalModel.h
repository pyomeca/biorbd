#ifndef S2M_MUSCULO_SKELETAL_MODEL_H
#define S2M_MUSCULO_SKELETAL_MODEL_H

#include "biorbdConfig.h"
#include "s2mJoints.h"
#include "s2mMarkers.h"
#include "s2mIMUs.h"
#include "s2mMuscles.h"
#include "s2mContacts.h"
#ifdef MODULE_ACTUATORS
#include "Actuators/Actuators.h"
#endif
#include "s2mPath.h"

class BIORBD_API s2mMusculoSkeletalModel :
        public s2mJoints
        ,public s2mMarkers
        ,public s2mIMUs
        ,public s2mMuscles
        ,public s2mContacts
        #ifdef MODULE_ACTUATORS
        ,public biorbd::actuators::Actuators
        #endif
{
    public:
        s2mMusculoSkeletalModel();
        virtual ~s2mMusculoSkeletalModel();
        s2mMusculoSkeletalModel(const s2mPath&);

        bool InverseKinematics(const std::vector<Eigen::Vector3d>& Mark, const s2mGenCoord& Qinit, s2mGenCoord &Q, bool removeAxes=true);

};

#endif // S2M_MUSCULO_SKELETAL_MODEL_H
