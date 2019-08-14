#ifndef S2M_MUSCULO_SKELETAL_MODEL_H
#define S2M_MUSCULO_SKELETAL_MODEL_H

#include "biorbdConfig.h"
#include "s2mJoints.h"
#include "s2mMarkers.h"
#include "s2mIMUs.h"
#include "s2mContacts.h"
#ifdef MODULE_ACTUATORS
#include "Actuators/Actuators.h"
#endif
#ifdef MODULE_MUSCLES
#include "Muscles/Muscles.h"
#endif
#include "Utils/Path.h"

class BIORBD_API s2mMusculoSkeletalModel :
        public s2mJoints
        ,public s2mMarkers
        ,public s2mIMUs
        ,public s2mContacts
        #ifdef MODULE_ACTUATORS
        ,public biorbd::actuator::Actuators
        #endif
        #ifdef MODULE_MUSCLES
        ,public s2mMuscles
        #endif
{
public:
    s2mMusculoSkeletalModel();
    virtual ~s2mMusculoSkeletalModel();
    s2mMusculoSkeletalModel(const biorbd::utils::Path&);

    bool InverseKinematics(
            const std::vector<s2mNodeBone>& Mark,
            const biorbd::utils::GenCoord& Qinit,
            biorbd::utils::GenCoord &Q,
            bool removeAxes=true);

};

#endif // S2M_MUSCULO_SKELETAL_MODEL_H
