#ifndef S2M_MUSCULO_SKELETAL_MODEL_H
#define S2M_MUSCULO_SKELETAL_MODEL_H

#include "biorbdConfig.h"
#include "Utils/Path.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Markers.h"
#include "RigidBody/IMUs.h"
#include "RigidBody/Contacts.h"
#ifdef MODULE_ACTUATORS
#include "Actuators/Actuators.h"
#endif
#ifdef MODULE_MUSCLES
#include "Muscles/Muscles.h"
#endif

class BIORBD_API s2mMusculoSkeletalModel :
        public biorbd::rigidbody::Joints
        ,public biorbd::rigidbody::Markers
        ,public biorbd::rigidbody::IMUs
        ,public biorbd::rigidbody::Contacts
        #ifdef MODULE_ACTUATORS
        ,public biorbd::actuator::Actuators
        #endif
        #ifdef MODULE_MUSCLES
        ,public biorbd::muscles::Muscles
        #endif
{
public:
    s2mMusculoSkeletalModel();
    virtual ~s2mMusculoSkeletalModel();
    s2mMusculoSkeletalModel(const biorbd::utils::Path&);

    bool InverseKinematics(
            const std::vector<biorbd::rigidbody::NodeBone>& Mark,
            const biorbd::utils::GenCoord& Qinit,
            biorbd::utils::GenCoord &Q,
            bool removeAxes=true);

};

#endif // S2M_MUSCULO_SKELETAL_MODEL_H
