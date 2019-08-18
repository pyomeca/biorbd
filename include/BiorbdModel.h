#ifndef BIORBD_MODEL_H
#define BIORBD_MODEL_H

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

namespace biorbd {

class BIORBD_API Model :
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
    Model();
    virtual ~Model();
    Model(const biorbd::utils::Path&);

    bool InverseKinematics(
            const std::vector<biorbd::rigidbody::NodeBone>& Mark,
            const biorbd::rigidbody::GeneralizedCoordinates& Qinit,
            biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxes=true);

};

}

#endif // BIORBD_MODEL_H
