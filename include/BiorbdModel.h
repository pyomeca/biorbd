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

    ///
    /// \brief Construct a model from a bioMod file
    /// \param path The path of the file
    ///
    Model(const biorbd::utils::Path& path);

    ///
    /// \brief Applies inverse kinematics
    /// \param marker Markers
    /// \param Qinit Initial guess for the state
    /// \param Q State vector of the internal joints
    /// \param removeAxes
    /// 
    bool InverseKinematics(
            const std::vector<biorbd::rigidbody::NodeBone>& markers,
            const biorbd::rigidbody::GeneralizedCoordinates& Qinit,
            biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxes=true);

};

}

#endif // BIORBD_MODEL_H
