#ifndef TEST_WRAPPER_CLASS_H
#define TEST_WRAPPER_CLASS_H

#include "BiorbdModel.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Contacts.h"
#include "RigidBody/SoftContacts.h"

namespace BIORBD_NAMESPACE
{
class WrapperToJointsContactSoftcontact :
        public rigidbody::Joints
        ,public rigidbody::Contacts
        ,public rigidbody::SoftContacts {

public:
    WrapperToJointsContactSoftcontact(const BIORBD_NAMESPACE::Model& m):
        rigidbody::Joints(m),
        rigidbody::Contacts(m),
        rigidbody::SoftContacts(m)
    {}

    std::vector<RigidBodyDynamics::Math::SpatialVector> * wrapCombineExtForceAndSoftContact(
            std::vector<utils::SpatialVector> *f_ext,
            const rigidbody::GeneralizedCoordinates& Q,
            const rigidbody::GeneralizedVelocity& QDot,
            bool updateKin,
            std::vector<utils::Vector> *f_contacts){
        return this->Joints::combineExtForceAndSoftContact(f_ext, Q, QDot, updateKin, f_contacts);
    }
};

}
#endif
