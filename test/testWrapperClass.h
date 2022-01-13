#ifndef TEST_WRAPPER_CLASS_H
#define TEST_WRAPPER_CLASS_H

#include "RigidBody/Joints.h"

namespace BIORBD_NAMESPACE
{
class WrapperJoints : public rigidbody::Joints {

public:
     std::vector<RigidBodyDynamics::Math::SpatialVector> * wrapCombineExtForceAndSoftContact(
            std::vector<utils::SpatialVector> *f_ext,
            const rigidbody::GeneralizedCoordinates& Q,
            const rigidbody::GeneralizedVelocity& QDot)
    {
        return this->Joints::combineExtForceAndSoftContact(f_ext, Q, QDot);
    };
};

}
#endif
