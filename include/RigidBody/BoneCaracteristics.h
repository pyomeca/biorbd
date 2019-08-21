#ifndef BIORBD_RIGIDBODY_BONE_CARACTERISTICS_H
#define BIORBD_RIGIDBODY_BONE_CARACTERISTICS_H

#include <memory>
#include <rbdl/Body.h>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Node3d;
}

namespace rigidbody {
class BoneMesh;

class BIORBD_API Caracteristics : public RigidBodyDynamics::Body
{
public:
    Caracteristics();
    Caracteristics(
            double mass, // Mass of the body
            const biorbd::utils::Node3d &com, // Center of Mass
            const RigidBodyDynamics::Math::Matrix3d &inertia); // Inertia matrix
    Caracteristics(
            double mass, // Mass of the body
            const biorbd::utils::Node3d &com, // Center of Mass
            const RigidBodyDynamics::Math::Matrix3d &inertia, // Inertia matrix
            const biorbd::rigidbody::BoneMesh &mesh); // position des meshings de l'os

    // Set and Get
    double length() const;
    double mass() const;
    void setLength(double val);
    const biorbd::rigidbody::BoneMesh& mesh() const;
    const Eigen::Matrix3d& inertia() const;

protected:
    double m_length;
    std::shared_ptr<biorbd::rigidbody::BoneMesh> m_mesh;
};

}}

#endif // BIORBD_RIGIDBODY_BONE_CARACTERISTICS_H
