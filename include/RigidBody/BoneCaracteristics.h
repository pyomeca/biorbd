#ifndef BIORBD_RIGIDBODY_BONE_CARACTERISTICS_H
#define BIORBD_RIGIDBODY_BONE_CARACTERISTICS_H

#include <rbdl/Body.h>
#include "biorbdConfig.h"
#include "RigidBody/BoneMesh.h"

namespace biorbd {
namespace rigidbody {

class BIORBD_API Caracteristics : public RigidBodyDynamics::Body
{
public:
    Caracteristics();
    Caracteristics(
            double mass, // Mass of the body
            const biorbd::utils::Node3d &com, // Center of Mass
            const RigidBodyDynamics::Math::Matrix3d &inertia, // Inertia matrix
            const Mesh &mesh = Mesh()) ; // position des meshings de l'os
    virtual ~Caracteristics();

    // Set and Get
    double length() const;
    double mass() const;
    void setLength(double val);
    const Mesh& mesh() const;
    const Eigen::Matrix3d& inertia() const;

protected:
    double m_length;
    Mesh m_mesh;
};

}}

#endif // BIORBD_RIGIDBODY_BONE_CARACTERISTICS_H
