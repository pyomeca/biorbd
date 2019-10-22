#ifndef BIORBD_RIGIDBODY_BONE_CHARACTERISTICS_H
#define BIORBD_RIGIDBODY_BONE_CHARACTERISTICS_H

#include <memory>
#include <rbdl/Body.h>
#include "biorbdConfig.h"

///
/// \brief Namespace biorbd
///
namespace biorbd {
///
/// \brief Namespace utils
///
namespace utils {
class Node3d;
}

///
/// \brief Namespace rigidbody that holds the bone caracteristics and mesh
///
namespace rigidbody {
class BoneMesh;

class BIORBD_API BoneCharacteristics : public RigidBodyDynamics::Body
{
public:
<<<<<<< HEAD:include/RigidBody/BoneCaracteristics.h
    ///
    /// \brief Get bone characteristics
    ///
    BoneCaracteristics();

    ///
    /// \brief Get bone characteristics
    /// \param mass Mass of the body
    /// \param Center of mass
    /// \param inertia Inertia matrix
    ///
    BoneCaracteristics(
            double mass, 
            const biorbd::utils::Node3d &com, 
            const RigidBodyDynamics::Math::Matrix3d &inertia);

    ///
    /// \brief Get the bone characteristics
    /// \param mass Mass of the body
    /// \param com Center of Mass
    /// \param inertia Inertia matrix
    /// \param mesh Position of the bone meshing
    ///
    BoneCaracteristics(
            double mass, 
            const biorbd::utils::Node3d &com, 
            const RigidBodyDynamics::Math::Matrix3d &inertia, 
            const biorbd::rigidbody::BoneMesh &mesh); 

    ///
    /// \brief TODO: Copy the bone characteristics
    /// \return Copy of the bone characteristics
    ///
    biorbd::rigidbody::BoneCaracteristics DeepCopy() const;

    ///
    /// \brief Copy the bone characteristics
    /// \param other TODO: ?
    ///

    void DeepCopy(const biorbd::rigidbody::BoneCaracteristics& other);
=======
    BoneCharacteristics();
    BoneCharacteristics(
            double mass, // Mass of the body
            const biorbd::utils::Node3d &com, // Center of Mass
            const RigidBodyDynamics::Math::Matrix3d &inertia); // Inertia matrix
    BoneCharacteristics(
            double mass, // Mass of the body
            const biorbd::utils::Node3d &com, // Center of Mass
            const RigidBodyDynamics::Math::Matrix3d &inertia, // Inertia matrix
            const biorbd::rigidbody::BoneMesh &mesh); // position des meshings de l'os
    biorbd::rigidbody::BoneCharacteristics DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::BoneCharacteristics& other);
>>>>>>> master:include/RigidBody/BoneCharacteristics.h

    // Set and Get
    ///
    /// \brief Returns the bone length
    /// \return The bone length
    ///
    double length() const;

    ///
    /// \brief Returns the bone mass
    /// \return The bone mass
    ///
    double mass() const;

    ///
    /// \brief Set the bone length
    /// \param val Value of the new length
    ///
    void setLength(double val);

    ///
    /// \brief Returns the bone mesh
    /// \return The bone mesh
    ///
    const biorbd::rigidbody::BoneMesh& mesh() const;

    ///
    /// \brief Returns the bone inertia
    /// \return The bone inertia
    ///
    const Eigen::Matrix3d& inertia() const;

protected:
    std::shared_ptr<double> m_length; ///< Length of the bone
    std::shared_ptr<biorbd::rigidbody::BoneMesh> m_mesh; ///< Mesh of the bone
};

}}

#endif // BIORBD_RIGIDBODY_BONE_CHARACTERISTICS_H