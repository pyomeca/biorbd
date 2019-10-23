#ifndef BIORBD_RIGIDBODY_BONE_CHARACTERISTICS_H
#define BIORBD_RIGIDBODY_BONE_CHARACTERISTICS_H

#include <memory>
#include <rbdl/Body.h>
#include "biorbdConfig.h"

///
/// \brief Namespace biorbd
///
namespace biorbd {

namespace utils {
class Node3d;
}

///
/// \brief Namespace rigidbody that holds the bone caracteristics and mesh
///
namespace rigidbody {
class BoneMesh;

///
/// \brief Class BoneCharacteristics
///
class BIORBD_API BoneCharacteristics : public RigidBodyDynamics::Body
{
public:


///
/// \brief Construct bone characteristics
///
    BoneCharacteristics();

///
/// \brief Get bone characteristics
/// \param mass Mass of the body
/// \param com Center of mass
/// \param inertia Inertia matrix
///
    BoneCharacteristics(
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
    BoneCharacteristics(
            double mass, 
            const biorbd::utils::Node3d &com, 
            const RigidBodyDynamics::Math::Matrix3d &inertia, 
            const biorbd::rigidbody::BoneMesh &mesh); 

    ///
    /// \brief Deep copy of the bone characteristics
    /// \return Copy of the bone characteristics
    ///
    biorbd::rigidbody::BoneCharacteristics DeepCopy() const;

    ///
    /// \brief Copy the bone characteristics
    /// \param other The characteristics to copy
    ///
    void DeepCopy(const biorbd::rigidbody::BoneCharacteristics& other);


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
