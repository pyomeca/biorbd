#ifndef BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H
#define BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H

#include <memory>
#include <rbdl/Body.h>
#include "biorbdConfig.h"

///
/// \brief Namespace biorbd
///
namespace biorbd {

namespace utils {
class Vector3d;
}

///
/// \brief Namespace rigidbody that holds the segment characteristics and mesh
///
namespace rigidbody {
class Mesh;

///
/// \brief Class SegmentCharacteristics
///
class BIORBD_API SegmentCharacteristics : public RigidBodyDynamics::Body
{
public:


///
/// \brief Construct segment characteristics
///
    SegmentCharacteristics();

///
/// \brief Get segment characteristics
/// \param mass Mass of the body
/// \param com Center of mass
/// \param inertia Inertia matrix
///
    SegmentCharacteristics(
            double mass, 
            const biorbd::utils::Vector3d &com, 
            const RigidBodyDynamics::Math::Matrix3d &inertia); 


    ///
    /// \brief Get the segment characteristics
    /// \param mass Mass of the body
    /// \param com Center of Mass
    /// \param inertia Inertia matrix
    /// \param mesh Position of the meshing
    ///
    SegmentCharacteristics(
            double mass, 
            const biorbd::utils::Vector3d &com, 
            const RigidBodyDynamics::Math::Matrix3d &inertia, 
            const biorbd::rigidbody::Mesh &mesh); 

    ///
    /// \brief Deep copy of the segment characteristics
    /// \return Copy of the segment characteristics
    ///
    biorbd::rigidbody::SegmentCharacteristics DeepCopy() const;

    ///
    /// \brief Copy the segment characteristics
    /// \param other The characteristics to copy
    ///
    void DeepCopy(const biorbd::rigidbody::SegmentCharacteristics& other);


    // Set and Get
    ///
    /// \brief Returns the segment length
    /// \return The segment length
    ///
    double length() const;

    ///
    /// \brief Returns the segment mass
    /// \return The segment mass
    ///
    double mass() const;

    ///
    /// \brief Set the segment length
    /// \param val Value of the new length
    ///
    void setLength(double val);

    ///
    /// \brief Returns the segment mesh
    /// \return The segment mesh
    ///
    const biorbd::rigidbody::Mesh& mesh() const;

    ///
    /// \brief Returns the segment inertia
    /// \return The segment inertia
    ///
    const Eigen::Matrix3d& inertia() const;

protected:
    std::shared_ptr<double> m_length; ///< Length of the segment
    std::shared_ptr<biorbd::rigidbody::Mesh> m_mesh; ///< Mesh of the segment
};

}}

#endif // BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H
