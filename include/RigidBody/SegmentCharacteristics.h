#ifndef BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H
#define BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H

#include <memory>
#include <rbdl/Body.h>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Vector3d;
}

namespace rigidbody {
class Mesh;

///
/// \brief Characteristics of a segment, namely the mass, the center of mass,
/// the inertia, the length and its mesh geometry
///
class BIORBD_API SegmentCharacteristics : public RigidBodyDynamics::Body
{
public:
    ///
    /// \brief Construct segment characteristics
    ///
    SegmentCharacteristics();

    ///
    /// \brief Construct segment characteristics
    /// \param mass The mass of the segment
    /// \param com The position of the center of Mass
    /// \param inertia The inertia matrix
    ///
    SegmentCharacteristics(
            double mass, 
            const biorbd::utils::Vector3d &com, 
            const RigidBodyDynamics::Math::Matrix3d &inertia); 

    ///
    /// \brief Construct segment characteristics
    /// \param mass The mass of the segment
    /// \param com The position of the center of Mass
    /// \param inertia The inertia matrix
    /// \param mesh The mesh geometry of the segment
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

    ///
    /// \brief Set the segment length
    /// \param val Value of the new length
    ///
    void setLength(double val);

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
    /// \brief Returns the segment mesh
    /// \return The segment mesh
    ///
    const biorbd::rigidbody::Mesh& mesh() const;

    ///
    /// \brief Returns the segment inertia matrix
    /// \return The segment inertia matrix
    ///
    const Eigen::Matrix3d& inertia() const;

protected:
    std::shared_ptr<double> m_length; ///< Length of the segment
    std::shared_ptr<biorbd::rigidbody::Mesh> m_mesh; ///< Mesh of the segment
};

}}

#endif // BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H
