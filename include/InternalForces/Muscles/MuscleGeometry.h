#ifndef BIORBD_MUSCLES_MUSCLE_GEOMETRY_H
#define BIORBD_MUSCLES_MUSCLE_GEOMETRY_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"
#include "Utils/Scalar.h"
#include "InternalForces/Geometry.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class Matrix;
class Vector;
class Vector3d;
}

namespace rigidbody
{
class Joints;
class GeneralizedCoordinates;
class GeneralizedVelocity;
}

namespace internal_forces
{
class PathModifiers;

namespace muscles
{
class Characteristics;

///
/// \brief Class muscle geometry of the muscle
///
class BIORBD_API MuscleGeometry: public Geometry
{
public:
    ///
    /// \brief Construct muscle geometry
    ///
    MuscleGeometry();

    ///
    /// \brief Construct muscle geometry
    /// \param origin The origin node of the muscle
    /// \param insertion The insertion node of the muscle
    ///
    MuscleGeometry(
        const utils::Vector3d &origin,
        const utils::Vector3d &insertion);

    ///
    /// \brief Deep copy of a muscle geometry
    /// \return A deep copy of a muscle geometry
    ///
    MuscleGeometry DeepCopy() const;

    ///
    /// \brief Deep copy of a muscle geometry from another muscle geometry
    /// \param other The muscle geometry to copy
    ///
    void DeepCopy(const MuscleGeometry& other);

    ///
    /// \brief Updates the position and dynamic elements of the muscles.
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates of the joints (not needed if updateKinLevel is less than 2)
    /// \param Qdot The generalized velocities of the joints (not needed if updateKinLevel is less than 2)
    ///
    /// updateKinematics MUST be called before retreiving data that are dependent on Q and/or Qdot
    /// 
    /// WARNING, using updateKinematics overrides the previously sent position and jacobian
    ///
    void updateKinematics(
        rigidbody::Joints &updatedModel,
        const Characteristics& characteristics,
        const rigidbody::GeneralizedCoordinates* Q = nullptr,
        const rigidbody::GeneralizedVelocity* Qdot = nullptr);

    ///
    /// \brief Updates the position and dynamic elements of the muscles.
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The path modifiers
    /// \param Q The generalized coordinates of the joints (not needed if updateKinLevel is less than 2)
    /// \param Qdot The generalized velocities of the joints (not needed if updateKinLevel is less than 2)
    ///
    /// updateKinematics MUST be called before retreiving data that are dependent on Q and/or Qdot
    /// 
    /// WARNING, using updateKinematics overrides the previously sent position and jacobian
    ///
    void updateKinematics(
        rigidbody::Joints &updatedModel,
        const Characteristics& characteristics,
        internal_forces::PathModifiers& pathModifiers,
        const rigidbody::GeneralizedCoordinates* Q = nullptr,
        const rigidbody::GeneralizedVelocity* Qdot = nullptr);

    ///
    /// \brief Updates the position and dynamic elements of the muscles by hand.
    /// \param musclePointsInGlobal Position of all the points in global
    /// \param jacoPointsInGlobal Position of all the Jacobian points in global
    /// \param Qdot The generalized velocities of the joints
    ///
    /// updateKinematics MUST be called before retreiving data that are dependent on Q and/or Qdot
    ///
    void updateKinematics(
        std::vector<utils::Vector3d>& musclePointsInGlobal,
        utils::Matrix& jacoPointsInGlobal,
        const Characteristics& characteristics,
        const rigidbody::GeneralizedVelocity* Qdot = nullptr);

    ///
    /// \brief Return the previously computed muscle length
    /// \return The muscle lengh
    ///
    const utils::Scalar& length() const;

    ///
    /// \brief Return the previously computed muscle tendon length
    /// \return The muscle tendon lengh
    ///
    const utils::Scalar& musculoTendonLength() const;

protected:
    ///
    /// \brief Actual function that implements the update of the kinematics
    /// \param Qdot The generalized velocities
    /// \param pathModifiers The set of path modifiers
    ///
    void _updateKinematics(
        const rigidbody::GeneralizedVelocity *Qdot,
        const Characteristics* characteristics,
        internal_forces::PathModifiers* pathModifiers = nullptr);

    ///
    /// \brief Update the kinematics, compute and return the muscle length
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \return The muscle length
    ///
    const utils::Scalar& length(
        const Characteristics* characteristics,
        internal_forces::PathModifiers* pathModifiers = nullptr);

    // Position des nodes dans le repere local
    std::shared_ptr<utils::Scalar> m_muscleLength; ///< length
    std::shared_ptr<utils::Scalar> m_muscleTendonLength; ///< muscle tendon length

};

}
}
}

#endif // BIORBD_MUSCLES_MUSCLE_GEOMETRY_H
