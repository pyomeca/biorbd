#ifndef BIORBD_INTERNAL_FORCES_GEOMETRY_H
#define BIORBD_INTERNAL_FORCES_GEOMETRY_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"
#include "Utils/Scalar.h"

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
class Characteristics;

///
/// \brief Class Geometry of the muscle
///
class BIORBD_API Geometry
{
public:
    ///
    /// \brief Construct geometry
    ///
    Geometry();

    ///
    /// \brief Construct geometry
    /// \param origin The origin node of the muscle
    /// \param insertion The insertion node of the muscle
    ///
    Geometry(
        const utils::Vector3d &origin,
        const utils::Vector3d &insertion);

    ///
    /// \brief Deep copy of a geometry
    /// \return A deep copy of a geometry
    ///
    Geometry DeepCopy() const;

    ///
    /// \brief Deep copy of a geometry from another geometry
    /// \param other The geometry to copy
    ///
    void DeepCopy(
        const Geometry& other);

    ///
    /// \brief Updates the position and dynamic elements of the muscles.
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates of the joints (not needed if updateKin is less than 2)
    /// \param Qdot The generalized velocities of the joints (not needed if updateKin is less than 2)
    ///
    /// updateKinematics MUST be called before retreiving data that are dependent on Q and/or Qdot
    /// 
    /// WARNING If one used updateKinematics providing the jacobian, using updateKinematics overrides
    /// this previously sent position and jacobian
    ///
    void updateKinematics(
        rigidbody::Joints &updatedModel,
        const rigidbody::GeneralizedCoordinates* Q = nullptr,
        const rigidbody::GeneralizedVelocity* Qdot = nullptr);

    ///
    /// \brief Updates the position and dynamic elements of the muscles.
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The path modifiers
    /// \param Q The generalized coordinates of the joints (not needed if updateKin is less than 2)
    /// \param Qdot The generalized velocities of the joints (not needed if updateKin is less than 2)
    ///
    /// updateKinematics MUST be called before retreiving data that are dependent on Q and/or Qdot
    /// 
    /// WARNING If one used updateKinematics providing the jacobian, using updateKinematics overrides
    /// this previously sent position and jacobian
    ///
    void updateKinematics(
        rigidbody::Joints &updatedModel,
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
        const rigidbody::GeneralizedVelocity* Qdot = nullptr);

    ///
    /// \brief Set the origin position in the local reference frame of the muscle
    /// \param position The origin position to set
    ///
    void setOrigin(
        const utils::Vector3d &position);

    ///
    /// \brief Return the origin position
    /// \return The origin position
    ///
    const utils::Vector3d& originInLocal() const;

    ///
    /// \brief Set the insertion position
    /// \param position The insertion position value to set
    ///
    void setInsertionInLocal(
        const utils::Vector3d &position);

    ///
    /// \brief Return the insertion position
    /// \return The insertion position
    ///
    const utils::Vector3d& insertionInLocal() const;

    ///
    /// \brief Return the origin position in the global reference
    /// \return The origin position in the global reference
    ///
    /// Geometry (updateKin) must be computed at least once before calling originInLocal()
    ///
    const utils::Vector3d& originInGlobal() const;

    ///
    /// \brief Return the insertion position in the global reference
    /// \return The insertion position in the global reference
    ///
    /// Geometry (updateKin) must be computed at least once before calling
    ///
    const utils::Vector3d& insertionInGlobal() const;

    ///
    /// \brief Return the muscles points in the global reference
    /// \return The muscle points in the global reference
    ///
    /// Geometry (updateKin) must be computed at least once before calling. Return already computed via points
    ///
    const std::vector<utils::Vector3d>& pointsInGlobal() const;

    ///
    /// \brief Return the previously computed muscle length
    /// \return The muscle lengh
    ///
    const utils::Scalar& length() const;

    ///
    /// \brief Return the previously computed velocity
    /// \return The computed velocity
    ///
    const utils::Scalar& velocity() const;

    ///
    /// \brief Return the previously computed muscle jacobian
    /// \return The muscle jacobian
    ///
    const utils::Matrix& jacobian() const;

    ///
    /// \brief Return the gradient of the jacobian for the origin node
    /// \return The origin's node jacobian
    ///
    utils::Matrix jacobianOrigin() const;

    ///
    /// \brief Return the gradient of the jacobian for the insertion node
    /// \return The insertion's node jacobian
    ///
    utils::Matrix jacobianInsertion() const ;

    ///
    /// \brief Return the gradient of the jacobian for the via specified node
    /// \brief idxViaPoint the index of the via point (0 being the the origin)
    /// \return The via point's node jacobian
    ///
    utils::Matrix jacobian(
        size_t idxViaPoint) const;

    ///
    /// \brief Return the jacobian length of the muscle
    /// \return The jacobian length
    ///
    const utils::Matrix& jacobianLength() const;


protected:
    ///
    /// \brief Actual function that implements the update of the kinematics
    /// \param Qdot The generalized velocities
    /// \param pathModifiers The set of path modifiers
    ///
    void _updateKinematics(
        const rigidbody::GeneralizedVelocity *Qdot,
        internal_forces::PathModifiers* pathModifiers = nullptr);

    ///
    /// \brief Updates the kinematics and return the position of the origin node
    /// \param model The joint model
    /// \param Q The generalized coordinates of the model
    /// \return The origin position in the global reference
    ///
    const utils::Vector3d& originInGlobal(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates &Q);

    ///
    /// \brief Updates the kinematics and return the position of the insertion node
    /// \param model The joint model
    /// \param Q The generalized coordinates of the model
    /// \return The origin position in the global reference
    ///
    const utils::Vector3d& insertionInGlobal(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates &Q);

    ///
    /// \brief Set the muscle points in global
    /// \param ptsInGlobal The new muscle points in global
    ///
    void setPointsInGlobal(
        std::vector<utils::Vector3d>& ptsInGlobal);

    ///
    /// \brief Set the muscle points in global
    /// \param model The joint model
    /// \param Q The generalized coordinates of the model
    /// \param pathModifiers The set of path modifiers
    ///
    void setPointsInGlobal(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        internal_forces::PathModifiers* pathModifiers = nullptr);


    ///
    /// \brief Update the kinematics, compute and return the muscle length
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \return The muscle length
    ///
    const utils::Scalar& length(
        internal_forces::PathModifiers* pathModifiers = nullptr);

    ///
    /// \brief Update the kinematics, compute and return the muscle velocity assuming not via points nor wrapping objects
    /// \param Qdot The generalized velocities
    /// \return The muscle velocity
    ///
    const utils::Scalar& velocity(
        const rigidbody::GeneralizedVelocity &Qdot);

    ///
    /// \brief Set the jacobian dimensions
    /// \param model The joint model
    ///
    void setJacobianDimension(
        rigidbody::Joints &model);

    ///
    /// \brief Force a jacobian computed from the user
    /// \param jaco The jacobian
    ///
    void jacobian(
        const utils::Matrix &jaco);

    ///
    /// \brief Compute the jacobian
    /// \param model The joint model
    /// \param Q The generalize coordinates
    //const Characteristics* characteristics = nullptr,
    ///
    void jacobian(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates &Q);

    ///
    /// \brief Compute the muscle length jacobian
    ///
    void computeJacobianLength();

    // Position des nodes dans le repere local
    std::shared_ptr<utils::Vector3d> m_origin; ///< Origin node
    std::shared_ptr<utils::Vector3d> m_insertion; ///< Insertion node

    std::shared_ptr<utils::Vector3d> m_originInGlobal; ///< Position of the origin in the global reference
    std::shared_ptr<utils::Vector3d> m_insertionInGlobal; ///< Position of the insertion node in the global reference
    std::shared_ptr<std::vector<utils::Vector3d>> m_pointsInGlobal; ///< Position of all the points in the global reference
    std::shared_ptr<std::vector<utils::Vector3d>> m_pointsInLocal; ///< Position of all the points in local
    std::shared_ptr<utils::Matrix> m_jacobian; ///<The jacobian matrix
    std::shared_ptr<utils::Matrix> m_jacobianLength; ///< The muscle length jacobian

    std::shared_ptr<utils::Scalar> m_length; ///< length
    std::shared_ptr<utils::Scalar> m_velocity; ///< Velocity of the muscular elongation

    std::shared_ptr<bool> m_isGeometryComputed; ///< To know if the geometry was computed at least once
    std::shared_ptr<bool> m_isVelocityComputed; ///< To know if the velocity was computed in the last update
    std::shared_ptr<bool> m_posAndJacoWereForced; ///< To know if the override was used on the muscle position and the Jacobian

};

}
}

#endif // BIORBD_INTERNAL_FORCES_GEOMETRY_H
