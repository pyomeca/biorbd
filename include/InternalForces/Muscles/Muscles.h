#ifndef BIORBD_MUSCLES_MUSCLES_H
#define BIORBD_MUSCLES_MUSCLES_H

#include <vector>
#include <memory>

#include "biorbdConfig.h"
#include "InternalForces/Geometry.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
class Matrix;
class Vector;
class Vector3d;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedTorque;
}

namespace internal_forces
{
namespace muscles
{
class MuscleGroup;
class State;
class Muscle;

///
/// \brief Muscle group holder
///
class BIORBD_API Muscles
{
public:
    ///
    /// \brief Construct muscles
    ///
    Muscles();

    ///
    /// \brief Construct muscles from other muscles
    /// \param other The other muscles
    ///
    Muscles(const Muscles& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Muscles();

    ///
    /// \brief Deep copy of muscles
    /// \return A deep copy of muscles
    ///
    Muscles DeepCopy() const;

    ///
    /// \brief Deep copy of muscles into another mucles
    /// \param other The muscles to copy
    ///
    void DeepCopy(
        const Muscles& other);

    ///
    /// \brief Add a muscle group to the set
    /// \param name The name of the muscle group
    /// \param originName The origin segment name where the origin lies
    /// \param insertionName The insertion segment name where the origin lies
    ///
    void addMuscleGroup(
        const utils::String &name,
        const utils::String &originName,
        const utils::String &insertionName);

    ///
    /// \brief Return the group ID
    /// \param name The name of the muscle group
    /// \return The group ID (returns -1 if not found)
    ///
    int getMuscleGroupId(
        const utils::String &name) const;


    ///
    /// \brief Returns all the muscles. It sorts the muscles by group
    /// \return All the muscle
    ///
    std::vector<std::shared_ptr<Muscle>> muscles() const;

    ///
    /// \brief Returns a specific muscle sorted by muscles()
    /// \param idx The muscle index
    /// \return The muscle
    ///
    Muscle& muscle(
        size_t idx) const;

    ///
    /// \brief muscleNames Return the names for all the muscle ordered by their
    /// respective group name
    /// \return All the muscle names
    ///
    std::vector<utils::String> muscleNames() const;

    ///
    /// \brief Return the muscle groups
    /// \return The muscle groups
    ///
    std::vector<MuscleGroup>& muscleGroups();

    ///
    /// \brief Return the muscle groups
    /// \return The muscle groups
    ///
    const std::vector<MuscleGroup>& muscleGroups() const;

    ///
    /// \brief Return the muscle group of specific index
    /// \param idx The index of the muscle group to return
    /// \return A muscle group
    ///
    MuscleGroup& muscleGroup(
        size_t idx);

    ///
    /// \brief Return the muscle group of specific index
    /// \param idx The index of the muscle group to return
    /// \return A muscle group
    ///
    const MuscleGroup& muscleGroup(
        size_t idx) const;

    ///
    /// \brief Return the muscle group of specific name
    /// \param name The name of the muscle group to return
    /// \return A muscle group
    ///
    const MuscleGroup& muscleGroup(
        const utils::String& name) const;

    ///
    /// \brief Update all the muscles (positions, jacobian, etc.)
    /// \param updatedModel The model previously updated to proper kinematic level
    /// \param Q The generalized coordinates
    ///
    void updateMuscles(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q);

    ///
    /// \brief Update all the muscles (positions, jacobian, etc.)
    /// \param Q The generalized coordinates
    /// \param updateKin If the joint model should be updated
    ///
    void updateMuscles(
        const rigidbody::GeneralizedCoordinates& Q,
        bool updateKin = true);

    ///
    /// \brief Update all the muscles (positions, jacobian, etc.)
    /// \param updatedModel The model previously updated to proper kinematic level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    ///
    void updateMuscles(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot);

    ///
    /// \brief Update all the muscles (positions, jacobian, etc.)
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the joint model should be updated
    ///
    void updateMuscles(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        bool updateKin = true);

    ///
    /// \brief Update by hand all the muscles (positions, jacobian, velocity, etc.)
    /// \param musclePointsInGlobal The muscle points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    ///
    void updateMuscles(
        std::vector<std::vector<utils::Vector3d>>& musclePointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal);

    ///
    /// \brief Update by hand all the muscles (positions, jacobian, velocity, etc.)
    /// \param musclePointsInGlobal The muscle points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    /// \param Qdot The generalized velocities
    ///
    void updateMuscles(
        std::vector<std::vector<utils::Vector3d>>& musclePointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal,
        const rigidbody::GeneralizedVelocity& Qdot);

    ///
    /// \brief Get the vector of state that must be used to update states
    /// \return The vector of state
    ///
    /// Note: creating your own vector of state is possible. However, it
    /// will override the state type that is associated with the muscle
    ///
    std::vector<std::shared_ptr<State>> stateSet();

    ///
    /// \brief Compute the muscular joint torque
    /// \param F The force vector of all the muscles
    ///
    /// The computation for the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    /// Warning: This function assumes that muscles are already updated (via `updateMuscles`)
    ///
    rigidbody::GeneralizedTorque muscularJointTorque(
        const utils::Vector& F);

    ///
    /// \brief Compute the muscular joint torque
    /// \param F The force vector of all the muscles
    /// \param updatedModel The model previously updated to proper kinematic level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updatedModel The model previously updated to proper kinematic level
    ///
    /// This function updates the muscles and then performs the computation for
    /// the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    rigidbody::GeneralizedTorque muscularJointTorque(
        const utils::Vector& F,
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot, 
        bool updateMuscleParameters = true);

    ///
    /// \brief Compute the muscular joint torque
    /// \param F The force vector of all the muscles
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and muscles])
    ///
    /// This function updates the muscles and then performs the computation for
    /// the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    rigidbody::GeneralizedTorque muscularJointTorque(
        const utils::Vector& F,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot, 
        int updateKin = 2);

    ///
    /// \brief Compute the muscular joint torque
    /// \param emg The dynamic state to compute the force vector
    ///
    /// This functions converts muscle activations into muscle forces and then performs
    /// the computation for the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    /// Warning: This function assumes that muscles are already updated (via `updateMuscles`)
    ///
    rigidbody::GeneralizedTorque muscularJointTorque(
        const std::vector<std::shared_ptr<State>>& emg);

    ///
    /// \brief Compute the muscular joint torque
    /// \param updatedModel The model previously updated to proper kinematic level
    /// \param emg The dynamic state to compute the force vector
    /// \param Q The generalized coordinates (not needed if updateKin is false)
    /// \param Qdot The generalized velocities (not needed if updateKin is false)
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    ///
    /// The computation for the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    rigidbody::GeneralizedTorque muscularJointTorque(
        rigidbody::Joints& updatedModel,
        const std::vector<std::shared_ptr<State>>& emg,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        bool updateMuscleParameters = true);

    ///
    /// \brief Compute the muscular joint torque
    /// \param emg The dynamic state to compute the force vector
    /// \param Q The generalized coordinates (not needed if updateKin is false)
    /// \param Qdot The generalized velocities (not needed if updateKin is false)
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and muscles])
    ///
    /// The computation for the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    rigidbody::GeneralizedTorque muscularJointTorque(
        const std::vector<std::shared_ptr<State>>& emg,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        int updateKin = 2);

    ///
    /// \brief Interface that returns in a vector all the activations dot
    /// \param states The state of the muscle
    /// \param areadyNormalized If the states are already normalized
    /// \return All the activations dot
    ///
    utils::Vector activationDot(
        const std::vector<std::shared_ptr<State>>& states,
        bool areadyNormalized = true);

    ///
    /// \brief Return the previously computed muscle length jacobian
    /// \return The muscle length jacobian
    ///
    utils::Matrix musclesLengthJacobian();

    ///
    /// \brief Compute and return the muscle length Jacobian
    /// \param updatedModel The model previously updated to proper kinematic level
    /// \param Q The generalized coordinates
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The muscle length Jacobian
    ///
    utils::Matrix musclesLengthJacobian(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q, 
        bool updateMuscleParameters = true);

    ///
    /// \brief Compute and return the muscle length Jacobian
    /// \param Q The generalized coordinates
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and muscles])
    /// \return The muscle length Jacobian
    ///
    utils::Matrix musclesLengthJacobian(
        const rigidbody::GeneralizedCoordinates& Q, 
        int updateKin = 2);

    ///
    /// \brief Compute and return the muscle forces
    /// \param emg The dynamic state
    /// \return The muscle forces
    ///
    /// Warning: This function assumes that muscles are already updated (via `updateMuscles`)
    ///
    utils::Vector muscleForces(
        const std::vector<std::shared_ptr<State>>& emg);

    ///
    /// \brief Compute and return the muscle forces
    /// \param updatedModel The model previously updated to proper kinematic level
    /// \param emg The dynamic state
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The muscle forces
    ///
    utils::Vector muscleForces(
        rigidbody::Joints& updatedModel,
        const std::vector<std::shared_ptr<State>>& emg,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot, 
        bool updateMuscleParameters = true);

    ///
    /// \brief Compute and return the muscle forces
    /// \param emg The dynamic state
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and muscles])
    /// \return The muscle forces
    ///
    utils::Vector muscleForces(
        const std::vector<std::shared_ptr<State>>& emg,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot, 
        int updateKin = 2);

    ///
    /// \brief Return the total number of muscle groups
    /// \return The total number of muscle groups
    ///
    size_t nbMuscleGroups() const;

    ///
    /// \brief Return the total number of muscle
    /// \return The total number of muscles
    ///
    size_t nbMuscleTotal() const;

    ///
    /// \brief Return the total number of muscle
    /// \return The total number of muscles
    ///
    size_t nbMuscles() const;

protected:
    std::shared_ptr<std::vector<MuscleGroup>>
            m_mus; ///< Holder for muscle groups
};

}
}
}

#endif // BIORBD_MUSCLES_MUSCLES_H

