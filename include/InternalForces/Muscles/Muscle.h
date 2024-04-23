#ifndef BIORBD_MUSCLES_H
#define BIORBD_MUSCLES_H

#include "biorbdConfig.h"
#include "InternalForces/Compound.h"
#include "InternalForces/Muscles/MusclesEnums.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class Matrix;
class Vector3d;
}

namespace internal_forces
{

namespace muscles
{
class Characteristics;
class State;
class Muscles;

///
/// \brief Base class of all muscle
///
class BIORBD_API Muscle : public Compound
{
    friend Muscles;

public:
    ///
    /// \brief Construct a muscle
    ///
    Muscle();

    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    ///
    Muscle(
        const utils::String& name,
        const internal_forces::muscles::MuscleGeometry& position,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    /// \param emg Dynamic state
    ///
    Muscle(
        const utils::String& name,
        const internal_forces::muscles::MuscleGeometry& position,
        const Characteristics& characteristics,
        const State& emg);

    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    /// \param pathModifiers The path modifiers
    ///
    Muscle(
        const utils::String& name,
        const internal_forces::muscles::MuscleGeometry& position,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    /// \param pathModifiers The path modifier
    /// \param emg The dynamic state
    ///
    Muscle(
        const utils::String& name,
        const internal_forces::muscles::MuscleGeometry& position,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        const State& emg);

    ///
    /// \brief Construct a muscle from another muscle
    /// \param other The other muscle
    ///
    Muscle(
        const Muscle& other);

    ///
    /// \brief Construct a muscle from another muscle
    /// \param other The other muscle
    ///
    Muscle(
        const std::shared_ptr<Muscle> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Muscle();

    ///
    /// \brief Deep copy of a muscle in new muscle
    /// \param other The muscle to copy
    ///
    void DeepCopy(
        const Muscle& other);

    // Get and set

    ///
    /// \brief Get the length of the muscle
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The length of the muscle
    ///
    const utils::Scalar& length(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        bool updateMuscleParameters = true);

    ///
    /// \brief Return the musculo tendon length
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The musculo tendon length
    ///
    const utils::Scalar& musculoTendonLength(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        bool updateMuscleParameters = true);

    ///
    /// \brief Return the velocity of the muscle
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The velocity of the muscle
    ///
    const utils::Scalar& velocity(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        bool updateMuscleParameters = true);

    ///
    /// \brief Update the position of the origin and insertion positions of the muscle
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates
    ///
    void updateOrientations(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates &Q);

    ///
    /// \brief Update the position of the origin and insertion nodes of the muscle
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    ///
    void updateOrientations(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot);

    ///
    /// \brief Update by hand the position of the origin and insertion nodes of the muscle
    /// \param musclePointsInGlobal The muscle points
    /// \param jacoPointsInGlobal The jacobian matrix
    ///
    void updateOrientations(
        std::vector<utils::Vector3d>& musclePointsInGlobal,
        utils::Matrix& jacoPointsInGlobal);

    ///
    /// \brief Update by hand the position of the origin and insertion nodes of the muscle
    /// \param musclePointsInGlobal The muscle points
    /// \param jacoPointsInGlobal The Jacobian matrix
    /// \param Qdot The genelized velocities
    ///
    void updateOrientations(
        std::vector<utils::Vector3d>& musclePointsInGlobal,
        utils::Matrix& jacoPointsInGlobal,
        const rigidbody::GeneralizedVelocity &Qdot);

    ///
    /// \brief Set the position of all the points attached to the muscle (0 being the origin)
    /// \param positions New value of the position
    ///
    void setPosition(
        const internal_forces::muscles::MuscleGeometry &positions);

    ///
    /// \brief Return the position of all the points attached to the muscle (0 being the origin)
    /// \return The positions
    ///
    const internal_forces::muscles::MuscleGeometry& position() const;

    ///
    /// \brief Set the muscle characteristics
    /// \param characteristics New value of the muscle characteristics
    ///
    void setCharacteristics(
        const Characteristics &characteristics);

    ///
    /// \brief Return the muscle characteristics
    /// \return The muscle characteristics
    ///
    Characteristics& characteristics() const;

    ///
    /// \brief Return the muscle points in global reference frame
    /// \param updatedModel The joint model
    /// \param Q The generalized coordinates
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The muscle points in global reference frame
    ///
    const std::vector<utils::Vector3d>& musclesPointsInGlobal(
        rigidbody::Joints &updatedModel,
        const rigidbody::GeneralizedCoordinates &Q, 
        bool updateMuscleParameters = true);

    ///
    /// \brief Return the previously computed muscle points in global reference frame
    /// \return The muscle points in global reference frame
    ///
    const std::vector<utils::Vector3d>& musclesPointsInGlobal() const;

    ///
    /// \brief Set the maximal isometric force
    /// \param forceMax The force to set
    ///
    void setForceIsoMax(
        const utils::Scalar& forceMax);

    ///
    /// \brief Set the dynamic state
    /// \param emg The dynamic state value
    ///
    void setState(
        const State &emg);


    ///
    /// \brief Return the dynamic state
    /// \return The dynamic state
    ///
    const State& state() const;

    ///
    /// \brief Return the dynamic state
    /// \return The dynamic state
    ///
    State& state();

    ///
    /// \brief Return the activation time derivative
    /// \param state The dynamic state
    /// \param alreadyNormalized If the emg is already normalized
    /// \return The activation time derivative
    ///
    const utils::Scalar& activationDot(
        const State& state,
        bool alreadyNormalized = false) const;

    ///
    /// \brief Computes and returns the forces norm from the EMG
    /// \param emg EMG data
    /// \return The computed forces from the EMG
    ///
    virtual const utils::Scalar& force(
        const State& emg) = 0;

    ///
    /// \brief Return the computed force norm from EMG
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates of the model
    /// \param Qdot The generalized velocities of the model
    /// \param emg EMG data
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The computed force from EMG
    ///
    virtual const utils::Scalar& force(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        const State& emg,
        bool updateMuscleParameters = true) = 0;

    ///
    /// \brief Return the computed force norm from EMG
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates of the model
    /// \param emg EMG data
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The computed force from EMG
    ///
    virtual const utils::Scalar& force(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const State& emg,
        bool updateMuscleParameters = true) = 0;

    ///
    /// \brief Return the type of the muscle
    /// \return The type of the muscle
    ///
    MUSCLE_TYPE type() const;

protected:
    ///
    /// \brief Computer the forces from a specific emg
    /// \param emg EMG data
    ///
    virtual void computeForce(
        const State &emg);

    ///
    /// \brief Function allowing modification of the way the multiplication is done in computeForce(EMG)
    /// \param emg The EMG data
    /// \return The force from activation
    ///
    virtual utils::Scalar getForceFromActivation(const
            State &emg) = 0;

    ///
    /// \brief Set the type of muscle
    ///
    virtual void setType();

    std::shared_ptr<MUSCLE_TYPE> m_type; ///< The type of the muscle
    std::shared_ptr<internal_forces::muscles::MuscleGeometry> m_position;
    std::shared_ptr<Characteristics> m_characteristics; ///< The muscle characteristics
    std::shared_ptr<State> m_state; ///< The dynamic state
    std::shared_ptr<utils::Scalar> m_muscleLength; ///< muscle tendon length
};

}
}
}

#endif // BIORBD_MUSCLES_H
