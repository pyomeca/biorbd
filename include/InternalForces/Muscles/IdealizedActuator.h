#ifndef BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H
#define BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H

#include "biorbdConfig.h"
#include "InternalForces/Muscles/Muscle.h"

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
namespace muscles
{

///
/// \brief Muscle that has a constant maximal force
///
class BIORBD_API IdealizedActuator : public Muscle
{
public:
    ///
    /// \brief Contruct an idealized actuator
    ///
    IdealizedActuator();

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name of the muscle
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    IdealizedActuator(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics);

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name of the muscle
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    ///
    IdealizedActuator(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const State& emg);

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name of the muscle
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    IdealizedActuator(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name of the muscle
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The dynamic state
    ///
    IdealizedActuator(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        const State& emg);

    ///
    /// \brief Construct an idealized actuator from another muscle
    /// \param other The other muscle
    ///
    IdealizedActuator(
        const Muscle& other);

    ///
    /// \brief Construct an idealized actuator from another muscle
    /// \param other The other muscle
    ///
    IdealizedActuator(
        const std::shared_ptr<Muscle> other);

    ///
    /// \brief Deep copy of an idealized actuator
    /// \return A deep copy of an idealized actuator
    ///
    IdealizedActuator DeepCopy() const;

    ///
    /// \brief Deep copy of an idealized actuator from another idealized actuator
    /// \param other The idealized actuator to copy
    ///
    void DeepCopy(const IdealizedActuator& other);

    ///
    /// \brief Return the force
    /// \param emg EMG data
    /// \return The force
    ///
    virtual const utils::Scalar& force(
        const State& emg);

    ///
    /// \brief Return the muscle force vector at origin and insertion
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param emg The EMG data
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The force vector at origin and insertion
    ///
    virtual const utils::Scalar& force(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        const State& emg,
        bool updateMuscleParameters = true);

    ///
    /// \brief Return the muscle force vector at origin and insertion
    /// \param updatedModel The joint model previously updated to the proper level
    /// \param Q The generalized coordinates
    /// \param emg The EMG data
    /// \param updateMuscleParameters Update the kinematic related parameters of the muscles
    /// \return The force vector at origin and insertion
    ///
    virtual const utils::Scalar& force(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        const State& emg,
        bool updateMuscleParameters = true);
protected:
    ///
    /// \brief Function allowing modification of the way the multiplication is done in computeForce(EMG)
    /// \param emg The EMG data
    /// \return The force from activation
    virtual utils::Scalar getForceFromActivation(
        const State &emg);

    ///
    /// \brief Set the type to Idealized_actuator
    ///
    virtual void setType();

};

}
}
}

#endif // BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H
