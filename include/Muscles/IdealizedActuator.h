#ifndef BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H
#define BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H

#include "biorbdConfig.h"
#include "Muscles/Muscle.h"

namespace biorbd {
namespace muscles {

    ///
    /// \brief Class IdealizedActuator
    ///
class BIORBD_API IdealizedActuator : public biorbd::muscles::Muscle
{
public:
    ///
    /// \brief Contruct an idealized actuator
    ///
    IdealizedActuator();

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics);

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param dynamicState The muscle dynamic state
    ///
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::StateDynamics& dynamicState);

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The path changers
    ///
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers);

    ///
    /// \brief Construct an idealized actuator
    /// \param name The name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The mupath changers
    /// \param dynamicState The dynamic state
    ///
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState);

    ///
    /// \brief Construct an idealized actuator from a muscle
    /// \param muscle The muscle
    ///
    IdealizedActuator(
            const biorbd::muscles::Muscle& muscle);

    ///
    /// \brief Construct an idealized actuator from a muscle
    /// \param muscle The muscle (pointer)
    ///
    IdealizedActuator(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);

    ///
    /// \brief Deep copy of an idealized actuator
    /// \return A deep copy of an idealized actuator
    ///
    biorbd::muscles::IdealizedActuator DeepCopy() const;

    ///
    /// \brief Deep copy of an idealized actuator from another idealized actuator
    /// \param other The idealized actuator to copy
    ///
    void DeepCopy(const biorbd::muscles::IdealizedActuator& other);

    ///
    /// \brief Return the force 
    /// \param emg EMG data
    /// \return The force
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            const biorbd::muscles::StateDynamics& emg);

    ///
    /// \brief Return the force
    /// \param model The model
    /// \param Q The position variables
    /// \param Qdot The velocity variables
    /// \param emg THe EMG data
    /// \param updateKin Update kinematics (default: 2)
    /// \return The force
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);
    ///
    /// \brief Return the force
    /// \param model The model
    /// \param Q The position variables
    /// \param emg THe EMG data
    /// \param updateKin Update kinematics (default: 2)
    /// \return The force
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);
protected:
    ///
    /// \brief Return the force from activation
    /// \return The force from activation
    ///
    virtual double getForceFromActivation(
            const biorbd::muscles::State &emg);

    ///
    /// \brief Set the type to Idealized_actuator
    ///
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H
