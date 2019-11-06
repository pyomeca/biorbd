#ifndef BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H
#define BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H

#include "biorbdConfig.h"
#include "Muscles/Muscle.h"

namespace biorbd {
namespace muscles {

class BIORBD_API IdealizedActuator : public biorbd::muscles::Muscle
{
public:
    IdealizedActuator();
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics);
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::StateDynamics& dynamicState);
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers);
    IdealizedActuator(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState);
    IdealizedActuator(
            const biorbd::muscles::Muscle& muscle);
    IdealizedActuator(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);
    biorbd::muscles::IdealizedActuator DeepCopy() const;
    void DeepCopy(const biorbd::muscles::IdealizedActuator& other);

    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            const biorbd::muscles::StateDynamics& emg);
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);
protected:
    virtual double getForceFromActivation(
            const biorbd::muscles::State &emg); // Voir dans la fonction pour descriptif
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_IDEALIZED_ACTUATOR_H
