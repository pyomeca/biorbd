#ifndef BIORBD_ACTUATORS_ACTUATORS_H
#define BIORBD_ACTUATORS_ACTUATORS_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"

namespace biorbd {

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
}

namespace actuator {
class Actuator;

class BIORBD_API Actuators
{
public:
    Actuators();
    Actuators(
            const biorbd::actuator::Actuators& other);
    virtual ~Actuators();

    void addActuator(
            const biorbd::actuator::Actuator &a);
    void closeActuator();

    // Retourne deux vecteur de torque max (il manque l'entrée de puissance afin de savoir si c'est positif ou négatif
    // à chaque articulation, donc les deux sont retournés)
    std::pair<biorbd::rigidbody::GeneralizedTorque, biorbd::rigidbody::GeneralizedTorque> torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot);
    biorbd::rigidbody::GeneralizedTorque torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates& a,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot);
    biorbd::rigidbody::GeneralizedTorque torque(
            const biorbd::rigidbody::GeneralizedCoordinates& a,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot);

    // Get and set
    const std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>& actuator(unsigned int dof);
    const biorbd::actuator::Actuator& actuator(unsigned int dof, unsigned int idx);
    unsigned int nbActuators() const;

protected:
    std::shared_ptr<std::vector<std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>>> m_all; // Tous les actuators réunis / pair (+ ou -)
    std::shared_ptr<std::vector<bool>> m_isDofSet;
    std::shared_ptr<bool> m_isClose;

};

}}

#endif // BIORBD_ACTUATORS_ACTUATORS_H
