#ifndef BIORBD_ACTUATORS_ACTUATORS_H
#define BIORBD_ACTUATORS_ACTUATORS_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
}

namespace actuator {

class BIORBD_API Actuators
{
public:
    Actuators();
    Actuators(const Actuators&);
    virtual ~Actuators();

    void addActuator(Actuator &a);
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
    virtual std::pair<std::shared_ptr<Actuator>, std::shared_ptr<Actuator>> actuator(unsigned int dof);
    virtual std::shared_ptr<Actuator> actuator(unsigned int dof, unsigned int idx);
    unsigned int nbActuators() const;

protected:
    std::vector<std::pair<std::shared_ptr<Actuator>, std::shared_ptr<Actuator>>> m_all; // Tous les actuators réunis / pair (+ ou -)
    bool * m_isDofSet;
    bool m_isClose;

};

}}

#endif // BIORBD_ACTUATORS_ACTUATORS_H
