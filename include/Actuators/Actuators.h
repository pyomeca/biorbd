#ifndef BIORBD_ACTUATORS_ACTUATORS_H
#define BIORBD_ACTUATORS_ACTUATORS_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace utils {
class GenCoord;
class Tau;
}

namespace rigidbody {
class Joints;
}

namespace actuator {

class BIORBD_API Actuators
{
public:
    Actuators();
    Actuators(const Actuators&);
    virtual ~Actuators();

    void addActuator(const biorbd::rigidbody::Joints&, Actuator &a);
    void closeActuator(biorbd::rigidbody::Joints& m);

    // Retourne deux vecteur de torque max (il manque l'entrée de puissance afin de savoir si c'est positif ou négatif
    // à chaque articulation, donc les deux sont retournés)
    std::pair<biorbd::utils::Tau, biorbd::utils::Tau> torqueMax(
            const biorbd::rigidbody::Joints&,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord& Qdot);
    biorbd::utils::Tau torqueMax(
            const biorbd::rigidbody::Joints&,
            const biorbd::utils::GenCoord& a,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord &Qdot);
    biorbd::utils::Tau torque(
            const biorbd::rigidbody::Joints&,
            const biorbd::utils::GenCoord& a,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord &Qdot);

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
