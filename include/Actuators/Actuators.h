#ifndef S2M_ACTUATORS_H
#define S2M_ACTUATORS_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

class s2mJoints;
class s2mGenCoord;
class s2mTau;
namespace biorbd { namespace actuator {
class BIORBD_API Actuators
{
    public:
        Actuators();
        Actuators(const Actuators&);
        virtual ~Actuators();

        void addActuator(const s2mJoints&, Actuator &a);
        void closeActuator(s2mJoints& m);

        // Retourne deux vecteur de torque max (il manque l'entrée de puissance afin de savoir si c'est positif ou négatif
        // à chaque articulation, donc les deux sont retournés)
        std::pair<s2mTau, s2mTau> torqueMax(const s2mJoints&, const s2mGenCoord& Q, const s2mGenCoord &Qdot);
        s2mTau torqueMax(const s2mJoints&, const s2mGenCoord& a, const s2mGenCoord& Q, const s2mGenCoord &Qdot);
        s2mTau torque(const s2mJoints&, const s2mGenCoord& a, const s2mGenCoord& Q, const s2mGenCoord &Qdot);

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

#endif // S2M_ACTUATORS_H
