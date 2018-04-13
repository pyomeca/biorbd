#ifndef S2MACTUATORS_H
#define S2MACTUATORS_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include "s2mError.h"
#include "s2mActuator.h"
#include "s2mActuatorGauss3p.h"
#include "s2mActuatorGauss6p.h"
#include "s2mActuatorConstant.h"
#include "s2mActuatorLinear.h"
#include "s2mTau.h"

class s2mActuators
{
    public:
        s2mActuators();
        s2mActuators(const s2mActuators&);
        ~s2mActuators();

        void addActuator(const s2mJoints&, s2mActuator &a);
        void closeActuator(s2mJoints& m);

        // Retourne deux vecteur de torque max (il manque l'entrée de puissance afin de savoir si c'est positif ou négatif
        // à chaque articulation, donc les deux sont retournés)
        std::pair<s2mTau, s2mTau> torqueMax(const s2mJoints&, const s2mGenCoord& Q, const s2mGenCoord &Qdot);
        s2mTau torqueMax(const s2mJoints&, const s2mGenCoord& a, const s2mGenCoord& Q, const s2mGenCoord &Qdot);
        s2mTau torque(const s2mJoints&, const s2mGenCoord& a, const s2mGenCoord& Q, const s2mGenCoord &Qdot);

        // Get and set
        virtual std::pair<boost::shared_ptr<s2mActuator>, boost::shared_ptr<s2mActuator> > actuator(unsigned int dof);
        virtual boost::shared_ptr<s2mActuator> actuator(unsigned int dof, unsigned int idx);
        unsigned int nbActuators() const {return m_all.size();}

    protected:
        std::vector<std::pair<boost::shared_ptr<s2mActuator>, boost::shared_ptr<s2mActuator> > > m_all; // Tous les actuators réunis / pair (+ ou -)
        bool * m_isDofSet;
        bool m_isClose;
private:
};
#endif // S2MACTUATORS_H
