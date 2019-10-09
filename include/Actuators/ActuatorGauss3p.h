#ifndef BIORBD_ACTUATORS_ACTUATOR_GAUSS_3P_H
#define BIORBD_ACTUATORS_ACTUATOR_GAUSS_3P_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
}

namespace actuator {

class BIORBD_API ActuatorGauss3p : public Actuator
{
public:
    ActuatorGauss3p();
    ActuatorGauss3p(
            const biorbd::actuator::ActuatorGauss3p& other);
    ActuatorGauss3p(
            int direction,
            double Tmax,
            double T0,
            double wmax,
            double wc,
            double amin,
            double wr,
            double w1,
            double r,
            double qopt,
            unsigned int dofIdx);
    ActuatorGauss3p(
            int direction,
            double Tmax,
            double T0,
            double wmax,
            double wc,
            double amin,
            double wr,
            double w1,
            double r,
            double qopt,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName);
    virtual ~ActuatorGauss3p();
    biorbd::actuator::ActuatorGauss3p DeepCopy() const;
    void DeepCopy(const biorbd::actuator::ActuatorGauss3p& other);

    virtual double torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot);

protected:

    virtual void setType();             // Quel type d'actuator

    // For informations on these parameters, see Monique Iris Jackson's these from page 54
    // Angular/velocity relationship
    std::shared_ptr<double> m_k;         // Ratio of slope of the eccentric and concentric phases
    std::shared_ptr<double> m_Tmax;      // Maximum torque in the eccentric phase
    std::shared_ptr<double> m_T0;        // Maximum torque isometric
    std::shared_ptr<double> m_wmax;      // Maximum angular velocity above which torque cannot be produced
    std::shared_ptr<double> m_wc;        // Angular velocity of the vertical asymptote of the concentric hyperbola

    // Activation/velocity relationship
    std::shared_ptr<double> m_amax;      // Maximum activation level (set to 1)
    std::shared_ptr<double> m_amin;      // Low plateau level
    std::shared_ptr<double> m_wr;        // 1/10 de la distance amax/amin
    std::shared_ptr<double> m_w1;        // Mid point plateau

    // Torque/angle relationship
    std::shared_ptr<double> m_r;         // width of the gaussian curve
    std::shared_ptr<double> m_qopt;      // Optimal position

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_GAUSS_3P_H
