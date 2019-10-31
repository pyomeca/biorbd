#ifndef BIORBD_ACTUATORS_ACTUATOR_GAUSS_3P_H
#define BIORBD_ACTUATORS_ACTUATOR_GAUSS_3P_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
}

namespace actuator {

    ///
    /// \brief Class ActuatorGauss3p that holds class Actuator
    ///
class BIORBD_API ActuatorGauss3p : public Actuator
{
public:
    ///
    /// \brief Construct Gauss 3p actuator
    ///
    ActuatorGauss3p();

    ///
    /// \brief Construct Gauss 3p actuator from another Gauss 3p actuator
    /// \param other The other Gauss 3p actuator to copy
    ///
    ActuatorGauss3p(
            const biorbd::actuator::ActuatorGauss3p& other);

    ///
    /// \brief Construct Gauss 3p actuator 
    /// \param direction The direction of the Gauss 3p actuator
    /// \param Tmax The maximal torque in the eccentric phase
    /// \param T0 The maxaiml torque isometric
    /// \param wmax Maximum angular velocity above which torque cannot be produced
    /// \param wc Angluar velocity of the vertical asymptote of the concentric hyperbola
    /// \param amin Low plateau level
    /// \param wr 1/10 of the distance amax/amin
    /// \param w1 Mid point plateau
    /// \param r width of the gaussian curve
    /// \param qopt Optimal position
    /// \param dofIdx Index of the DoF associated with actuator
    ///
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

    ///
    /// \brief Construct Gauss 3p actuator 
    /// \param direction The direction of the Gauss 3p actuator
    /// \param Tmax The maximal torque in the eccentric phase
    /// \param T0 The maxaiml torque isometric
    /// \param wmax Maximum angular velocity above which torque cannot be produced
    /// \param wc Angluar velocity of the vertical asymptote of the concentric hyperbola
    /// \param amin Low plateau level
    /// \param wr 1/10 of the distance amax/amin
    /// \param w1 Mid point plateau
    /// \param r width of the gaussian curve
    /// \param qopt Optimal position
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName Name of the parent joint
    ///
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

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~ActuatorGauss3p();

    ///
    /// \brief Deep copy of the Gauss 3p actuator
    /// \return A deep copy of the Gauss 3p actuator
    ///
    biorbd::actuator::ActuatorGauss3p DeepCopy() const;

    /// 
    /// \brief Deep copy of the Gauss 3p actuator from another Gauss 3p actuator
    /// \other The Gauss 3p actuator to copy
    ///
    void DeepCopy(const biorbd::actuator::ActuatorGauss3p& other);

    ///
    /// \brief Return the maximal torque
    /// \param Q The position variables of the actuator
    /// \param Qdot The velocity variables of the actuator
    /// \return The maximal torque
    ///
    virtual double torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot);

protected:
    ///
    /// \brief Set the type of actuator
    ///
    virtual void setType();             

    // For informations on these parameters, see Monique Iris Jackson's these from page 54
    // Angular/velocity relationship
    std::shared_ptr<double> m_k;         ///< Ratio of slope of the eccentric and concentric phases
    std::shared_ptr<double> m_Tmax;      ///< Maximum torque in the eccentric phase
    std::shared_ptr<double> m_T0;        ///< Maximum torque isometric
    std::shared_ptr<double> m_wmax;      ///< Maximum angular velocity above which torque cannot be produced
    std::shared_ptr<double> m_wc;        ///< Angular velocity of the vertical asymptote of the concentric hyperbola

    // Activation/velocity relationship
    std::shared_ptr<double> m_amax;      ///< Maximum activation level (set to 1)
    std::shared_ptr<double> m_amin;      ///< Low plateau level
    std::shared_ptr<double> m_wr;        ///< 1/10 de la distance amax/amin
    std::shared_ptr<double> m_w1;        ///< Mid point plateau

    // Torque/angle relationship
    std::shared_ptr<double> m_r;         ///< Width of the gaussian curve
    std::shared_ptr<double> m_qopt;      ///< Optimal position

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_GAUSS_3P_H
