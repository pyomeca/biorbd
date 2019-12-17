#ifndef BIORBD_ACTUATORS_ACTUATOR_GAUSS_6P_H
#define BIORBD_ACTUATORS_ACTUATOR_GAUSS_6P_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedVelocity;
}

namespace actuator {
///
/// \brief Class ActuatorGauss6p is a joint actuator type which maximum is bimodal 6 parameter gaussian (Gauss6p)
///
class BIORBD_API ActuatorGauss6p : public Actuator
{
public:

    ///
    /// \brief Construct Gauss6p actuator
    ///
    ActuatorGauss6p();

    ///
    /// \brief Construct Gauss6p actuator from another Gauss6p actuator
    /// \param other The other Gauss6p actuator to copy
    ///
    ActuatorGauss6p(
            const biorbd::actuator::ActuatorGauss6p& other);

    ///
    /// \brief Construct Gauss6p actuator
    /// \param direction The direction of the Gauss6p actuator (+1 or -1)
    /// \param Tmax The maximal torque in the eccentric phase
    /// \param T0 The maxaiml torque isometric
    /// \param wmax Maximum angular velocity above which torque cannot be produced
    /// \param wc Angluar velocity of the vertical asymptote of the concentric hyperbola
    /// \param amin Low plateau level
    /// \param wr 1/10 of the distance amax/amin
    /// \param w1 Mid point plateau
    /// \param r Width of the 1st gaussian curve
    /// \param qopt 1st Optimal position
    /// \param facteur Factor of the 6p
    /// \param r2 Width of the 2nd gaussian curve2
    /// \param qopt2 2nd Optimal position
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    ActuatorGauss6p(
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
            double facteur,
            double r2,
            double qopt2,
            unsigned int dofIdx);

    ///
    /// \brief Construct Gauss6p actuator
    /// \param direction The direction of the Gauss6p actuator (+1 or -1)
    /// \param Tmax The maximal torque in the eccentric phase
    /// \param T0 The maxaiml torque isometric
    /// \param wmax Maximum angular velocity above which torque cannot be produced
    /// \param wc Angluar velocity of the vertical asymptote of the concentric hyperbola
    /// \param amin Low plateau level
    /// \param wr 1/10 of the distance amax/amin
    /// \param w1 Mid point plateau
    /// \param r Width of the 1st gaussian curve
    /// \param qopt 1st Optimal position
    /// \param facteur Factor of the 6p
    /// \param r2 Width of the 2nd gaussian curve
    /// \param qopt2 2nd Optimal position 2
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName The name of the parent joint
    ///
    ActuatorGauss6p(
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
            double facteur,
            double r2,
            double qopt2,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~ActuatorGauss6p();

    ///
    /// \brief Deep copy of the Gauss6p actuator
    /// \return A deep copy of the Gauss6p actuator
    ///
    biorbd::actuator::ActuatorGauss6p DeepCopy() const;

    /// 
    /// \brief Deep copy of the Gauss 3p actuator from another Gauss6p actuator
    /// \param other The Gauss6p actuator to copy
    ///
    void DeepCopy(
            const biorbd::actuator::ActuatorGauss6p& other);

    ///
    /// \brief Return the maximal torque at a given Q and Qdot
    /// \param Q The generalized coordinates of the actuator
    /// \param Qdot The generalized velocities of the actuator
    /// \return The maximal torque
    ///
    virtual double torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedVelocity &Qdot);

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
    std::shared_ptr<double> m_wr;        ///< 1/10 of the distance amax/amin
    std::shared_ptr<double> m_w1;        ///< Mid point plateau

    // Torque/angle relationship
    std::shared_ptr<double> m_r;         ///< width of the 1st gaussian curve
    std::shared_ptr<double> m_qopt;      ///< 1st Optimal position
    std::shared_ptr<double> m_facteur;   ///< Factor of the 6p
    std::shared_ptr<double> m_r2;        ///< width of the 2nd gaussian curve
    std::shared_ptr<double> m_qopt2;     ///< 2nd Optimal position

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_GAUSS_6P_H
