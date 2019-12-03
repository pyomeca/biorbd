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
/// 
/// \brief Class Actuators
///
class BIORBD_API Actuators
{
public:
    ///
    /// \brief Construct actuators
    ///
    Actuators();

    ///
    /// \brief Construct actuators from other actuators
    /// \param other The other actuators
    ///
    Actuators(
            const biorbd::actuator::Actuators& other);

    ///
    /// \brief Destroy actuators class properly
    ///
    virtual ~Actuators();

    ///
    /// \brief Deep copy of the actuators from other actuators
    /// \param other The other actuators
    ///
    void DeepCopy(const biorbd::actuator::Actuators& other);

    /// 
    /// \brief Add an actuator to all the actuators
    /// \param a The actuator to add
    ///
    void addActuator(
            const biorbd::actuator::Actuator &a);

    ///
    /// \brief To close the actuator loop? TODO
    /// 
    void closeActuator();



    ///
    /// \brief Return two vectors of max torque (we're missing the power entry to know if it's positive or negative at each joint; therefore both are returned)
    /// \param Q The position variables of the actuators
    /// \param Qdot The velocity variables of the actuators
    /// \return Two vectors of max torque
    ///
    std::pair<biorbd::rigidbody::GeneralizedTorque, biorbd::rigidbody::GeneralizedTorque> torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot);

    ///
    /// \brief Return the maximal generalized torque
    /// \param a TODO?
    /// \param Q The position variables of the actuators
    /// \param Qdot The velocity variables of the actuators
    /// \return The maximal generalized torque
    ///
    biorbd::rigidbody::GeneralizedTorque torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates& a,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot);

    ///
    /// \brief Return the generalized torque
    /// \param a TODO?
    /// \param Q The position variables of the actuators
    /// \param Qdot The velocity variables of the actuators
    /// \return The maximal generalized torque
    ///
    biorbd::rigidbody::GeneralizedTorque torque(
            const biorbd::rigidbody::GeneralizedCoordinates& a,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot);

    // Get and set
    ///
    /// \brief Return a specific actuator
    /// \param dof Index of the DoF associated with actuator
    /// \return A specific actuator
    ///
    const std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>& actuator(unsigned int dof);
    
    ///
    /// \brief Return a specific actuator
    /// \param dof Index of the DoF associated with actuator
    /// \param idx Index of actuator (0 or 1)
    /// \return A specific actuator
    ///
    const biorbd::actuator::Actuator& actuator(unsigned int dof, unsigned int idx);

    ///
    /// \brief Return the toal number of actuators
    /// \return The total number of actuators
    ///
    unsigned int nbActuators() const;

protected:
    std::shared_ptr<std::vector<std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>>> m_all; ///<All the actuators reunited /pair (+ or -)
    std::shared_ptr<std::vector<bool>> m_isDofSet;///< If DoF is set? TODO
    std::shared_ptr<bool> m_isClose; ///< If loop is closed? TODO

};

}}

#endif // BIORBD_ACTUATORS_ACTUATORS_H
