#ifndef BIORBD_LIGAMENTS_LIGAMENT_SPRING_SECOND_ORDER_H
#define BIORBD_LIGAMENTS_LIGAMENT_SPRING_SECOND_ORDER_H

#include "biorbdConfig.h"
#include "InternalForces/Ligaments/Ligament.h"
#include "Utils/Scalar.h"

///
/// \brief Class LigamentSpringSecondOrder is a ligament with same behavior than second order spring defined by:
/// (k/2) * ((l-l0) + sqrt((l-l0)*(l-l0) + epsilon*epsilon)) with k is the stiffness and espilon a small parameter to avoid negative values.
/// https://ieeexplore.ieee.org/abstract/document/6755458
///
namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
class Geometry;
namespace ligaments
{

class BIORBD_API LigamentSpringSecondOrder : public Ligament
{
public:
    ///
    /// \brief Contruct a Ligament
    ///
    LigamentSpringSecondOrder();

    ///
    /// \brief Construct a second order spring like ligament
    /// \param stiffness The ligament stiffness
    /// \param epsilon The ligament param to avoid negative values
    /// \param name The ligament name
    /// \param geometry The ligament geometry
    /// \param characteristics The ligament characteristics
    ///
    LigamentSpringSecondOrder(
        const utils::Scalar& stiffness,
        const utils::Scalar& epsilon,
        const utils::String& name,
        const Geometry& geometry,
        const LigamentCharacteristics& characteristics);

    ///
    /// \brief Construct a second order spring like ligament
    /// \param stiffness The ligament stiffness
    /// \param epsilon The ligament param to avoid negative values
    /// \param name The ligament name
    /// \param geometry The ligament geometry
    /// \param characteristics The ligament characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    LigamentSpringSecondOrder(
        const utils::Scalar& stiffness,
        const utils::Scalar& epsilon,
        const utils::String& name,
        const Geometry& geometry,
        const LigamentCharacteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a second order spring like ligament from another ligament
    /// \param other The other ligament
    ///
    LigamentSpringSecondOrder(
        const Ligament& other);

    ///
    /// \brief Construct a second order spring like ligament from another ligament
    /// \param other THe other ligament
    ///
    LigamentSpringSecondOrder(
        const std::shared_ptr<Ligament> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~LigamentSpringSecondOrder();

    ///
    /// \brief Deep copy of a second order spring like ligament
    /// \return A deep copy of a second order spring like ligament
    ///
    LigamentSpringSecondOrder DeepCopy() const;

    ///
    /// \brief Deep copy of a second order spring like ligament in a new second order spring like ligament
    /// \param other The second order spring like ligament to copy
    ///
    void DeepCopy(
        const LigamentSpringSecondOrder& other);

    ///
    /// \brief Get the stiffness
    /// \return The stiffness
    ///
    const utils::Scalar& stiffness() const;

    ///
    /// \brief Set the stiffness
    /// \param stiffness The stiffness
    ///
    void setStiffness(
        const utils::Scalar& stiffness);

    ///
    /// \brief Get the epsilon
    /// \return The epsilon
    ///
    const utils::Scalar& epsilon() const;

    ///
    /// \brief Set the epsilon
    /// \param epsilon The epsilon
    ///
    void setEpsilon(
        const utils::Scalar& epsilon);

protected:
    ///
    /// \brief Set type to second order spring like ligament
    ///
    virtual void setType();

    ///
    /// \brief Compute the Force-length of the contractile element
    ///
    virtual void computeFl();

    std::shared_ptr<utils::Scalar> m_stiffness; ///<stiffness of the ligament
    std::shared_ptr<utils::Scalar> m_epsilon; ///<small parameter to avoid negative value

};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENT_SPRING_SECOND_ORDER_H
