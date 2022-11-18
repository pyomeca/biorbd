#ifndef BIORBD_LIGAMENTS_LIGAMENT_SPRING_SECOND_ORDER_H
#define BIORBD_LIGAMENTS_LIGAMENT_SPRING_SECOND_ORDER_H

#include "biorbdConfig.h"
#include "InternalForces/Ligaments/Ligament.h"
#include "Utils/Scalar.h"


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
    /// \brief Construct a Hill-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    LigamentSpringSecondOrder(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a Hill-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    LigamentSpringSecondOrder(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a Hill-type muscle from another muscle
    /// \param other The other muscle
    ///
    LigamentSpringSecondOrder(
        const Ligament& other);

    ///
    /// \brief Construct a Hill-type muscle from another muscle
    /// \param other THe other muscle
    ///
    LigamentSpringSecondOrder(
        const std::shared_ptr<Ligament> other);

    ///
    /// \brief Deep copy of a Hill-type muscle
    /// \return A deep copy of a Hill-type muscle
    ///
    LigamentSpringSecondOrder DeepCopy() const;

    ///
    /// \brief Deep copy of a Hill-type muscle in a new Hill-type muscle
    /// \param other The Hill-type to copy
    ///
    void DeepCopy(
        const LigamentSpringSecondOrder& other);

protected:
    ///
    /// \brief Set type to Hill
    ///
    virtual void setType();

    ///
    /// \brief Compute the Force-length of the contractile element
    ///
    virtual void computeFl();

};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENT_SPRING_SECOND_ORDER_H
