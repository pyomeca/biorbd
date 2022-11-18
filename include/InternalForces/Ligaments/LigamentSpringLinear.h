#ifndef BIORBD_LIGAMENTS_LIGAMENT_SPRING_LINEAR_H
#define BIORBD_LIGAMENTS_LIGAMENT_SPRING_LINEAR_H

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

class BIORBD_API LigamentSpringLinear : public Ligament
{
public:
    ///
    /// \brief Contruct a Ligament
    ///
    LigamentSpringLinear();

    ///
    /// \brief Construct a Hill-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    LigamentSpringLinear(
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
    LigamentSpringLinear(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a Hill-type muscle from another muscle
    /// \param other The other muscle
    ///
    LigamentSpringLinear(
        const Ligament& other);

    ///
    /// \brief Construct a Hill-type muscle from another muscle
    /// \param other THe other muscle
    ///
    LigamentSpringLinear(
        const std::shared_ptr<Ligament> other);

    ///
    /// \brief Deep copy of a Hill-type muscle
    /// \return A deep copy of a Hill-type muscle
    ///
    LigamentSpringLinear DeepCopy() const;

    ///
    /// \brief Deep copy of a Hill-type muscle in a new Hill-type muscle
    /// \param other The Hill-type to copy
    ///
    void DeepCopy(
        const LigamentSpringLinear& other);

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

#endif // BIORBD_LIGAMENTS_LIGAMENT_SPRING_LINEAR_H
