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

///
/// \brief Class LigamentSpringLinear is a ligament with same behavior than linear spring defined by:
/// k(l-l0) with k is the stiffness
class BIORBD_API LigamentSpringLinear : public Ligament
{
public:
    ///
    /// \brief Contruct a Ligament
    ///
    LigamentSpringLinear();

    ///
    /// \brief Construct a linear spring like ligament
    /// \param stifness The ligament stifness
    /// \param name The ligament name
    /// \param geometry The ligament geometry
    /// \param characteristics The ligament characteristics
    ///
    LigamentSpringLinear(
        const utils::Scalar& stiffness,
        const utils::String& name,
        const Geometry& geometry,
        const LigamentCharacteristics& characteristics);

    ///
    /// \brief Construct a linear spring like ligament
    /// \param stifness The ligament stifness
    /// \param name The ligament name
    /// \param geometry The ligament geometry
    /// \param characteristics The ligament characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    LigamentSpringLinear(
        const utils::Scalar& stiffness,
        const utils::String& name,
        const Geometry& geometry,
        const LigamentCharacteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a linear spring like ligament from another ligament
    /// \param other The other ligament
    ///
    LigamentSpringLinear(
        const Ligament& other);

    ///
    /// \brief Construct a linear spring like ligament from another ligament
    /// \param other THe other ligament
    ///
    LigamentSpringLinear(
        const std::shared_ptr<Ligament> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~LigamentSpringLinear();

    ///
    /// \brief Deep copy of a linear spring like ligament
    /// \return A deep copy of a linear spring like ligament
    ///
    LigamentSpringLinear DeepCopy() const;

    ///
    /// \brief Deep copy of a linear spring like ligament in a new linear spring like ligament
    /// \param other The linear spring like ligament to copy
    ///
    void DeepCopy(
        const LigamentSpringLinear& other);

    ///
    /// \brief Return the ligament stiffness
    /// \return The ligament stiffness
    ///
    const utils::Scalar& stiffness() const;

    ///
    /// \brief Set the ligament stiffness
    /// \param stiffness The ligament stiffness
    ///
    void setStiffness(
        const utils::Scalar& stiffness);

protected:
    ///
    /// \brief Set type to linear spring like ligament
    ///
    virtual void setType();

    ///
    /// \brief Compute the Force-length of the contractile element
    ///
    virtual void computeFl();

    std::shared_ptr<utils::Scalar> m_stiffness; ///<stiffness of the ligament

};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENT_SPRING_LINEAR_H
