#ifndef BIORBD_LIGAMENTS_LIGAMENT_CONSTANT_H
#define BIORBD_LIGAMENTS_LIGAMENT_CONSTANT_H

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
/// \brief Base class for ligament constant
class BIORBD_API LigamentConstant : public Ligament
{
public:
    ///
    /// \brief Contruct a constant ligament
    ///
    LigamentConstant();

    ///
    /// \brief Construct a ligament constant
    /// \param name The ligament name
    /// \param geometry The ligament geometry
    /// \param characteristics The ligament characteristics
    ///
    LigamentConstant(
        const utils::Scalar& force,
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a ligament constant
    /// \param name The ligament name
    /// \param geometry The ligament geometry
    /// \param characteristics The ligament characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    LigamentConstant(
        const utils::Scalar& force,
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a ligament constant  from another ligament
    /// \param other The other ligament
    ///
    LigamentConstant(
        const Ligament& other);

    ///
    /// \brief Construct a ligament constant  from another ligament
    /// \param other THe other ligament
    ///
    LigamentConstant(
        const std::shared_ptr<Ligament> other);


    ///
    /// \brief Destroy class properly
    ///
    virtual ~LigamentConstant();

    ///
    /// \brief Deep copy of a ligament constant
    /// \return A deep copy of a ligament constant
    ///
    LigamentConstant DeepCopy() const;

    ///
    /// \brief Deep copy of a ligament constant  in a new ligament constant
    /// \param other The Hill-type to copy
    ///
    void DeepCopy(
        const LigamentConstant& other);

protected:
    ///
    /// \brief Set type to Hill
    ///
    virtual void setType();

    ///
    /// \brief Compute the Force-length
    ///
    virtual void computeFl();

    std::shared_ptr<utils::Scalar> m_force; ///<Force of the ligament applied when l>l0
};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENT_CONSTANT_H
