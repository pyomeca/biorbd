#ifndef BIORBD_MUSCLES_HILL_DE_GROOTE_ACTIVE_ONLY_TYPE_H
#define BIORBD_MUSCLES_HILL_DE_GROOTE_ACTIVE_ONLY_TYPE_H

#include "biorbdConfig.h"
#include "InternalForces/Muscles/HillDeGrooteType.h"

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
namespace muscles
{
class MuscleGeometry;
///
/// \brief Muscle of Hill type augmented by Thelen (https://simtk-confluence.stanford.edu/display/OpenSim/Thelen+2003+Muscle+Model)
///
class BIORBD_API HillDeGrooteActiveOnlyType : public
    HillDeGrooteType
{
public:
    ///
    /// \brief Contruct a Hill-DeGroote-type muscle
    ///
    HillDeGrooteActiveOnlyType();

    ///
    /// \brief Construct a Hill-DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    HillDeGrooteActiveOnlyType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a Hill-DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    ///
    HillDeGrooteActiveOnlyType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const State& emg);

    ///
    /// \brief Construct a Hill-DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    HillDeGrooteActiveOnlyType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a Hill-DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The dynamic state
    ///
    HillDeGrooteActiveOnlyType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        const State& emg);

    ///
    /// \brief Construct a Hill-DeGroote-type muscle from another muscle
    /// \param other The other muscle
    ///
    HillDeGrooteActiveOnlyType(
        const Muscle& other);

    ///
    /// \brief Construct a Hill-DeGroote-type muscle from another muscle
    /// \param other The other muscle (pointer)
    ///
    HillDeGrooteActiveOnlyType(
        const std::shared_ptr<Muscle> other);

    ///
    /// \brief Deep copy of a Hill-DeGroote-type muscle
    /// \return A deep copy of a Hill-DeGroote-type muscle
    ///
    HillDeGrooteActiveOnlyType DeepCopy() const;

    ///
    /// \brief Deep copy of a Hill-DeGroote-type muscle in a new Hill-DeGroote-type muscle
    /// \param other The Hill-DeGroote-type muscle to copy
    ///
    void DeepCopy(const HillDeGrooteActiveOnlyType& other);

    ///
    /// \brief Compute the Force-Length of the passive element
    ///
    virtual void computeFlPE();

    ///
    /// \brief Compute the muscle damping
    ///
    virtual void computeDamping();

protected:
    ///
    /// \brief Set type to Hill_DeGroote
    ///
    virtual void setType();

};

}
}
}

#endif // BIORBD_MUSCLES_HILL_DE_GROOTE_ACTIVE_ONLY_TYPE_H
