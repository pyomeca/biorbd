#ifndef BIORBD_MUSCLES_HILL_THELEN_ACTIVE_ONLY_TYPE_H
#define BIORBD_MUSCLES_HILL_THELEN_ACTIVE_ONLY_TYPE_H

#include "biorbdConfig.h"
#include "InternalForces/Muscles/HillThelenType.h"

namespace BIORBD_NAMESPACE
{
namespace internalforce
{
namespace muscles
{
///
/// \brief Muscle of Hill type augmented by Thelen (https://simtk-confluence.stanford.edu/display/OpenSim/Thelen+2003+Muscle+Model)
///
class BIORBD_API HillThelenActiveOnlyType : public
    HillThelenType
{
public:
    ///
    /// \brief Contruct a Hill-Thelen-type muscle
    ///
    HillThelenActiveOnlyType();

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    HillThelenActiveOnlyType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    ///
    HillThelenActiveOnlyType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const State& emg);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    HillThelenActiveOnlyType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const PathModifiers& pathModifiers);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The dynamic state
    ///
    HillThelenActiveOnlyType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const PathModifiers& pathModifiers,
        const State& emg);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param other The other muscle
    ///
    HillThelenActiveOnlyType(
        const Muscle& other);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param other The other muscle (pointer)
    ///
    HillThelenActiveOnlyType(
        const std::shared_ptr<Muscle> other);

    ///
    /// \brief Deep copy of a Hill-Thelen-type muscle
    /// \return A deep copy of a Hill-Thelen-type muscle
    ///
    HillThelenActiveOnlyType DeepCopy() const;

    ///
    /// \brief Deep copy of a Hill-Thelen-type muscle in a new Hill-Thelen-type muscle
    /// \param other The Hill-Thelen-type muscle to copy
    ///
    void DeepCopy(const HillThelenActiveOnlyType& other);

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
    /// \brief Set type to Hill_Thelen
    ///
    virtual void setType();

};

}
}
}

#endif // BIORBD_MUSCLES_HILL_THELEN_ACTIVE_ONLY_TYPE_H
