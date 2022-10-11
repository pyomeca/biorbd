#ifndef BIORBD_MUSCLES_HILL_DE_GROOTE_TYPE_H
#define BIORBD_MUSCLES_HILL_DE_GROOTE_TYPE_H

#include "biorbdConfig.h"
#include "InternalForces/Muscles/HillType.h"

namespace BIORBD_NAMESPACE
{
namespace internalforce
{
namespace muscles
{
///
/// \brief Muscle based on DeGroote 2016
/// https://link.springer.com/article/10.1007%2Fs10439-016-1591-9
///
class BIORBD_API HillDeGrooteType : public HillType
{
public:
    ///
    /// \brief Contruct a DeGroote-type muscle
    ///
    HillDeGrooteType();

    ///
    /// \brief Construct a DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    HillDeGrooteType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    ///
    HillDeGrooteType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const State& emg);

    ///
    /// \brief Construct a DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    HillDeGrooteType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const PathModifiers& pathModifiers);

    ///
    /// \brief Construct a DeGroote-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The dynamic state
    ///
    HillDeGrooteType(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const PathModifiers& pathModifiers,
        const State& emg);

    ///
    /// \brief Construct a DeGroote-type muscle from another muscle
    /// \param other The other muscle
    ///
    HillDeGrooteType(
        const Muscle& other);

    ///
    /// \brief Construct a DeGroote-type muscle from another muscle
    /// \param other The other muscle (pointer)
    ///
    HillDeGrooteType(
        const std::shared_ptr<Muscle> other);

    ///
    /// \brief Deep copy of a DeGroote-type muscle
    /// \return A deep copy of a DeGroote-type muscle
    ///
    HillDeGrooteType DeepCopy() const;

    ///
    /// \brief Deep copy of a DeGroote-type muscle in a new DeGroote-type muscle
    /// \param other The DeGroote-type muscle to copy
    ///
    void DeepCopy(const HillDeGrooteType& other);

    ///
    /// \brief Compute the Force-Length of the passive element
    ///
    virtual void computeFlPE();
    
    ///
    /// \brief Compute the Force-Velocity of the passive element
    ///
    virtual void computeFvCE();

    ///
    /// \brief Compute the Force-Length of the passive element
    /// \param emg EMG data
    ///
    virtual void computeFlCE(const State &emg);

protected:
    ///
    /// \brief Set type to De_Groote
    ///
    virtual void setType();

};

}
}
}

#endif // BIORBD_MUSCLES_DE_GROOTE_TYPE_H
