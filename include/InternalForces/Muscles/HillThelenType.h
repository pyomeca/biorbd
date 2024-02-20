#ifndef BIORBD_MUSCLES_HILL_THELEN_TYPE_H
#define BIORBD_MUSCLES_HILL_THELEN_TYPE_H

#include "biorbdConfig.h"
#include "InternalForces/Muscles/HillType.h"

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
namespace muscles
{
class MuscleGeometry;
///
/// \brief Muscle of Hill type augmented by Thelen
/// https://simtk-confluence.stanford.edu/display/OpenSim/Thelen+2003+Muscle+Model
///
/// WARNING CONCENTRIC IS NOT FROM THELEN. The equation used is: 
/// (F + a) * (V + b) = (F0 + a) * b, assuming F0 = 1
/// Solved using the points: (F, V) = {(1, 0); (0, Vmax); (0.3, Vmax/3) }
/// giving a = 3 / 11;
/// giving b = 3 * this->characteristics().maxShorteningSpeed() / 11; with maxShorteningSpeed being normalized (i.e. 1)
/// And finally rearranged so F is function of all other variables
///
class BIORBD_API HillThelenType : public HillType
{
public:
    ///
    /// \brief Contruct a Hill-Thelen-type muscle
    ///
    HillThelenType();

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    HillThelenType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    ///
    HillThelenType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const State& emg);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    HillThelenType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The dynamic state
    ///
    HillThelenType(
        const utils::String& name,
        const MuscleGeometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        const State& emg);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param other The other muscle
    ///
    HillThelenType(
        const Muscle& other);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param other The other muscle (pointer)
    ///
    HillThelenType(
        const std::shared_ptr<Muscle> other);

    ///
    /// \brief Deep copy of a Hill-Thelen-type muscle
    /// \return A deep copy of a Hill-Thelen-type muscle
    ///
    HillThelenType DeepCopy() const;

    ///
    /// \brief Deep copy of a Hill-Thelen-type muscle in a new Hill-Thelen-type muscle
    /// \param other The Hill-Thelen-type muscle to copy
    ///
    void DeepCopy(const HillThelenType& other);

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
    /// \brief Set type to Hill_Thelen
    ///
    virtual void setType();

};

}
}
}

#endif // BIORBD_MUSCLES_HILL_THELEN_TYPE_H
