#ifndef BIORBD_MUSCLES_FATIGUE_STATE_H
#define BIORBD_MUSCLES_FATIGUE_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "MusclesEnums.h"

namespace biorbd {
namespace muscles {

/// 
/// \brief Class FatigueState
///
class BIORBD_API FatigueState
{
public:
    ///
    /// \brief Construct fatigue state
    /// \param active Active muscle (default: 1)
    /// \param fatigued Muscle fatigue (default: 0)
    /// \param resting Resting muscle (default: 0)
    ///
    FatigueState(
            double active = 1,
            double fatigued = 0,
            double resting = 0);

    ///
    /// \brief Construct fatigue state from another fatigue state
    /// \param other The other fatigure state
    ///
    FatigueState(const biorbd::muscles::FatigueState& other);

    ///
    /// \brief Construct fatigue state from another fatigue state
    /// \param other The other fatigure state
    ///
    FatigueState(const std::shared_ptr<biorbd::muscles::FatigueState> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~FatigueState();

    ///
    /// \brief Deep copy of the fatigue state
    /// \return A deep copy of the fatigue state
    ///
    biorbd::muscles::FatigueState DeepCopy() const;
    ///
    /// \brief Deep copy of fatigue state in another fatigue state
    /// \param other The fatigue state to copy
    ///
    void DeepCopy(const biorbd::muscles::FatigueState& other);

    // Set and Get
    ///
    /// \brief Set the state
    /// \param active Active muscle
    /// \param fatigued Fatigued muscle
    /// \param resting Resting muscle
    ///
    virtual void setState(
            double active,
            double fatigued,
            double resting);

    ///
    /// \brief Return the active muscle fibers
    /// \return The active muscle fibers
    ///
    double activeFibers() const;

    ///
    /// \brief Return the fatigued muscle fibers
    /// \return The fatigued muscle fibers
    ///
    double fatiguedFibers() const;

    ///
    /// \brief Return the resting muscle fibers
    /// \return The resting muscle fibers
    ///
    double restingFibers() const;
    ///
    /// \brief Return the type of muscle fatigue
    /// \return The type of muscle fatigue
    ///
    biorbd::muscles::STATE_FATIGUE_TYPE getType() const;
protected:
    std::shared_ptr<double> m_activeFibers; ///< Active muscle fibers
    std::shared_ptr<double> m_fatiguedFibers;///< Fatigued muscle fibers
    std::shared_ptr<double> m_restingFibers;///< Resting muscle fibers

    ///
    /// \brief Set the type
    ///
    virtual void setType();
    std::shared_ptr<biorbd::muscles::STATE_FATIGUE_TYPE> m_type; ///< Type of the muscle fatigue

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_STATE_H
