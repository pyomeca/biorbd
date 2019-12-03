#ifndef BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
#define BIORBD_MUSCLES_FATIGUE_PARAMETERS_H

#include <memory>
#include "biorbdConfig.h"

namespace biorbd {
namespace muscles {

///
/// \brief Class FatigueParameters that holds the muscle fatigue parameters
///
class BIORBD_API FatigueParameters
{
public:

    ///
    /// \brief Construct the fatigue parameters
    /// \param fatigueRate The fatigue rate (default: 0)
    /// \param recoveryRate The recovery rate (default: 0)
    /// \param developFactor The develop factor (default: 0)
    /// \param recoveryFactor The recovery factor (default: 0)
    ///
    FatigueParameters(
            double fatigueRate = 0,
            double recoveryRate = 0,
            double developFactor = 0,
            double recoveryFactor = 0);

    ///
    /// \brief Deep copy of the fatigue parameters
    /// \return Copy of the fatigue parameters
    ///
    biorbd::muscles::FatigueParameters DeepCopy() const;

    ///
    /// \brief Deep copy of fatigue parameters into another FatigueParameters
    /// \param other The fatigue parameters to copy
    ///
    void DeepCopy(const biorbd::muscles::FatigueParameters& other);

    // Get and Set
    ///
    /// \brief Return the fatigue rate
    /// \return The fatigue rate
    ///
    double fatigueRate() const;
    
    ///
    /// \brief Return the recovery rate
    /// \return The recovery rate
    ///
    double recoveryRate() const;
    
    ///
    /// \brief Return the develop factor
    /// \return The develop factor
    ///
    double developFactor() const;
    
    ///
    /// \brief Return the recovery factor
    /// \return The recovery factor
    ///
    double recoveryFactor() const;

    ///
    /// \brief Set the fatigue rate
    /// \param fatigueRate The fatigue rate
    ///
    void fatigueRate(double fatigueRate);

    ///
    /// \brief Set the recovery rate
    /// \param recoveryRate The recovery rate
    ///
    void recoveryRate(double recoveryRate);

    ///
    /// \brief Set the develop factor
    /// \param developFactor The develop factor
    ///
    void developFactor(double developFactor);

    ///
    /// \brief Set the recovery factor
    /// \param recoveryFactor The recovery factor
    ///
    void recoveryFactor(double recoveryFactor);

protected:
    std::shared_ptr<double> m_fatigueRate; ///< The fatigue rate
    std::shared_ptr<double> m_recoveryRate;///< The recovery rate
    std::shared_ptr<double> m_developFactor; ///<The develop factor
    std::shared_ptr<double> m_recoveryFactor; ///<The recovery factor

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
