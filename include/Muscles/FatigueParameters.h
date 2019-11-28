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
    /// \param fatigueRate The fatigue rate
    /// \param recoveryRate The recovery rate
    /// \param developFactor The develop factor
    /// \param recoveryFactor The recovery factor
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
    void DeepCopy(
            const biorbd::muscles::FatigueParameters& other);

    ///
    /// \brief Set the fatigue rate
    /// \param fatigueRate The fatigue rate
    ///
    void setFatigueRate(double fatigueRate);

    // Get and Set
    ///
    /// \brief Return the fatigue rate
    /// \return The fatigue rate
    ///
    double fatigueRate() const;

    ///
    /// \brief Set the recovery rate
    /// \param recoveryRate The recovery rate
    ///
    void setRecoveryRate(double recoveryRate);

    ///
    /// \brief Return the recovery rate
    /// \return The recovery rate
    ///
    double recoveryRate() const;

    ///
    /// \brief Set the develop factor
    /// \param developFactor The develop factor
    ///
    void setDevelopFactor(double developFactor);

    ///
    /// \brief Return the develop factor
    /// \return The develop factor
    ///
    double developFactor() const;

    ///
    /// \brief Set the recovery factor
    /// \param recoveryFactor The recovery factor
    ///
    void setRecoveryFactor(double recoveryFactor);

    ///
    /// \brief Return the recovery factor
    /// \return The recovery factor
    ///
    double recoveryFactor() const;

protected:
    std::shared_ptr<double> m_fatigueRate; ///< The fatigue rate
    std::shared_ptr<double> m_recoveryRate;///< The recovery rate
    std::shared_ptr<double> m_developFactor; ///<The develop factor
    std::shared_ptr<double> m_recoveryFactor; ///<The recovery factor

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
