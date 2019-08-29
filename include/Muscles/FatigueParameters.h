#ifndef BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
#define BIORBD_MUSCLES_FATIGUE_PARAMETERS_H

#include <memory>
#include "biorbdConfig.h"

namespace biorbd {
namespace muscles {

class BIORBD_API FatigueParameters
{
public:
    FatigueParameters(
            double fatigueRate = 0,
            double recoveryRate = 0,
            double developFactor = 0,
            double recoveryFactor = 0);
    biorbd::muscles::FatigueParameters DeepCopy() const;
    void DeepCopy(const biorbd::muscles::FatigueParameters& other);

    // Get and Set
    double fatigueRate() const;
    double recoveryRate() const;
    double developFactor() const;
    double recoveryFactor() const;

    void fatigueRate(double fatigueRate);
    void recoveryRate(double recoveryRate);
    void developFactor(double developFactor);
    void recoveryFactor(double recoveryFactor);

protected:
    std::shared_ptr<double> m_fatigueRate;
    std::shared_ptr<double> m_recoveryRate;
    std::shared_ptr<double> m_developFactor;
    std::shared_ptr<double> m_recoveryFactor;

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
