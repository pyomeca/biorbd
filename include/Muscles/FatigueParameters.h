#ifndef BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
#define BIORBD_MUSCLES_FATIGUE_PARAMETERS_H

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
    double m_fatigueRate;
    double m_recoveryRate;
    double m_developFactor;
    double m_recoveryFactor;

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
