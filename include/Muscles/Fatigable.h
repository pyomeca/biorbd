#ifndef BIORBD_MUSCLES_FATIGABLE_H
#define BIORBD_MUSCLES_FATIGABLE_H

#include <memory>
#include "biorbdConfig.h"
#include "MusclesEnums.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {
class Muscle;
class StateDynamics;
class FatigueState;

class BIORBD_API Fatigable
{
public:
    Fatigable(biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType);
    Fatigable(const biorbd::muscles::Fatigable& m);
    Fatigable(const std::shared_ptr<biorbd::muscles::Fatigable> m);
    virtual ~Fatigable() = 0;
    void DeepCopy(const biorbd::muscles::Fatigable& other);

    virtual void computeTimeDerivativeState(const biorbd::muscles::StateDynamics& emg);

    biorbd::muscles::FatigueState& fatigueState();
    const biorbd::muscles::FatigueState& fatigueState() const;
    virtual void fatigueState(double active, double fatigued, double resting);

protected:
    std::shared_ptr<biorbd::muscles::FatigueState> m_fatigueState;

};

}}

#endif // BIORBD_MUSCLES_FATIGABLE_H
