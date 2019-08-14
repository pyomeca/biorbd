#ifndef BIORBD_MUSCLES_FATIGABLE_H
#define BIORBD_MUSCLES_FATIGABLE_H

#include <memory>
#include "biorbdConfig.h"

namespace biorbd { namespace utils {
class String;
}}
namespace biorbd { namespace muscles {
class Muscle;
class StateDynamics;
class FatigueState;
class BIORBD_API Fatigable
{
public:
    Fatigable(const biorbd::utils::String& dynamicFatigueType);
    Fatigable(const biorbd::muscles::Muscle& m);
    Fatigable(const std::shared_ptr<biorbd::muscles::Muscle> m);
    virtual ~Fatigable() = 0;

    virtual void computeTimeDerivativeState(const biorbd::muscles::StateDynamics& emg);

    std::shared_ptr<biorbd::muscles::FatigueState> fatigueState();
    virtual void fatigueState(double active, double fatigued, double resting);

protected:
    std::shared_ptr<biorbd::muscles::FatigueState> m_fatigueState;

};

}}

#endif // BIORBD_MUSCLES_FATIGABLE_H
