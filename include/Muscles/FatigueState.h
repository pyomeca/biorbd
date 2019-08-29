#ifndef BIORBD_MUSCLES_FATIGUE_STATE_H
#define BIORBD_MUSCLES_FATIGUE_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "MusclesEnums.h"

namespace biorbd {
namespace muscles {

class BIORBD_API FatigueState
{
public:
    FatigueState(
            double active = 0,
            double fatigued = 0,
            double resting = 1);
    FatigueState(const biorbd::muscles::FatigueState& other);
    FatigueState(const std::shared_ptr<biorbd::muscles::FatigueState> other);
    virtual ~FatigueState();
    biorbd::muscles::FatigueState DeepCopy() const;
    void DeepCopy(const biorbd::muscles::FatigueState& other);

    // Set and Get
    virtual void setState(
            double active,
            double fatigued,
            double resting);

    double activeFibers() const;
    double fatiguedFibers() const;
    double restingFibers() const;

    biorbd::muscles::STATE_FATIGUE_TYPE getType() const;
protected:
    std::shared_ptr<double> m_activeFibers;
    std::shared_ptr<double> m_fatiguedFibers;
    std::shared_ptr<double> m_restingFibers;

    virtual void setType();
    std::shared_ptr<biorbd::muscles::STATE_FATIGUE_TYPE> m_type; // type of the muscle fatigue

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_STATE_H
