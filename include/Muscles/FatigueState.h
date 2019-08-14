#ifndef BIORBD_MUSCLES_FATIGUE_STATE_H
#define BIORBD_MUSCLES_FATIGUE_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace muscles {

class BIORBD_API FatigueState
{
public:
    FatigueState(
            double active = 0,
            double fatigued = 0,
            double resting = 1);

    FatigueState(const std::shared_ptr<FatigueState> fatigue);
    virtual ~FatigueState();

    // Set and Get
    virtual void setState(
            double active,
            double fatigued,
            double resting);

    double activeFibers() const;
    double fatiguedFibers() const;
    double restingFibers() const;

    std::string getType() const;
protected:
    double m_activeFibers;
    double m_fatiguedFibers;
    double m_restingFibers;

    virtual void setType();
    std::string m_type; // type of the muscle fatigue

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_STATE_H
