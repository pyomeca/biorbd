#ifndef BIORBD_MUSCLES_HILL_TYPE_THELEN_H
#define BIORBD_MUSCLES_HILL_TYPE_THELEN_H

#include "biorbdConfig.h"
#include "Muscles/HillType.h"

namespace biorbd {
namespace muscles {

class BIORBD_API HillTypeThelen : public biorbd::muscles::HillType
{
public:
    HillTypeThelen();
    HillTypeThelen(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract);
    HillTypeThelen(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState);
    HillTypeThelen(
            const biorbd::muscles::Muscle& muscle);
    HillTypeThelen(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);

    virtual void computeFlPE();
    virtual void computeFlCE(const biorbd::muscles::StateDynamics &emg);

protected:
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_THELEN_H
