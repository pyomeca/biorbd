#ifndef BIORBD_MUSCLES_HILL_TYPE_THELEN_H
#define BIORBD_MUSCLES_HILL_TYPE_THELEN_H

#include "biorbdConfig.h"
#include "Muscles/HillType.h"

namespace biorbd {
namespace muscles {

class BIORBD_API HillTypeThelen : public biorbd::muscles::HillType
{
public:
    HillTypeThelen(const biorbd::utils::String& s= "") : biorbd::muscles::HillType(s){setType();}
    HillTypeThelen(
            const biorbd::muscles::Geometry& g,
            const biorbd::muscles::Caracteristics& c,
            const biorbd::muscles::PathChangers & w= biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & s= biorbd::muscles::StateDynamics());

    HillTypeThelen(
            const biorbd::utils::String& n,
            const biorbd::muscles::Geometry& g,
            const biorbd::muscles::Caracteristics& c,
            const biorbd::muscles::PathChangers & w= biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & s= biorbd::muscles::StateDynamics());
    HillTypeThelen(const biorbd::muscles::Muscle& m);
    HillTypeThelen(const std::shared_ptr<biorbd::muscles::Muscle> m);
    virtual ~HillTypeThelen();

    virtual void computeFlPE();
    virtual void computeFlCE(const biorbd::muscles::StateDynamics &emg);

protected:
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_THELEN_H
