#ifndef BIORBD_MUSCLES_HILL_THELEN_TYPE_H
#define BIORBD_MUSCLES_HILL_THELEN_TYPE_H

#include "biorbdConfig.h"
#include "Muscles/HillType.h"

namespace biorbd {
namespace muscles {

class BIORBD_API HillThelenType : public biorbd::muscles::HillType
{
public:
    HillThelenType();
    HillThelenType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics);
    HillThelenType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::StateDynamics& dynamicState);
    HillThelenType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers);
    HillThelenType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState);
    HillThelenType(
            const biorbd::muscles::Muscle& muscle);
    HillThelenType(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);
    biorbd::muscles::HillThelenType DeepCopy() const;
    void DeepCopy(const biorbd::muscles::HillThelenType& other);

    virtual void computeFlPE();
    virtual void computeFlCE(const biorbd::muscles::StateDynamics &emg);

protected:
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_THELEN_TYPE_H
