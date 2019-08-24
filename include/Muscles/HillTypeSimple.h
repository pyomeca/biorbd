#ifndef BIORBD_MUSCLES_HILL_TYPE_SIMPLE_H
#define BIORBD_MUSCLES_HILL_TYPE_SIMPLE_H

#include "biorbdConfig.h"
#include "Muscles/HillType.h"

namespace biorbd {
namespace muscles {

class BIORBD_API HillTypeSimple : public biorbd::muscles::HillType
{
public:
    HillTypeSimple();
    HillTypeSimple(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract);
    HillTypeSimple(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState);
    HillTypeSimple(
            const HillType& muscle);
    HillTypeSimple(
            const std::shared_ptr<HillType> muscle);

    virtual const std::vector<biorbd::muscles::Force>& force(
            const biorbd::muscles::StateDynamics &emg);
protected:
    double multiplyCaractByActivationAndForce(
            const biorbd::muscles::StateDynamics &emg); // Voir dans la fonction pour descriptif
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_SIMPLE_H
