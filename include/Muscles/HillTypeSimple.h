#ifndef BIORBD_MUSCLES_HILL_TYPE_SIMPLE_H
#define BIORBD_MUSCLES_HILL_TYPE_SIMPLE_H

#include "biorbdConfig.h"
#include "Muscles/HillType.h"

namespace biorbd {
namespace muscles {

class BIORBD_API HillTypeSimple : public biorbd::muscles::HillType
{
public:
    HillTypeSimple(const biorbd::utils::String& s= "");
    HillTypeSimple(
            const biorbd::muscles::Geometry& g,
            const biorbd::muscles::Caracteristics& c,
            const biorbd::muscles::PathChangers & w= biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & s= biorbd::muscles::StateDynamics());
    HillTypeSimple(
            const biorbd::utils::String& n,
            const biorbd::muscles::Geometry& g,
            const biorbd::muscles::Caracteristics& c,
            const biorbd::muscles::PathChangers & w= biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & s= biorbd::muscles::StateDynamics());
    HillTypeSimple(const biorbd::muscles::Muscle& m);
    HillTypeSimple(const std::shared_ptr<biorbd::muscles::Muscle> m);
    virtual ~HillTypeSimple();

    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(const biorbd::muscles::StateDynamics &emg);
protected:
    double multiplyCaractByActivationAndForce(const biorbd::muscles::StateDynamics &emg); // Voir dans la fonction pour descriptif
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_SIMPLE_H
