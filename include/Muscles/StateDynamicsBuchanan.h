#ifndef BIORBD_MUSCLES_STATE_ACTUAL_BUCHANAN_H
#define BIORBD_MUSCLES_STATE_ACTUAL_BUCHANAN_H

#include "biorbdConfig.h"
#include "Muscles/StateDynamics.h"

namespace biorbd {
namespace muscles {

class BIORBD_API StateDynamicsBuchanan : public biorbd::muscles::StateDynamics
{
public:
    StateDynamicsBuchanan(
            double neuralCommand = 0,
            double excitation = 0);
    ~StateDynamicsBuchanan();

    virtual double timeDerivativeExcitation(
            const Caracteristics &c,
            bool alreadyNormalized);
    virtual void setExcitation(double val);
    virtual void setNeuralCommand(double val);
    void shapeFactor(double m_shape_factor);
    double shapeFactor();
    double activation();

protected:
    double m_neuralCommand;
    double m_shapeFactor; //Buchanan2004, le 22 mars 2018
    double m_excitationDot;

};

}}

#endif // BIORBD_MUSCLES_STATE_ACTUAL_BUCHANAN_H
