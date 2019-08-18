#ifndef BIORBD_MUSCLES_STATE_H
#define BIORBD_MUSCLES_STATE_H

#include "biorbdConfig.h"

namespace biorbd {
namespace muscles {

class BIORBD_API State
{
public:
    State(
            double e = 0,
            double a = 0);
    virtual ~State();

    // Set and Get
    virtual void setExcitation(double val);
    virtual void setActivation(double val);

    double excitation() const;
    double activation() const;


protected:
    double m_excitation;
    double m_activation;

};

}}

#endif // BIORBD_MUSCLES_STATE_H
