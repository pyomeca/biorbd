#ifndef BIORBD_MUSCLES_STATE_H
#define BIORBD_MUSCLES_STATE_H

#include "biorbdConfig.h"

namespace biorbd {
namespace muscles {

class BIORBD_API State
{
public:
    State(
            const double &e = 0,
            const double &a = 0);
    virtual ~State();

    // Set and Get
    virtual void setExcitation(const double &val);
    virtual void setActivation(const double &val);

    double excitation() const;
    double activation() const;


protected:
    double m_excitation;
    double m_activation;

};

}}

#endif // BIORBD_MUSCLES_STATE_H
