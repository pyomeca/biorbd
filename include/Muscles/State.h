#ifndef BIORBD_MUSCLES_STATE_H
#define BIORBD_MUSCLES_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "Muscles/MusclesEnums.h"

namespace biorbd {
namespace muscles {

class BIORBD_API State
{
public:
    State(
            double excitation = 0,
            double activation = 0);
    State(const biorbd::muscles::State& other);
    virtual ~State();
    biorbd::muscles::State DeepCopy() const;
    void DeepCopy(const biorbd::muscles::State& other);

    // Set and Get
    virtual void setExcitation(double val);
    virtual void setActivation(double val);

    double excitation() const;
    double activation() const;

    biorbd::muscles::STATE_TYPE type() const;
protected:
    virtual void setType();

    std::shared_ptr<biorbd::muscles::STATE_TYPE> m_stateType;
    std::shared_ptr<double> m_excitation;
    std::shared_ptr<double> m_activation;

};

}}

#endif // BIORBD_MUSCLES_STATE_H
