#define BIORBD_API_EXPORTS
#include "Muscles/HillThelenType.h"

#include <math.h>
#include "Utils/String.h"
#include "Muscles/Geometry.h"
#include "Muscles/Characteristics.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

muscles::HillThelenType::HillThelenType() :
    muscles::HillType()
{
    setType();
}
muscles::HillThelenType::HillThelenType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics) :
    muscles::HillType (name, geometry, characteristics)
{
    setType();
}

muscles::HillThelenType::HillThelenType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg) :
    muscles::HillType (name, geometry, characteristics, emg)
{
    setType();
}

muscles::HillThelenType::HillThelenType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers) :
    muscles::HillType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

muscles::HillThelenType::HillThelenType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers &pathModifiers,
    const muscles::State& emg) :
    muscles::HillType (name, geometry, characteristics, pathModifiers, emg)
{
    setType();
}

muscles::HillThelenType::HillThelenType(
    const muscles::Muscle &other) :
    muscles::HillType (other)
{

}

muscles::HillThelenType::HillThelenType(
    const std::shared_ptr<muscles::Muscle> other) :
    muscles::HillType(other)
{

}

muscles::HillThelenType muscles::HillThelenType::DeepCopy()
const
{
    muscles::HillThelenType copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::HillThelenType::DeepCopy(
    const muscles::HillThelenType &other)
{
    muscles::HillType::DeepCopy(other);
}

void muscles::HillThelenType::computeFlPE()
{
#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else(
                  IF_ELSE_NAMESPACE::gt(position().length(), characteristics().tendonSlackLength()),
                  (exp( *m_cste_FlPE_1 * (position().length()/characteristics().optimalLength()
                                          -1)) -1)
                  /
                  (exp( *m_cste_FlPE_2 )-1),
                  0);
#else
    if (position().length() > characteristics().tendonSlackLength())
        *m_FlPE = (exp( *m_cste_FlPE_1 *
                        (position().length()/characteristics().optimalLength()-1)) -1)
                  /
                  (exp( *m_cste_FlPE_2 )-1);
    else {
        *m_FlPE = 0;
    }
#endif
}

void muscles::HillThelenType::computeFlCE(
    const muscles::State&)
{
    *m_FlCE = exp( -pow(((position().length() / characteristics().optimalLength())
                         -1), 2 ) /  *m_cste_FlCE_2 );
}

void muscles::HillThelenType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_THELEN;
}
