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
    utils::Scalar normLength = position().length()  / characteristics().optimalLength();
    utils::Scalar kpe = 5.0;
    utils::Scalar e0 = 0.6;
    utils::Scalar t5 = exp(kpe * (normLength - 1) / e0);
    utils::Scalar t7 = exp(kpe);

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(normLength, 1),
                  ((t5 - 1) / (t7 - 1))
                );
#else
    if (normLength > 1)
        *m_FlPE = (t5 - 1) / (t7 - 1);
    else
        *m_FlPE = 0;

#endif
}

void muscles::HillThelenType::computeFlCE(
    const muscles::State&)
{
    utils::Scalar normLength = position().length() / characteristics().optimalLength();
    *m_FlCE = exp( -((normLength - 1)*(normLength - 1)) /  0.45 );
}

void muscles::HillThelenType::computeFvCE()
{
	utils::Scalar v = m_position->velocity();
    utils::Scalar norm_v = v / *m_cste_maxShorteningSpeed;
    utils::Scalar kvce = 0.06;
    utils::Scalar flen = 1.6;

#ifdef BIORBD_USE_CASADI_MATH
    *m_FvCE = IF_ELSE_NAMESPACE::if_else(
                  IF_ELSE_NAMESPACE::gt(norm_v, 0),
                  ((1 + norm_v * flen / kvce) / (1 + norm_v / kvce)),
                  0);
#else
    if (norm_v > 0){
        *m_FvCE = (1 + norm_v * flen / kvce) / (1 + norm_v / kvce);
    } else {
        *m_FvCE = 0;
    }
#endif
}
void muscles::HillThelenType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_THELEN;
}
