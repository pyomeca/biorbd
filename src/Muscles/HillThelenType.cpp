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
	utils::Scalar norm_len = position().length()/characteristics().optimalLength();
        double kpe = 5.0;
	double e0 = 0.6;
#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else(
                  IF_ELSE_NAMESPACE::gt(norm_len, 1),
                  ((exp( kpe * (norm_len - 1) / e0 ) -1)
                  /
                  (exp( kpe)-1)),
                  0);
#else
    if (norm_len > 1)
        *m_FlPE = (exp( kpe * (norm_len - 1) / e0) -1)
                  /
                  (exp( kpe )-1);
    else
        *m_FlPE = 0;

#endif
}

void muscles::HillThelenType::computeFlCE(
    const muscles::State&)
{
    utils::Scalar norm_len = position().length()/characteristics().optimalLength();
    *m_FlCE = exp( -((norm_len - 1) * (norm_len - 1)) /  0.45 );
}

void muscles::HillThelenType::computeFvCE()
{
	utils::Scalar v = m_position->velocity();
	utils::Scalar norm_v = v / *m_cste_maxShorteningSpeed;
	double kvce = 0.06;
	double flen = 1.6;

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else(
                  IF_ELSE_NAMESPACE::gt(norm_v, 0),
                  ((1 + norm_v * flen / kvce) / (1 + norm_v / kvce)),
                  0);
#else
    if (norm_v > 0){
        *m_FlPE = (1 + norm_v * flen / kvce) / (1 + norm_v / kvce);
    } else {
        *m_FlPE = 0;
    }
#endif
}
void muscles::HillThelenType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_THELEN;
}
