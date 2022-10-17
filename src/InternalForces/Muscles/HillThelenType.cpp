#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillThelenType.h"

#include <math.h>
#include "Utils/String.h"
#include "InternalForces/Muscles/Geometry.h"
#include "InternalForces/Muscles/Characteristics.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

internalforce::muscles::HillThelenType::HillThelenType() :
    internalforce::muscles::HillType()
{
    setType();
}
internalforce::muscles::HillThelenType::HillThelenType(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics) :
    internalforce::muscles::HillType (name, geometry, characteristics)
{
    setType();
}

internalforce::muscles::HillThelenType::HillThelenType(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::muscles::State &emg) :
    internalforce::muscles::HillType (name, geometry, characteristics, emg)
{
    setType();
}

internalforce::muscles::HillThelenType::HillThelenType(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const PathModifiers &pathModifiers) :
    internalforce::muscles::HillType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

internalforce::muscles::HillThelenType::HillThelenType(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics,
    const PathModifiers &pathModifiers,
    const internalforce::muscles::State& emg) :
    internalforce::muscles::HillType (name, geometry, characteristics, pathModifiers, emg)
{
    setType();
}

internalforce::muscles::HillThelenType::HillThelenType(
    const internalforce::muscles::Muscle &other) :
    internalforce::muscles::HillType (other)
{

}

internalforce::muscles::HillThelenType::HillThelenType(
    const std::shared_ptr<internalforce::muscles::Muscle> other) :
    internalforce::muscles::HillType(other)
{

}

internalforce::muscles::HillThelenType internalforce::muscles::HillThelenType::DeepCopy()
const
{
    internalforce::muscles::HillThelenType copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::HillThelenType::DeepCopy(
    const internalforce::muscles::HillThelenType &other)
{
    internalforce::muscles::HillType::DeepCopy(other);
}

void internalforce::muscles::HillThelenType::computeFlPE()
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

void internalforce::muscles::HillThelenType::computeFlCE(
    const internalforce::muscles::State&)
{
    utils::Scalar normLength = position().length() / characteristics().optimalLength();
    *m_FlCE = exp( -((normLength - 1)*(normLength - 1)) /  0.45 );
}

void internalforce::muscles::HillThelenType::computeFvCE()
{
	utils::Scalar v = m_position->velocity();
    utils::Scalar norm_v = v / (characteristics().optimalLength() * *m_cste_maxShorteningSpeed);
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
void internalforce::muscles::HillThelenType::setType()
{
    *m_type = internalforce::muscles::MUSCLE_TYPE::HILL_THELEN;
}
