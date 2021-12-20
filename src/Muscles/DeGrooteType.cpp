#define BIORBD_API_EXPORTS
#include "Muscles/DeGrooteType.h"

#include <math.h>
#include "Utils/String.h"
#include "Muscles/Geometry.h"
#include "Muscles/Characteristics.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

muscles::DeGrooteType::DeGrooteType() :
    muscles::HillType()
{
    setType();
}
muscles::DeGrooteType::DeGrooteType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics) :
    muscles::HillType (name, geometry, characteristics)
{
    setType();
}

muscles::DeGrooteType::DeGrooteType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg) :
    muscles::HillType (name, geometry, characteristics, emg)
{
    setType();
}

muscles::DeGrooteType::DeGrooteType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers) :
    muscles::HillType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

muscles::DeGrooteType::DeGrooteType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers &pathModifiers,
    const muscles::State& emg) :
    muscles::HillType (name, geometry, characteristics, pathModifiers, emg)
{
    setType();
}

muscles::DeGrooteType::DeGrooteType(
    const muscles::Muscle &other) :
    muscles::HillType (other)
{

}

muscles::DeGrooteType::DeGrooteType(
    const std::shared_ptr<muscles::Muscle> other) :
    muscles::HillType(other)
{

}

muscles::DeGrooteType muscles::DeGrooteType::DeepCopy()
const
{
    muscles::DeGrooteType copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::DeGrooteType::DeepCopy(
    const muscles::DeGrooteType &other)
{
    muscles::HillType::DeepCopy(other);
}

void muscles::DeGrooteType::computeFlPE()
{
    double kpe = 4;
    double e0 = 0.6;
    utils::Scalar norm_length = position().length()/characteristics().optimalLength();

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(norm_length, 1),
                (exp( (kpe * (norm_length-1)) / e0) -1)
                                      /
                                  (exp( kpe ) - 1));
#else

    if (norm_length > 1) {
        *m_FlPE = (exp( (kpe * (norm_length-1)) / e0) -1)
                      /
                  (exp( kpe ) - 1);
    } else {
        *m_FlPE = 0;
    }
#endif
}

void muscles::DeGrooteType::computeFvCE()
{

    double d1 = -0.318;
    double d2 = -8.149;
    double d3 = -0.374;
    double d4 = 0.886;
    utils::Scalar norm_v = m_position->velocity() / (*m_cste_maxShorteningSpeed * characteristics().optimalLength());

    *m_FvCE = d1 * log((d2 * norm_v + d3) +
                             sqrt(( d2 * norm_v + d3)*( d2 * norm_v + d3) + 1))
            + d4;
}

void muscles::DeGrooteType::computeFlCE(
    const muscles::State&)
{
    double b11 = 0.815;
    double b21 = 1.055;
    double b31 = 0.162;
    double b41 = 0.063;
    double b12 = 0.433;
    double b22 = 0.717;
    double b32 = -0.030;
    double b42 = 0.200;
    double b13 = 0.100;
    double b23 = 1.000;
    double b33 = 0.354;
    double b43 = 0.0;
    utils::Scalar norm_length = position().length()/characteristics().optimalLength();

    *m_FlCE = b11 * exp((-0.5*((norm_length-b21)*(norm_length-b21)))
                       /
                       ((b31 + b41*norm_length)*(b31 + b41*norm_length)))
            +
            b12 * exp((-0.5*((norm_length-b22)*(norm_length-b22)))
                        /
                        ((b32 + b42*norm_length)*(b32 + b42*norm_length)))
            +
            b13 * exp((-0.5*((norm_length-b23)*(norm_length-b23)))
                        /
                        ((b33 + b43*norm_length)*(b33 + b43*norm_length)));
}

void muscles::DeGrooteType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::DE_GROOTE;
}
