#define BIORBD_API_EXPORTS
#include "Muscles/HillDeGrooteType.h"

#include <math.h>
#include "Utils/String.h"
#include "Muscles/Geometry.h"
#include "Muscles/Characteristics.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

muscles::HillDeGrooteType::HillDeGrooteType() :
    muscles::HillType()
{
    setType();
}
muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics) :
    muscles::HillType (name, geometry, characteristics)
{
    setType();
}

muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg) :
    muscles::HillType (name, geometry, characteristics, emg)
{
    setType();
}

muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers) :
    muscles::HillType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers &pathModifiers,
    const muscles::State& emg) :
    muscles::HillType (name, geometry, characteristics, pathModifiers, emg)
{
    setType();
}

muscles::HillDeGrooteType::HillDeGrooteType(
    const muscles::Muscle &other) :
    muscles::HillType (other)
{

}

muscles::HillDeGrooteType::HillDeGrooteType(
    const std::shared_ptr<muscles::Muscle> other) :
    muscles::HillType(other)
{

}

muscles::HillDeGrooteType muscles::HillDeGrooteType::DeepCopy()
const
{
    muscles::HillDeGrooteType copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::HillDeGrooteType::DeepCopy(
    const muscles::HillDeGrooteType &other)
{
    muscles::HillType::DeepCopy(other);
}

void muscles::HillDeGrooteType::computeFlPE()
{
    utils::Scalar kpe = 4;
    utils::Scalar e0 = 0.6;
    utils::Scalar normLength = position().length() / characteristics().optimalLength();

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(normLength, 1),
                (exp( (kpe * (normLength-1)) / e0) -1)
                                      /
                                  (exp( kpe ) - 1));
#else

    if (normLength > 1) {
        *m_FlPE = (exp( (kpe * (normLength-1)) / e0) -1)
                      /
                  (exp( kpe ) - 1);
    } else {
        *m_FlPE = 0;
    }
#endif
}

void muscles::HillDeGrooteType::computeFvCE()
{

    utils::Scalar d1 = -0.318;
    utils::Scalar d2 = -8.149;
    utils::Scalar d3 = -0.374;
    utils::Scalar d4 = 0.886;
    utils::Scalar norm_v = m_position->velocity() / *m_cste_maxShorteningSpeed;

    *m_FvCE = d1 * std::log(
            (d2 * norm_v + d3) + std::sqrt(( d2 * norm_v + d3)*( d2 * norm_v + d3) + 1)
        ) + d4;
}

void muscles::HillDeGrooteType::computeFlCE(
    const muscles::State&)
{
    utils::Scalar b11 = 0.815;
    utils::Scalar b21 = 1.055;
    utils::Scalar b31 = 0.162;
    utils::Scalar b41 = 0.063;
    utils::Scalar b12 = 0.433;
    utils::Scalar b22 = 0.717;
    utils::Scalar b32 = -0.030;
    utils::Scalar b42 = 0.200;
    utils::Scalar b13 = 0.100;
    utils::Scalar b23 = 1.000;
    utils::Scalar b33 = 0.354;
    utils::Scalar b43 = 0.0;
    utils::Scalar normLength = position().length() / characteristics().optimalLength();

    *m_FlCE = b11 * exp((-0.5*((normLength-b21)*(normLength-b21)))
                       /
                       ((b31 + b41*normLength)*(b31 + b41*normLength)))
            +
            b12 * exp((-0.5*((normLength-b22)*(normLength-b22)))
                        /
                        ((b32 + b42*normLength)*(b32 + b42*normLength)))
            +
            b13 * exp((-0.5*((normLength-b23)*(normLength-b23)))
                        /
                        ((b33 + b43*normLength)*(b33 + b43*normLength)));
}

void muscles::HillDeGrooteType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_DE_GROOTE;
}
