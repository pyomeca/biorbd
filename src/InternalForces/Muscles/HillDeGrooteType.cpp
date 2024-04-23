#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillDeGrooteType.h"

#include <math.h>
#include "Utils/String.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "InternalForces/Muscles/Characteristics.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::HillDeGrooteType::HillDeGrooteType() :
    internal_forces::muscles::HillType()
{
    setType();
}
internal_forces::muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String& name,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics) :
    internal_forces::muscles::HillType (name, geometry, characteristics)
{
    setType();
}

internal_forces::muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State &emg) :
    internal_forces::muscles::HillType (name, geometry, characteristics, emg)
{
    setType();
}

internal_forces::muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::muscles::HillType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

internal_forces::muscles::HillDeGrooteType::HillDeGrooteType(
    const utils::String& name,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    const internal_forces::muscles::State& emg) :
    internal_forces::muscles::HillType (name, geometry, characteristics, pathModifiers, emg)
{
    setType();
}

internal_forces::muscles::HillDeGrooteType::HillDeGrooteType(
    const internal_forces::muscles::Muscle &other) :
    internal_forces::muscles::HillType (other)
{

}

internal_forces::muscles::HillDeGrooteType::HillDeGrooteType(
    const std::shared_ptr<internal_forces::muscles::Muscle> other) :
    internal_forces::muscles::HillType(other)
{

}

internal_forces::muscles::HillDeGrooteType internal_forces::muscles::HillDeGrooteType::DeepCopy()
const
{
    internal_forces::muscles::HillDeGrooteType copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::HillDeGrooteType::DeepCopy(
    const internal_forces::muscles::HillDeGrooteType &other)
{
    internal_forces::muscles::HillType::DeepCopy(other);
}

void internal_forces::muscles::HillDeGrooteType::computeFlPE()
{
    utils::Scalar kpe = 4;
    utils::Scalar e0 = 0.6;
    utils::Scalar normLength = position().length() / characteristics().optimalLength();
    utils::Scalar t5 = exp( kpe * (normLength - 1) / e0);
    utils::Scalar t7 = exp(kpe);

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else_zero(
        IF_ELSE_NAMESPACE::gt(normLength, 1), (t5 - 1) / (t7 - 1)
    );
#else
    *m_FlPE = normLength > 1 ? (t5 - 1) / (t7 - 1) : 0;
#endif
}

void internal_forces::muscles::HillDeGrooteType::computeFvCE()
{

    utils::Scalar d1 = -0.318;
    utils::Scalar d2 = -8.149;
    utils::Scalar d3 = -0.374;
    utils::Scalar d4 = 0.886;
    utils::Scalar norm_v = m_position->velocity() / this->characteristics().maxShorteningSpeed();

    *m_FvCE = d1 * std::log(
            (d2 * norm_v + d3) + std::sqrt(( d2 * norm_v + d3)*( d2 * norm_v + d3) + 1)
        ) + d4;
}

void internal_forces::muscles::HillDeGrooteType::computeFlCE(
    const internal_forces::muscles::State&)
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

void internal_forces::muscles::HillDeGrooteType::setType()
{
    *m_type = internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE;
}
