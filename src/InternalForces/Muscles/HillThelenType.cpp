#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillThelenType.h"

#include <math.h>
#include "Utils/String.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "InternalForces/Muscles/Characteristics.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::HillThelenType::HillThelenType() :
    internal_forces::muscles::HillType()
{
    setType();
}
internal_forces::muscles::HillThelenType::HillThelenType(
    const utils::String& name,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics) :
    internal_forces::muscles::HillType (name, geometry, characteristics)
{
    setType();
}

internal_forces::muscles::HillThelenType::HillThelenType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State &emg) :
    internal_forces::muscles::HillType (name, geometry, characteristics, emg)
{
    setType();
}

internal_forces::muscles::HillThelenType::HillThelenType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::muscles::HillType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

internal_forces::muscles::HillThelenType::HillThelenType(
    const utils::String& name,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    const internal_forces::muscles::State& emg) :
    internal_forces::muscles::HillType (name, geometry, characteristics, pathModifiers, emg)
{
    setType();
}

internal_forces::muscles::HillThelenType::HillThelenType(
    const internal_forces::muscles::Muscle &other) :
    internal_forces::muscles::HillType (other)
{

}

internal_forces::muscles::HillThelenType::HillThelenType(
    const std::shared_ptr<internal_forces::muscles::Muscle> other) :
    internal_forces::muscles::HillType(other)
{

}

internal_forces::muscles::HillThelenType internal_forces::muscles::HillThelenType::DeepCopy()
const
{
    internal_forces::muscles::HillThelenType copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::HillThelenType::DeepCopy(
    const internal_forces::muscles::HillThelenType &other)
{
    internal_forces::muscles::HillType::DeepCopy(other);
}

void internal_forces::muscles::HillThelenType::computeFlPE()
{
    utils::Scalar normLength = position().length()  / characteristics().optimalLength();
    utils::Scalar kpe = 5.0;
    utils::Scalar e0 = 0.6;
    utils::Scalar t5 = exp(kpe * (normLength - 1) / e0);
    utils::Scalar t7 = exp(kpe);

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else_zero(
        IF_ELSE_NAMESPACE::gt(normLength, 1), ((t5 - 1) / (t7 - 1))
    );
#else
    *m_FlPE = normLength > 1 ? (t5 - 1) / (t7 - 1) : 0;
#endif
}

void internal_forces::muscles::HillThelenType::computeFlCE(
    const internal_forces::muscles::State&)
{
    utils::Scalar normLength = position().length() / characteristics().optimalLength();
    *m_FlCE = exp( -((normLength - 1)*(normLength - 1)) /  0.45 );
}

void internal_forces::muscles::HillThelenType::computeFvCE()
{
	utils::Scalar v = m_position->velocity();
    utils::Scalar norm_v = v / (characteristics().optimalLength() * this->characteristics().maxShorteningSpeed());
    utils::Scalar kvce = 0.06;
    utils::Scalar flen = 1.6;

    // WARNING CONCENTRIC IS NOT FROM THELEN (see header file for precisions)
    utils::Scalar a = 3.0 / 11.0;
    utils::Scalar b = 3.0 / 11.0;

    
#ifdef BIORBD_USE_CASADI_MATH
    *m_FvCE = 
        IF_ELSE_NAMESPACE::if_else(IF_ELSE_NAMESPACE::ge(norm_v, 0), 
            ((1 + norm_v * flen / kvce) / (1 + norm_v / kvce)), // If norm_v >= 0
            IF_ELSE_NAMESPACE::if_else(IF_ELSE_NAMESPACE::ge(norm_v, -1), 
                (1 + a) * b / (-norm_v + b) - a, // if norm_v >= -1
                0 // if norm_v < -1
            )
        );
#else
    
    if (norm_v >= 0){
        *m_FvCE = (1 + norm_v * flen / kvce) / (1 + norm_v / kvce);
    } else if (norm_v >= -1) {
        *m_FvCE = (1 + a) * b / (-norm_v + b) - a;
    }
    else {
        *m_FvCE = 0;
    }
#endif
}
void internal_forces::muscles::HillThelenType::setType()
{
    *m_type = internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN;
}
