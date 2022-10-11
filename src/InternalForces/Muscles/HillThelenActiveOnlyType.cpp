#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillThelenActiveOnlyType.h"

#include <math.h>
#include "Utils/String.h"
#include "InternalForces/Muscles/Geometry.h"
#include "InternalForces/Muscles/Characteristics.h"

using namespace BIORBD_NAMESPACE;
using namespace internalforce;

muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType() :
    muscles::HillThelenType()
{
    setType();
}
muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics) :
    muscles::HillThelenType (name, geometry, characteristics)
{
    setType();
}

muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg) :
    muscles::HillThelenType (name, geometry, characteristics, emg)
{
    setType();
}

muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const PathModifiers &pathModifiers) :
    muscles::HillThelenType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const PathModifiers &pathModifiers,
    const muscles::State& emg) :
    muscles::HillThelenType (name, geometry, characteristics, pathModifiers,
                                     emg)
{
    setType();
}

muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const muscles::Muscle &other) :
    muscles::HillThelenType (other)
{

}

muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const std::shared_ptr<muscles::Muscle>
    other) :
    muscles::HillThelenType(other)
{

}

muscles::HillThelenActiveOnlyType
muscles::HillThelenActiveOnlyType::DeepCopy() const
{
    muscles::HillThelenActiveOnlyType copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::HillThelenActiveOnlyType::DeepCopy(
    const muscles::HillThelenActiveOnlyType &other)
{
    muscles::HillThelenType::DeepCopy(other);
}

void muscles::HillThelenActiveOnlyType::computeFlPE()
{
    *m_FlPE = 0;
}

void muscles::HillThelenActiveOnlyType::computeDamping()
{
    *m_damping = 0;
}

void muscles::HillThelenActiveOnlyType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE;
}
