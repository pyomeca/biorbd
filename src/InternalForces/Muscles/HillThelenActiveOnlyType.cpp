#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillThelenActiveOnlyType.h"

#include <math.h>
#include "Utils/String.h"
#include "InternalForces/Muscles/Geometry.h"
#include "InternalForces/Muscles/Characteristics.h"

using namespace BIORBD_NAMESPACE;
internalforce::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType() :
    internalforce::muscles::HillThelenType()
{
    setType();
}
internalforce::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics) :
    internalforce::muscles::HillThelenType (name, geometry, characteristics)
{
    setType();
}

internalforce::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::muscles::State &emg) :
    internalforce::muscles::HillThelenType (name, geometry, characteristics, emg)
{
    setType();
}

internalforce::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::PathModifiers &pathModifiers) :
    internalforce::muscles::HillThelenType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

internalforce::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics,
    const internalforce::PathModifiers &pathModifiers,
    const internalforce::muscles::State& emg) :
    internalforce::muscles::HillThelenType (name, geometry, characteristics, pathModifiers,
                                     emg)
{
    setType();
}

internalforce::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const internalforce::muscles::Muscle &other) :
    internalforce::muscles::HillThelenType (other)
{

}

internalforce::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const std::shared_ptr<internalforce::muscles::Muscle>
    other) :
    internalforce::muscles::HillThelenType(other)
{

}

internalforce::muscles::HillThelenActiveOnlyType
internalforce::muscles::HillThelenActiveOnlyType::DeepCopy() const
{
    internalforce::muscles::HillThelenActiveOnlyType copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::HillThelenActiveOnlyType::DeepCopy(
    const internalforce::muscles::HillThelenActiveOnlyType &other)
{
    internalforce::muscles::HillThelenType::DeepCopy(other);
}

void internalforce::muscles::HillThelenActiveOnlyType::computeFlPE()
{
    *m_FlPE = 0;
}

void internalforce::muscles::HillThelenActiveOnlyType::computeDamping()
{
    *m_damping = 0;
}

void internalforce::muscles::HillThelenActiveOnlyType::setType()
{
    *m_type = internalforce::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE;
}
