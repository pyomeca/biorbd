#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillDeGrooteActiveOnlyType.h"

#include <math.h>
#include "Utils/String.h"
#include "InternalForces/Muscles/Geometry.h"
#include "InternalForces/Muscles/Characteristics.h"

using namespace BIORBD_NAMESPACE;
internalforce::muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType() :
    internalforce::muscles::HillDeGrooteType()
{
    setType();
}
internalforce::muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics) :
    internalforce::muscles::HillDeGrooteType (name, geometry, characteristics)
{
    setType();
}

internalforce::muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::muscles::State &emg) :
    internalforce::muscles::HillDeGrooteType (name, geometry, characteristics, emg)
{
    setType();
}

internalforce::muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::PathModifiers &pathModifiers) :
    internalforce::muscles::HillDeGrooteType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

internalforce::muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics,
    const internalforce::PathModifiers &pathModifiers,
    const internalforce::muscles::State& emg) :
    internalforce::muscles::HillDeGrooteType (name, geometry, characteristics, pathModifiers,
                                     emg)
{
    setType();
}

internalforce::muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const internalforce::muscles::Muscle &other) :
    internalforce::muscles::HillDeGrooteType (other)
{

}

internalforce::muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const std::shared_ptr<internalforce::muscles::Muscle>
    other) :
    internalforce::muscles::HillDeGrooteType(other)
{

}

internalforce::muscles::HillDeGrooteActiveOnlyType
internalforce::muscles::HillDeGrooteActiveOnlyType::DeepCopy() const
{
    internalforce::muscles::HillDeGrooteActiveOnlyType copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::HillDeGrooteActiveOnlyType::DeepCopy(
    const internalforce::muscles::HillDeGrooteActiveOnlyType &other)
{
    internalforce::muscles::HillDeGrooteType::DeepCopy(other);
}

void internalforce::muscles::HillDeGrooteActiveOnlyType::computeFlPE()
{
    *m_FlPE = 0;
}

void internalforce::muscles::HillDeGrooteActiveOnlyType::computeDamping()
{
    *m_damping = 0;
}

void internalforce::muscles::HillDeGrooteActiveOnlyType::setType()
{
    *m_type = internalforce::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE;
}
