#define BIORBD_API_EXPORTS
#include "Muscles/HillDeGrooteActiveOnlyType.h"

#include <math.h>
#include "Utils/String.h"
#include "Muscles/Geometry.h"
#include "Muscles/Characteristics.h"

using namespace BIORBD_NAMESPACE;

muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType() :
    muscles::HillDeGrooteType()
{
    setType();
}
muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics) :
    muscles::HillDeGrooteType (name, geometry, characteristics)
{
    setType();
}

muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg) :
    muscles::HillDeGrooteType (name, geometry, characteristics, emg)
{
    setType();
}

muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers) :
    muscles::HillDeGrooteType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers &pathModifiers,
    const muscles::State& emg) :
    muscles::HillDeGrooteType (name, geometry, characteristics, pathModifiers,
                                     emg)
{
    setType();
}

muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const muscles::Muscle &other) :
    muscles::HillDeGrooteType (other)
{

}

muscles::HillDeGrooteActiveOnlyType::HillDeGrooteActiveOnlyType(
    const std::shared_ptr<muscles::Muscle>
    other) :
    muscles::HillDeGrooteType(other)
{

}

muscles::HillDeGrooteActiveOnlyType
muscles::HillDeGrooteActiveOnlyType::DeepCopy() const
{
    muscles::HillDeGrooteActiveOnlyType copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::HillDeGrooteActiveOnlyType::DeepCopy(
    const muscles::HillDeGrooteActiveOnlyType &other)
{
    muscles::HillDeGrooteType::DeepCopy(other);
}

void muscles::HillDeGrooteActiveOnlyType::computeFlPE()
{
    *m_FlPE = 0;
}

void muscles::HillDeGrooteActiveOnlyType::computeDamping()
{
    *m_damping = 0;
}

void muscles::HillDeGrooteActiveOnlyType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE;
}
