#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedCoordinateRange.h"

biorbd::rigidbody::GeneralizedCoordinateRange::GeneralizedCoordinateRange(
        double min,
        double max) :
    m_min(std::make_shared<double>(min)),
    m_max(std::make_shared<double>(max))
{

}

biorbd::rigidbody::GeneralizedCoordinateRange
biorbd::rigidbody::GeneralizedCoordinateRange::DeepCopy() const
{
    biorbd::rigidbody::GeneralizedCoordinateRange copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::GeneralizedCoordinateRange::DeepCopy(
        const biorbd::rigidbody::GeneralizedCoordinateRange &other)
{
    *m_min = *other.m_min;
    *m_max = *other.m_max;
}

void biorbd::rigidbody::GeneralizedCoordinateRange::setMin(
        double min)
{
    *m_min = min;
}

double biorbd::rigidbody::GeneralizedCoordinateRange::min() const
{
    return *m_min;
}

void biorbd::rigidbody::GeneralizedCoordinateRange::setMax(
        double max)
{
    *m_max = max;
}

double biorbd::rigidbody::GeneralizedCoordinateRange::max() const
{
    return *m_max;
}
