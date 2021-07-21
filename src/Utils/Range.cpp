#define BIORBD_API_EXPORTS
#include "Utils/Range.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

utils::Range::Range(
    double min,
    double max) :
    m_min(std::make_shared<double>(min)),
    m_max(std::make_shared<double>(max))
{

}

utils::Range
utils::Range::DeepCopy() const
{
    utils::Range copy;
    copy.DeepCopy(*this);
    return copy;
}

void utils::Range::DeepCopy(
    const utils::Range &other)
{
    *m_min = *other.m_min;
    *m_max = *other.m_max;
}

void utils::Range::setMin(
    double min)
{
    *m_min = min;
}

double utils::Range::min() const
{
    return *m_min;
}

void utils::Range::setMax(
    double max)
{
    *m_max = max;
}

double utils::Range::max() const
{
    return *m_max;
}
