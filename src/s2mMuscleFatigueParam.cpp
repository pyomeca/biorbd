#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueParam.h"

s2mMuscleFatigueParam::s2mMuscleFatigueParam(double param1) :
    m_param1(param1)
{

}

double s2mMuscleFatigueParam::param1() const
{
    return m_param1;
}

void s2mMuscleFatigueParam::param1(double param1)
{
    m_param1 = param1;
}
