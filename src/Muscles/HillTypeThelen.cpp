#define BIORBD_API_EXPORTS
#include "Muscles/HillTypeThelen.h"

biorbd::muscles::HillTypeThelen::HillTypeThelen(
        const biorbd::muscles::Geometry& g,
        const biorbd::muscles::Caracteristics& c,
        const biorbd::muscles::PathChangers & w,
        const biorbd::muscles::StateDynamics & s) :
    biorbd::muscles::HillType(g,c,w,s)
{
    setType();
}
biorbd::muscles::HillTypeThelen::HillTypeThelen(
        const biorbd::utils::String& n,
        const biorbd::muscles::Geometry& g,
        const biorbd::muscles::Caracteristics& c,
        const biorbd::muscles::PathChangers & w,
        const biorbd::muscles::StateDynamics & s) :
    biorbd::muscles::HillType(n,g,c,w,s)
{
    setType();
}

biorbd::muscles::HillTypeThelen::HillTypeThelen(const biorbd::muscles::Muscle &m) :
    biorbd::muscles::HillType (m)
{
    setType();
}

biorbd::muscles::HillTypeThelen::HillTypeThelen(const std::shared_ptr<biorbd::muscles::Muscle> m):
    biorbd::muscles::HillType (m)
{
    setType();
}

biorbd::muscles::HillTypeThelen::~HillTypeThelen()
{

}

void biorbd::muscles::HillTypeThelen::computeFlPE(){
    if (m_position.length() > caract().tendonSlackLength())
        m_FlPE = (exp(m_cste_FlPE_1*(m_position.length()/caract().optimalLength()-1)) -1)/(exp(m_cste_FlPE_2)-1); //Thelen2003, le 07 août 2019
    else
        m_FlPE = 0;
}

void biorbd::muscles::HillTypeThelen::computeFlCE(const biorbd::muscles::StateDynamics&){
    m_FlCE = exp( -pow(((m_position.length() / caract().optimalLength())-1), 2 ) /  m_cste_FlCE_2 ); //Thelen2003, le 26 fevrier 2018
}

void biorbd::muscles::HillTypeThelen::setType()
{
    m_type = "HillThelen";
}
