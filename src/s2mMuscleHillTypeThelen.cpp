#define BIORBD_API_EXPORTS
#include "s2mMuscleHillTypeThelen.h"

s2mMuscleHillTypeThelen::s2mMuscleHillTypeThelen(const s2mMuscleGeometry& g,
                                                const s2mMuscleCaracteristics& c,
                                                const s2mMusclePathChangers & w,
                                                const s2mMuscleStateDynamics & s) :
    s2mMuscleHillType(g,c,w,s)
{
    setType();
}
s2mMuscleHillTypeThelen::s2mMuscleHillTypeThelen(const s2mString& n,
                                                const s2mMuscleGeometry& g,
                                                const s2mMuscleCaracteristics& c,
                                                const s2mMusclePathChangers & w,
                                                const s2mMuscleStateDynamics & s) :
    s2mMuscleHillType(n,g,c,w,s)
{
    setType();
}

s2mMuscleHillTypeThelen::s2mMuscleHillTypeThelen(const s2mMuscle &m) :
    s2mMuscleHillType (m)
{
    setType();
}

s2mMuscleHillTypeThelen::s2mMuscleHillTypeThelen(const std::shared_ptr<s2mMuscle> m):
    s2mMuscleHillType (m)
{
    setType();
}

<<<<<<< HEAD
void s2mMuscleHillTypeThelen::computeFlPE(){
    if (m_position.length() > caract().tendonSlackLength())
        m_FlPE = (exp(m_cste_FlPE_1*(m_position.length()/caract().optimalLength()-1)) -1)/(exp(m_cste_FlPE_2)-1);
    else
        m_FlPE = 0;
    }

void s2mMuscleHillTypeThelen::computeFlCE(const s2mMuscleStateActual&){
=======
s2mMuscleHillTypeThelen::~s2mMuscleHillTypeThelen()
{

}

void s2mMuscleHillTypeThelen::computeFlCE(const s2mMuscleStateDynamics&){
>>>>>>> stringCopyReducer
    m_FlCE = exp( -pow(((m_position.length() / caract().optimalLength())-1), 2 ) /  m_cste_FlCE_2 ); //Thelen2003, le 26 fevrier 2018
}

void s2mMuscleHillTypeThelen::setType()
{
    m_type = "HillThelen";
}
