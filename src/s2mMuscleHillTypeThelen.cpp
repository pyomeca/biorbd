#include "../include/s2mMuscleHillTypeThelen.h"

s2mMuscleHillTypeThelen::s2mMuscleHillTypeThelen(const s2mMuscleGeometry& g,
                                                const s2mMuscleCaracteristics& c,
                                                const s2mMusclePathChangers & w,
                                                const s2mMuscleStateActual & s) :
    s2mMuscleHillType(g,c,w,s)
{
    setType();
}
s2mMuscleHillTypeThelen::s2mMuscleHillTypeThelen(const s2mString& n,
                                                const s2mMuscleGeometry& g,
                                                const s2mMuscleCaracteristics& c,
                                                const s2mMusclePathChangers & w,
                                                const s2mMuscleStateActual & s) :
    s2mMuscleHillType(n,g,c,w,s)
{
    setType();
}

void s2mMuscleHillTypeThelen::computeFlCE(const s2mMuscleStateActual&){
    m_FlCE = exp( -pow(((m_position.length() / caract().optimalLength())-1), 2 ) /  m_cste_FlCE_2 ); //Thelen2003, le 26 fevrier 2018
}
