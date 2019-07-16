#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleHillTypeChadwick.h"


s2mMuscleHillTypeChadwick::s2mMuscleHillTypeChadwick(const s2mString &s) :
    s2mMuscleHillType(s)
{
    setType();
}

s2mMuscleHillTypeChadwick::s2mMuscleHillTypeChadwick(const s2mMuscleGeometry &g,
                                                     const s2mMuscleCaracteristics &c,
                                                     const s2mMusclePathChangers &w,
                                                     const s2mMuscleStateActual &s) :
    s2mMuscleHillType(g,c,w,s)
{
    setType();
}

s2mMuscleHillTypeChadwick::s2mMuscleHillTypeChadwick(const s2mString &n,
                                                     const s2mMuscleGeometry &g,
                                                     const s2mMuscleCaracteristics &c,
                                                     const s2mMusclePathChangers &w,
                                                     const s2mMuscleStateActual &s) :
    s2mMuscleHillType(n,g,c,w,s)
{
    setType();
}

s2mMuscleHillTypeChadwick::s2mMuscleHillTypeChadwick(const s2mMuscle &m):
    s2mMuscleHillType (m)
{
    setType();
}

s2mMuscleHillTypeChadwick::s2mMuscleHillTypeChadwick(const std::shared_ptr<s2mMuscle> m):
    s2mMuscleHillType (m)
{
    setType();
}

s2mMuscleHillTypeChadwick::~s2mMuscleHillTypeChadwick()
{
    setType();
}

void s2mMuscleHillTypeChadwick::setType()
{
    m_type = "HillChadwick";
}
