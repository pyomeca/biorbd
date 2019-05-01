#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleHillTypeSchutte.h"


s2mMuscleHillTypeSchutte::s2mMuscleHillTypeSchutte(const s2mString &s) :
    s2mMuscleHillType(s)
{
    setType();
}

s2mMuscleHillTypeSchutte::s2mMuscleHillTypeSchutte(const s2mMuscleGeometry &g,
                                                   const s2mMuscleCaracteristics &c,
                                                   const s2mMusclePathChangers &w,
                                                   const s2mMuscleStateActual &s) :
    s2mMuscleHillType(g,c,w,s)
{
    setType();
}

s2mMuscleHillTypeSchutte::s2mMuscleHillTypeSchutte(const s2mString &n,
                                                   const s2mMuscleGeometry &g,
                                                   const s2mMuscleCaracteristics &c,
                                                   const s2mMusclePathChangers &w,
                                                   const s2mMuscleStateActual &s) :
    s2mMuscleHillType(n,g,c,w,s)
{
    setType();
}

s2mMuscleHillTypeSchutte::s2mMuscleHillTypeSchutte(const s2mMuscle &m) :
    s2mMuscleHillType (m)
{

}

s2mMuscleHillTypeSchutte::s2mMuscleHillTypeSchutte(const std::shared_ptr<s2mMuscle> m) :
    s2mMuscleHillType (m)
{

}

s2mMuscleHillTypeSchutte::~s2mMuscleHillTypeSchutte()
{

}

void s2mMuscleHillTypeSchutte::setType()
{
    m_type = "HillSchutte";
}
