#define BIORBD_API_EXPORTS
#include "s2mIMU.h"

s2mIMU::s2mIMU(
        const biorbd::utils::Attitude &v,
        const s2mString &name,
        const s2mString &parentName,
        const bool &tech,
        const bool &ana,
        const int &id) :
    s2mNodeAttitude(v, name, parentName),
    m_technical(tech),
    m_anatomical(ana),
    m_id(id)
{
    //ctor
}

s2mIMU::~s2mIMU()
{
    //dtor
}




bool s2mIMU::isAnatomical() const
{
    return m_anatomical;
}



bool s2mIMU::isTechnical() const
{
    return m_technical;
}


int s2mIMU::parentId() const
{
    return m_id;
}
