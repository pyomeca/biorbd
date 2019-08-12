#define BIORBD_API_EXPORTS
#include "s2mWrappingSphere.h"

#include "Utils/Attitude.h"

s2mWrappingSphere::s2mWrappingSphere(
        const double &dia,
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    s2mWrappingObject(v,name,parentName),
    m_dia(dia)
{
    m_forme = "Sphere";
}


s2mWrappingSphere::~s2mWrappingSphere()
{
    //dtor
}

biorbd::utils::Attitude s2mWrappingSphere::RT(
        s2mJoints &,
        const biorbd::utils::GenCoord &,
        const bool &)
{
    return biorbd::utils::Attitude();
}

double s2mWrappingSphere::size() const
{
    return m_dia;
}

void s2mWrappingSphere::setSize(const double &val)
{
    m_dia = val;
}
