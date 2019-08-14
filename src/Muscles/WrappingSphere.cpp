#define BIORBD_API_EXPORTS
#include "Muscles/WrappingSphere.h"

#include "Utils/Attitude.h"

biorbd::muscles::WrappingSphere::WrappingSphere(
        const double &dia,
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    biorbd::muscles::WrappingObject(v,name,parentName),
    m_dia(dia)
{
    m_forme = "Sphere";
}


biorbd::muscles::WrappingSphere::~WrappingSphere()
{
    //dtor
}

biorbd::utils::Attitude biorbd::muscles::WrappingSphere::RT(
        s2mJoints &,
        const biorbd::utils::GenCoord &,
        const bool &)
{
    return biorbd::utils::Attitude();
}

double biorbd::muscles::WrappingSphere::size() const
{
    return m_dia;
}

void biorbd::muscles::WrappingSphere::setSize(const double &val)
{
    m_dia = val;
}
