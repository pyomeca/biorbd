#define BIORBD_API_EXPORTS
#include "../include/s2mWrappingSphere.h"


s2mWrappingSphere::s2mWrappingSphere(const double &dia,
                                     const Eigen::Vector3d &v, // Position du noeud
                                     const s2mString &name,  // Nom du noeud
                                     const s2mString &parentName) :
    s2mWrappingObject(v,name,parentName),
    m_dia(dia)
{
    m_forme = "Sphere";
}


s2mWrappingSphere::~s2mWrappingSphere()
{
    //dtor
}

double s2mWrappingSphere::size() const
{
    return m_dia;
}

void s2mWrappingSphere::setSize(const double &val)
{
    m_dia = val;
}
