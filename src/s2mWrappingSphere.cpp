#define BIORBD_API_EXPORTS
#include "../include/s2mWrappingSphere.h"


s2mWrappingSphere::s2mWrappingSphere(const double &dia,
                                     const Eigen::Vector3d &v, // Position du noeud
                                     const s2mString &name,  // Nom du noeud
                                     const s2mString &parentName) :
    s2mWrappingObject(v,name,parentName),
    m_dia(dia)
{
    //ctor
}


s2mWrappingSphere::~s2mWrappingSphere()
{
    //dtor
}
