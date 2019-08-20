#define BIORBD_API_EXPORTS
#include "Muscles/WrappingSphere.h"

#include "Utils/Attitude.h"

biorbd::muscles::WrappingSphere::WrappingSphere(
        double x,
        double y,
        double z,
        double diameter,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::muscles::WrappingObject(x, y, z, name, parentName),
    m_dia(diameter)
{
    m_forme = "Sphere";
}

biorbd::muscles::WrappingSphere::WrappingSphere(
        const biorbd::utils::Node &v,
        double diameter, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    biorbd::muscles::WrappingObject(v, name, parentName),
    m_dia(diameter)
{
    m_forme = "Sphere";
}


biorbd::muscles::WrappingSphere::~WrappingSphere()
{
    //dtor
}

biorbd::utils::Attitude biorbd::muscles::WrappingSphere::RT(
        biorbd::rigidbody::Joints &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        bool )
{
    return biorbd::utils::Attitude();
}

double biorbd::muscles::WrappingSphere::size() const
{
    return m_dia;
}

void biorbd::muscles::WrappingSphere::setSize(double val)
{
    m_dia = val;
}
