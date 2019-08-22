#define BIORBD_API_EXPORTS
#include "Utils/Node3d.h"

#include "Utils/Attitude.h"
#include "Utils/Vector.h"

biorbd::utils::Node3d::Node3d() :
    Eigen::Vector3d (0, 0, 0),
    biorbd::utils::Node ()
{

}

biorbd::utils::Node3d::Node3d(double x, double y, double z) :
    Eigen::Vector3d (x, y, z),
    biorbd::utils::Node ()
{

}

biorbd::utils::Node3d::Node3d(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    Eigen::Vector3d (x, y, z),
    biorbd::utils::Node (name, parentName)
{

}

biorbd::utils::Node3d biorbd::utils::Node3d::DeepCopy() const
{
    return biorbd::utils::Node3d(*this, this->name(), this->parent());
}

void biorbd::utils::Node3d::setPosition(const biorbd::utils::Node3d &n)
{
    this->block(0,0,3,1) = n.block(0,0,3,1);
}

void biorbd::utils::Node3d::setPosition(Eigen::Vector3d &n)
{
    this->block(0,0,3,1) = n;
}

void biorbd::utils::Node3d::setPosition(Eigen::Vector4d &n)
{
    this->block(0,0,3,1) = n.block(0,0,3,1);
}

const biorbd::utils::Node3d &biorbd::utils::Node3d::position() const
{
    return *this;
}

void biorbd::utils::Node3d::applyRT(const Attitude &a){
    Eigen::Vector4d tp;
    tp.block(0,0,3,1) = static_cast<Eigen::Vector3d>(*this);
    tp(3) = 1;
    tp = a.eigen() * tp;
    this->setPosition(tp);
}

const biorbd::utils::Node3d biorbd::utils::Node3d::operator-(const biorbd::utils::Node3d &other) const
{
    biorbd::utils::Node3d result (this->block(0,0,3,1)-other.block(0,0,3,1));
    return result;
}

const biorbd::utils::Node3d biorbd::utils::Node3d::operator*(double a) const
{
    biorbd::utils::Node3d result(*this);
    for (int i=0; i<3; ++i)
        result[i] *= a;
    return result;
}

const biorbd::utils::Node3d biorbd::utils::Node3d::operator/(double a) const
{
    biorbd::utils::Node3d result(*this);
    for (int i=0; i<3; ++i)
        result[i] /= a;
    return result;
}
