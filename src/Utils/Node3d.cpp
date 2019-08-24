#define BIORBD_API_EXPORTS
#include "Utils/Node3d.h"

#include "Utils/RotoTrans.h"
#include "Utils/Vector.h"

biorbd::utils::Node3d::Node3d() :
    Eigen::Vector3d (),
    biorbd::utils::Node ()
{

}

biorbd::utils::Node3d::Node3d(double x, double y, double z) :
    Eigen::Vector3d (x, y, z),
    biorbd::utils::Node ()
{

}

biorbd::utils::Node3d::Node3d(const Eigen::Vector4d &v) :
    Eigen::Vector3d(v[0], v[1], v[2]), biorbd::utils::Node ()
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

biorbd::utils::Node3d biorbd::utils::Node3d::applyRT(const biorbd::utils::RotoTrans &rt) const
{
    Eigen::Vector4d v;
    v.block(0, 0, 3, 1) << *this;
    v[3] = 1;
    return (rt * v).block(0, 0, 3, 1);
}

void biorbd::utils::Node3d::applyRT(const biorbd::utils::RotoTrans &rt){
    Eigen::Vector4d v;
    v.block(0, 0, 3, 1) << *this;
    v[3] = 1;
    *this = (rt * v).block(0, 0, 3, 1);
}

biorbd::utils::Node3d &biorbd::utils::Node3d::operator=(const Eigen::Vector4d &v){
    this->Eigen::Vector3d::operator=(biorbd::utils::Node3d(v));
    return *this;
}
