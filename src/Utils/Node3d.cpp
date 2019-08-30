#define BIORBD_API_EXPORTS
#include "Utils/Node3d.h"

#include "Utils/RotoTrans.h"
#include "Utils/Vector.h"

biorbd::utils::Node3d::Node3d() :
    Eigen::Vector3d (),
    biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Node3d::Node3d(double x, double y, double z) :
    Eigen::Vector3d (x, y, z),
    biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Node3d::Node3d(const Eigen::Vector4d &v) :
    Eigen::Vector3d(v[0], v[1], v[2]), biorbd::utils::Node ()
{
    setType();
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
    setType();
}

biorbd::utils::Node3d biorbd::utils::Node3d::DeepCopy() const
{
    biorbd::utils::Node3d copy;
    static_cast<Eigen::Vector3d&>(copy) = *this;
    copy.biorbd::utils::Node::DeepCopy(*this);
    return copy;
}

void biorbd::utils::Node3d::DeepCopy(const Node3d &other)
{
    *this = static_cast<Eigen::Vector3d>(other);
    biorbd::utils::Node::DeepCopy(other);
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

void biorbd::utils::Node3d::setType()
{
    *m_typeOfNode = biorbd::utils::NODE3D;
}
