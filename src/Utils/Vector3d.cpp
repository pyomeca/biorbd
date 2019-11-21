#define BIORBD_API_EXPORTS
#include "Utils/Vector3d.h"

#include "Utils/RotoTrans.h"
#include "Utils/Vector.h"

biorbd::utils::Vector3d::Vector3d() :
    Eigen::Vector3d (),
    biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Vector3d::Vector3d(double x, double y, double z) :
    Eigen::Vector3d (x, y, z),
    biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Vector3d::Vector3d(const Eigen::Vector4d &v) :
    Eigen::Vector3d(v[0], v[1], v[2]), biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Vector3d::Vector3d(
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

biorbd::utils::Vector3d biorbd::utils::Vector3d::DeepCopy() const
{
    biorbd::utils::Vector3d copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::utils::Vector3d::DeepCopy(const Vector3d &other)
{
    *this = static_cast<Eigen::Vector3d>(other);
    biorbd::utils::Node::DeepCopy(other);
}

biorbd::utils::Vector3d biorbd::utils::Vector3d::applyRT(const biorbd::utils::RotoTrans &rt) const
{
    Eigen::Vector4d v;
    v.block(0, 0, 3, 1) << *this;
    v[3] = 1;
    return (rt * v).block(0, 0, 3, 1);
}

void biorbd::utils::Vector3d::applyRT(const biorbd::utils::RotoTrans &rt){
    Eigen::Vector4d v;
    v.block(0, 0, 3, 1) << *this;
    v[3] = 1;
    *this = (rt * v).block(0, 0, 3, 1);
}

biorbd::utils::Vector3d &biorbd::utils::Vector3d::operator=(const Eigen::Vector4d &v){
    this->Eigen::Vector3d::operator=(biorbd::utils::Vector3d(v));
    return *this;
}

void biorbd::utils::Vector3d::setType()
{
    *m_typeOfNode = biorbd::utils::VECTOR3D;
}
