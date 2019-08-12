#define BIORBD_API_EXPORTS
#include "Utils/Node.h"

#include "Utils/Attitude.h"

biorbd::utils::Node::Node(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    Eigen::Vector3d (x, y, z),
    m_parentName(parentName),
    m_markName(name)
{

}

biorbd::utils::Node::Node(
        const Eigen::Vector3d &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) : // Nom du parent
    Eigen::Vector3d(v),
    m_parentName(parentName),
    m_markName(name)
{

}

const biorbd::utils::String& biorbd::utils::Node::parent() const
{
    return m_parentName;
}

void biorbd::utils::Node::setParent(const biorbd::utils::String &parentName)
{
    m_parentName = parentName;
}

void biorbd::utils::Node::setPosition(const biorbd::utils::Node &n)
{
    this->block(0,0,3,1) = n.block(0,0,3,1);
}

void biorbd::utils::Node::setPosition(Eigen::Vector3d &n)
{
    this->block(0,0,3,1) = n;
}

void biorbd::utils::Node::setPosition(Eigen::Vector4d &n)
{
    this->block(0,0,3,1) = n.block(0,0,3,1);
}

const biorbd::utils::Node &biorbd::utils::Node::position() const
{
    return *this;
}

void biorbd::utils::Node::applyRT(const Attitude &a){
    Eigen::Vector4d tp;
    tp.block(0,0,3,1) = static_cast<Eigen::Vector3d>(*this);
    tp(3) = 1;
    tp = static_cast<Eigen::Matrix4d>(a) * tp;
    this->setPosition(tp);
}

Eigen::Vector3d biorbd::utils::Node::vector() const{
    return  this->block(0,0,3,1);
}

void biorbd::utils::Node::setName(const biorbd::utils::String &name)
{
    m_markName = name;
}

const biorbd::utils::String &biorbd::utils::Node::name() const
{
    return m_markName;
}

const biorbd::utils::Node biorbd::utils::Node::operator-(const biorbd::utils::Node &other) const{
    biorbd::utils::Node result (this->block(0,0,3,1)-other.block(0,0,3,1));
    return result;
}

const biorbd::utils::Node biorbd::utils::Node::operator*(double a) const
{
    biorbd::utils::Node result(*this);
    for (int i=0; i<3; ++i)
        result[i] *= a;
    return result;
}

const biorbd::utils::Node biorbd::utils::Node::operator/(double a) const
{
    biorbd::utils::Node result(*this);
    for (int i=0; i<3; ++i)
        result[i] /= a;
    return result;
}
