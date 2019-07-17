#define BIORBD_API_EXPORTS
#include "../include/s2mNode.h"




s2mNode::s2mNode(double x, double y, double z, const s2mString &name, const s2mString &parentName) :
    Eigen::Vector3d (x, y, z),
    m_parentName(parentName),
    m_markName(name)
{

}

s2mNode::s2mNode(const Eigen::Vector3d &v, const s2mString &name, const s2mString &parentName) : // Nom du parent
    Eigen::Vector3d(v),
    m_parentName(parentName),
    m_markName(name)
{

}

s2mNode::~s2mNode()
{
    //dtor
}

s2mString s2mNode::parent() const
{
    return m_parentName;
}

void s2mNode::setParent(const s2mString &parentName)
{
    m_parentName = parentName;
}

void s2mNode::setPosition(const s2mNode &n)
{
    this->block(0,0,3,1) = n.block(0,0,3,1);
}

void s2mNode::setPosition(Eigen::Vector3d &n)
{
    this->block(0,0,3,1) = n;
}

void s2mNode::setPosition(Eigen::Vector4d &n)
{
    this->block(0,0,3,1) = n.block(0,0,3,1);
}

s2mNode s2mNode::position() const
{
    return *this;
}

void s2mNode::applyRT(const s2mAttitude &a){
    Eigen::Vector4d tp;
    tp.block(0,0,3,1) = static_cast<Eigen::Vector3d>(*this);
    tp(3) = 1;
    tp = static_cast<Eigen::Matrix4d>(a) * tp;
    this->setPosition(tp);
}

Eigen::Vector3d s2mNode::vector() const{
    return  this->block(0,0,3,1);
}

void s2mNode::setName(const s2mString &name)
{
    m_markName = name;
}

s2mString s2mNode::name() const
{
    return m_markName;
}

const s2mNode s2mNode::operator-(const s2mNode &other) const{
    s2mNode result (this->block(0,0,3,1)-other.block(0,0,3,1));
    return result;
}

const s2mNode s2mNode::operator*(double a) const
{
    s2mNode result(*this);
    for (int i=0; i<3; ++i)
        result[i] *= a;
    return result;
}

const s2mNode s2mNode::operator/(double a) const
{
    s2mNode result(*this);
    for (int i=0; i<3; ++i)
        result[i] /= a;
    return result;
}
