#define BIORBD_API_EXPORTS
#include "Utils/NodeAttitude.h"

#include <Eigen/Dense>
#include "Utils/String.h"

biorbd::utils::NodeAttitude::NodeAttitude(
        const Attitude &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) : // Nom du parent
    biorbd::utils::Attitude(v),
    m_parentName(std::make_shared<biorbd::utils::String>(parentName)),
    m_RTName(std::make_shared<biorbd::utils::String>(name))
{

}

biorbd::utils::NodeAttitude::~NodeAttitude()
{
    //dtor
}

const biorbd::utils::String &biorbd::utils::NodeAttitude::parent() const
{
    return *m_parentName;
}

void biorbd::utils::NodeAttitude::setParent(const biorbd::utils::String &parentName)
{
    *m_parentName = parentName;
}

void biorbd::utils::NodeAttitude::setAttitude(const biorbd::utils::Attitude &n)
{
    this->block(0,0,4,4) = n.block(0,0,4,4);
}


const biorbd::utils::Attitude &biorbd::utils::NodeAttitude::attitude() const
{
    return *this;
}


void biorbd::utils::NodeAttitude::setName(const biorbd::utils::String &name)
{
    *m_RTName = name;
}

const biorbd::utils::String &biorbd::utils::NodeAttitude::name() const
{
    return *m_RTName;
}
