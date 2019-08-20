#define BIORBD_API_EXPORTS
#include "Utils/NodeAttitude.h"

#include <Eigen/Dense>
#include "Utils/String.h"

biorbd::utils::NodeAttitude::NodeAttitude() :
    biorbd::utils::Attitude(),
    biorbd::utils::Node()
{

}

biorbd::utils::NodeAttitude::NodeAttitude(const biorbd::utils::Attitude &attitude) :
    biorbd::utils::Attitude(attitude),
    biorbd::utils::Node()
{

}

biorbd::utils::NodeAttitude::NodeAttitude(
        const Attitude &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Attitude(v),
    biorbd::utils::Node(name, parentName)
{

}

biorbd::utils::NodeAttitude biorbd::utils::NodeAttitude::DeepCopy()
{
    return biorbd::utils::NodeAttitude(this->attitude(), this->name(), this->parent());
}

void biorbd::utils::NodeAttitude::setAttitude(const biorbd::utils::Attitude &n)
{
    this->block(0,0,4,4) = n.block(0,0,4,4);
}


const biorbd::utils::Attitude &biorbd::utils::NodeAttitude::attitude() const
{
    return *this;
}
