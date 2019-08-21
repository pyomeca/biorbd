#define BIORBD_API_EXPORTS
#include "Utils/NodeAttitude.h"

#include <Eigen/Dense>
#include "Utils/String.h"

biorbd::utils::NodeAttitude::NodeAttitude() :
    biorbd::utils::Attitude(),
    biorbd::utils::Node()
{

}

biorbd::utils::NodeAttitude::NodeAttitude(
        double e00, double e01, double e02, double e03,
        double e10, double e11, double e12, double e13,
        double e20, double e21, double e22, double e23,
        double e30, double e31, double e32, double e33) :
    biorbd::utils::Attitude( e00,  e01,  e02,  e03, e10,  e11,  e12,  e13, e20,  e21,  e22,  e23, e30,  e31,  e32,  e33),
    biorbd::utils::Node()
{

}
biorbd::utils::NodeAttitude::NodeAttitude(
        double e00, double e01, double e02, double e03,
        double e10, double e11, double e12, double e13,
        double e20, double e21, double e22, double e23,
        double e30, double e31, double e32, double e33,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Attitude( e00,  e01,  e02,  e03, e10,  e11,  e12,  e13, e20,  e21,  e22,  e23, e30,  e31,  e32,  e33),
    biorbd::utils::Node(name, parentName)
{

}
biorbd::utils::NodeAttitude::NodeAttitude(const biorbd::utils::Attitude &attitude) :
    biorbd::utils::Attitude(attitude),
    biorbd::utils::Node()
{

}

biorbd::utils::NodeAttitude::NodeAttitude(
        const Attitude &attitude,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Attitude(attitude),
    biorbd::utils::Node(name, parentName)
{

}

biorbd::utils::NodeAttitude biorbd::utils::NodeAttitude::DeepCopy() const
{
    return biorbd::utils::NodeAttitude(this->biorbd::utils::Attitude::DeepCopy(), this->name(), this->parent());
}
