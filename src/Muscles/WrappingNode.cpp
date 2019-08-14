#define BIORBD_API_EXPORTS
#include "Muscles/WrappingNode.h"

s2mNodeWrap::s2mNodeWrap(
        const Eigen::Vector3d &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node(v, name,parentName)
{
}

s2mNodeWrap::~s2mNodeWrap()
{
    //dtor
}
