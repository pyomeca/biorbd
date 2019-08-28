#define BIORBD_API_EXPORTS
#include "RigidBody/Patch.h"

biorbd::rigidbody::Patch::Patch(const Eigen::Vector3i& points) :
    m_patch(std::make_shared<Eigen::Vector3i>(points))
{

}

biorbd::rigidbody::Patch biorbd::rigidbody::Patch::DeepCopy() const
{
    biorbd::rigidbody::Patch copy;
    *copy.m_patch = *m_patch;
    return copy;
}

void biorbd::rigidbody::Patch::DeepCopy(const biorbd::rigidbody::Patch &other)
{
    *m_patch = *other.m_patch;
}

int &biorbd::rigidbody::Patch::operator()(int i)
{
    return (*m_patch)[i];
}

biorbd::rigidbody::Patch biorbd::rigidbody::Patch::patch()
{
    return *m_patch;
}

void biorbd::rigidbody::Patch::patch(const Eigen::Vector3i & pts)
{
    *m_patch = pts;
}

void biorbd::rigidbody::Patch::patch(const biorbd::rigidbody::Patch &v)
{
    m_patch = v.m_patch;
}

