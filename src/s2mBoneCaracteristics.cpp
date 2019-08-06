#define BIORBD_API_EXPORTS
#include "../include/s2mBoneCaracteristics.h"

s2mBoneCaracteristics::s2mBoneCaracteristics() :
    Body(),
    m_length(0),
    m_mesh(s2mBoneMesh())
{
}
s2mBoneCaracteristics::s2mBoneCaracteristics(const double &mass,
                                             const s2mNode &com,
                                             const RigidBodyDynamics::Math::Matrix3d &inertia,
                                             const s2mBoneMesh &mesh) :
    Body(mass, com, inertia),
    m_length(0),
    m_mesh(mesh)
{
}

const s2mBoneMesh &s2mBoneCaracteristics::mesh() const {
    return m_mesh;
}

const Eigen::Matrix3d &s2mBoneCaracteristics::inertia() const {
    return mInertia;
}

s2mBoneCaracteristics::~s2mBoneCaracteristics()
{
    //dtor
}

double s2mBoneCaracteristics::length() const {
    return m_length;
}

double s2mBoneCaracteristics::mass() const {
    return mMass;
}

void s2mBoneCaracteristics::setLength(const double &val) {
    m_length = val;
}
