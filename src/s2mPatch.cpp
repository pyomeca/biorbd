#include "../include/s2mPatch.h"

s2mPatch::s2mPatch(const Eigen::Vector3i& points) :
    m_patch(points)
{

}

int &s2mPatch::operator()(int i)
{
    return m_patch[i];
}

s2mPatch s2mPatch::patch()
{
    return m_patch;
}

void s2mPatch::patch(const Eigen::Vector3i & pts)
{
    m_patch = pts;
}

void s2mPatch::patch(const s2mPatch &v)
{
    m_patch = v.m_patch;
}

