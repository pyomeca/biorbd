#define BIORBD_API_EXPORTS
#include "s2mBoneMesh.h"

#include "Utils/Node.h"
#include "s2mPatch.h"

s2mBoneMesh::s2mBoneMesh(
        const std::vector<biorbd::utils::Node> &mesh,
        const std::vector<s2mPatch> & v) :
    m_pathFile(""),
    m_mesh(mesh),
    m_patch(v)
{

}
s2mBoneMesh::~s2mBoneMesh(){

}
void s2mBoneMesh::addPoint(const biorbd::utils::Node &node){
    m_mesh.push_back(node);
}
const biorbd::utils::Node &s2mBoneMesh::point(unsigned int i) const
{
    return *(m_mesh.begin()+i);
}
unsigned int s2mBoneMesh::size() const
{
    return static_cast<unsigned int>(m_mesh.size());
}

void s2mBoneMesh::setPath(const biorbd::utils::Path& path)
{
    m_pathFile = path;
}

const biorbd::utils::Path &s2mBoneMesh::path() const
{
    return m_pathFile;
}

unsigned int s2mBoneMesh::sizePatch()
{
    return static_cast<unsigned int>(m_patch.size());
}
void s2mBoneMesh::addPatch(const s2mPatch & v)
{
    m_patch.push_back(v);
}
void s2mBoneMesh::addPatch(const Eigen::Vector3i & v)
{
    addPatch(s2mPatch(v));
}
const std::vector<s2mPatch>& s2mBoneMesh::patch() const
{
    return m_patch;
}
const s2mPatch &s2mBoneMesh::patch(unsigned int i) const
{
    return m_patch[i];
}
