#define BIORBD_API_EXPORTS
#include "RigidBody/BoneMesh.h"

#include "Utils/Node3d.h"
#include "RigidBody/Patch.h"

biorbd::rigidbody::Mesh::Mesh(
        const std::vector<biorbd::utils::Node3d> &mesh,
        const std::vector<biorbd::rigidbody::Patch> & v) :
    m_pathFile(""),
    m_mesh(mesh),
    m_patch(v)
{

}
biorbd::rigidbody::Mesh::~Mesh(){

}
void biorbd::rigidbody::Mesh::addPoint(const biorbd::utils::Node3d &node){
    m_mesh.push_back(node);
}
const biorbd::utils::Node3d &biorbd::rigidbody::Mesh::point(unsigned int i) const
{
    return *(m_mesh.begin()+i);
}
unsigned int biorbd::rigidbody::Mesh::size() const
{
    return static_cast<unsigned int>(m_mesh.size());
}

void biorbd::rigidbody::Mesh::setPath(const biorbd::utils::Path& path)
{
    m_pathFile = path;
}

const biorbd::utils::Path &biorbd::rigidbody::Mesh::path() const
{
    return m_pathFile;
}

unsigned int biorbd::rigidbody::Mesh::sizePatch()
{
    return static_cast<unsigned int>(m_patch.size());
}
void biorbd::rigidbody::Mesh::addPatch(const biorbd::rigidbody::Patch & v)
{
    m_patch.push_back(v);
}
void biorbd::rigidbody::Mesh::addPatch(const Eigen::Vector3i & v)
{
    addPatch(biorbd::rigidbody::Patch(v));
}
const std::vector<biorbd::rigidbody::Patch>& biorbd::rigidbody::Mesh::patch() const
{
    return m_patch;
}
const biorbd::rigidbody::Patch &biorbd::rigidbody::Mesh::patch(unsigned int i) const
{
    return m_patch[i];
}
