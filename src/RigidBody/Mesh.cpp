#define BIORBD_API_EXPORTS
#include "RigidBody/Mesh.h"

#include "Utils/Path.h"
#include "Utils/Vector3d.h"
#include "RigidBody/Patch.h"

biorbd::rigidbody::Mesh::Mesh() :
    m_vertex(std::make_shared<std::vector<biorbd::utils::Vector3d>>()),
    m_patch(std::make_shared<std::vector<biorbd::rigidbody::Patch>>()),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::Mesh::Mesh(
        const std::vector<biorbd::utils::Vector3d> &mesh) :
    m_vertex(std::make_shared<std::vector<biorbd::utils::Vector3d>>(mesh)),
    m_patch(std::make_shared<std::vector<biorbd::rigidbody::Patch>>()),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::Mesh::Mesh(
        const std::vector<biorbd::utils::Vector3d> &mesh,
        const std::vector<biorbd::rigidbody::Patch> & v) :
    m_vertex(std::make_shared<std::vector<biorbd::utils::Vector3d>>(mesh)),
    m_patch(std::make_shared<std::vector<biorbd::rigidbody::Patch>>(v)),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::Mesh biorbd::rigidbody::Mesh::DeepCopy() const
{
    biorbd::rigidbody::Mesh copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Mesh::DeepCopy(const biorbd::rigidbody::Mesh &other)
{
    m_vertex->resize(other.m_vertex->size());
    for (unsigned int i=0; i<other.m_vertex->size(); ++i)
        (*m_vertex)[i] = (*other.m_vertex)[i].DeepCopy();
    m_patch->resize(other.m_patch->size());
    for (unsigned int i=0; i<other.m_patch->size(); ++i)
        (*m_patch)[i] = (*other.m_patch)[i].DeepCopy();
    *m_pathFile = other.m_pathFile->DeepCopy();
}

void biorbd::rigidbody::Mesh::addPoint(const biorbd::utils::Vector3d &node)
{
    m_vertex->push_back(node);
}
const biorbd::utils::Vector3d &biorbd::rigidbody::Mesh::point(unsigned int i) const
{
    return (*m_vertex)[i];
}
unsigned int biorbd::rigidbody::Mesh::size() const
{
    return static_cast<unsigned int>(m_vertex->size());
}

unsigned int biorbd::rigidbody::Mesh::nbPatch()
{
    return static_cast<unsigned int>(m_patch->size());
}
void biorbd::rigidbody::Mesh::addPatch(const biorbd::rigidbody::Patch & v)
{
    m_patch->push_back(v);
}
void biorbd::rigidbody::Mesh::addPatch(const Eigen::Vector3i & v)
{
    addPatch(biorbd::rigidbody::Patch(v));
}
const std::vector<biorbd::rigidbody::Patch>& biorbd::rigidbody::Mesh::patch() const
{
    return *m_patch;
}
const biorbd::rigidbody::Patch &biorbd::rigidbody::Mesh::patch(unsigned int i) const
{
    return (*m_patch)[i];
}

void biorbd::rigidbody::Mesh::setPath(const biorbd::utils::Path& path)
{
    *m_pathFile = path;
}

const biorbd::utils::Path &biorbd::rigidbody::Mesh::path() const
{
    return *m_pathFile;
}
