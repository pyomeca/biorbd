#define BIORBD_API_EXPORTS
#include "RigidBody/Mesh.h"

#include "Utils/Path.h"
#include "Utils/Vector3d.h"
#include "RigidBody/MeshFace.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

rigidbody::Mesh::Mesh() :
    m_vertex(std::make_shared<std::vector<utils::Vector3d>>()),
    m_faces(std::make_shared<std::vector<rigidbody::MeshFace>>()),
    m_pathFile(std::make_shared<utils::Path>())
{

}

rigidbody::Mesh::Mesh(const std::vector<utils::Vector3d> &other)
    :
    m_vertex(std::make_shared<std::vector<utils::Vector3d>>(other)),
    m_faces(std::make_shared<std::vector<rigidbody::MeshFace>>()),
    m_pathFile(std::make_shared<utils::Path>())
{

}

rigidbody::Mesh::Mesh(const std::vector<utils::Vector3d>
                              &vertex,
                              const std::vector<rigidbody::MeshFace> & faces) :
    m_vertex(std::make_shared<std::vector<utils::Vector3d>>(vertex)),
    m_faces(std::make_shared<std::vector<rigidbody::MeshFace>>(faces)),
    m_pathFile(std::make_shared<utils::Path>())
{

}

rigidbody::Mesh rigidbody::Mesh::DeepCopy() const
{
    rigidbody::Mesh copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::Mesh::DeepCopy(const rigidbody::Mesh &other)
{
    m_vertex->resize(other.m_vertex->size());
    for (unsigned int i=0; i<other.m_vertex->size(); ++i) {
        (*m_vertex)[i] = (*other.m_vertex)[i].DeepCopy();
    }
    m_faces->resize(other.m_faces->size());
    for (unsigned int i=0; i<other.m_faces->size(); ++i) {
        (*m_faces)[i] = (*other.m_faces)[i].DeepCopy();
    }
    *m_pathFile = other.m_pathFile->DeepCopy();
}

void rigidbody::Mesh::addPoint(const utils::Vector3d &node)
{
    m_vertex->push_back(node);
}
const utils::Vector3d &rigidbody::Mesh::point(
    unsigned int idx) const
{
    return (*m_vertex)[idx];
}
unsigned int rigidbody::Mesh::nbVertex() const
{
    return static_cast<unsigned int>(m_vertex->size());
}

unsigned int rigidbody::Mesh::nbFaces()
{
    return static_cast<unsigned int>(m_faces->size());
}
void rigidbody::Mesh::addFace(const rigidbody::MeshFace& face)
{
    m_faces->push_back(face);
}
void rigidbody::Mesh::addFace(const std::vector<int> & face)
{
    addFace(rigidbody::MeshFace(face));
}
const std::vector<rigidbody::MeshFace>& rigidbody::Mesh::faces()
const
{
    return *m_faces;
}
const rigidbody::MeshFace &rigidbody::Mesh::face(
    unsigned int idx) const
{
    return (*m_faces)[idx];
}

void rigidbody::Mesh::setPath(const utils::Path& path)
{
    *m_pathFile = path;
}

const utils::Path &rigidbody::Mesh::path() const
{
    return *m_pathFile;
}
