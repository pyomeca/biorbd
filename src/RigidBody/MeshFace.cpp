#define BIORBD_API_EXPORTS
#include "RigidBody/MeshFace.h"

#include "Utils/Vector3d.h"

biorbd::rigidbody::MeshFace::MeshFace(const Eigen::Vector3i& vertex) :
    m_face(std::make_shared<Eigen::Vector3i>(vertex))
{

}

biorbd::rigidbody::MeshFace biorbd::rigidbody::MeshFace::DeepCopy() const
{
    biorbd::rigidbody::MeshFace copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::MeshFace::DeepCopy(const biorbd::rigidbody::MeshFace &other)
{
    *m_face = *other.m_face;
}

int &biorbd::rigidbody::MeshFace::operator()(int idx)
{
    return (*m_face)[idx];
}

biorbd::utils::Vector3d biorbd::rigidbody::MeshFace::faceAsDouble()
{
    return biorbd::utils::Vector3d(static_cast<double>(m_face->x()),
        static_cast<double>(m_face->y()),
        static_cast<double>(m_face->z()));
}

void biorbd::rigidbody::MeshFace::setFace(const Eigen::Vector3i & pts)
{
    *m_face = pts;
}

void biorbd::rigidbody::MeshFace::setFace(const biorbd::rigidbody::MeshFace &other)
{
    m_face = other.m_face;
}

