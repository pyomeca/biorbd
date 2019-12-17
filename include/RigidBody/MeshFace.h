#ifndef BIORBD_RIGIDBODY_MESH_FACE_H
#define BIORBD_RIGIDBODY_MESH_FACE_H

#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Vector3d;
}

namespace rigidbody {

///
/// \brief The face of the mesh
///
class BIORBD_API MeshFace
{
public:
    ///
    /// \brief Contruct face MeshFace
    /// \param vertex The vertex to connect to form a face
    ///
    MeshFace(
            const Eigen::Vector3i& vertex= Eigen::Vector3i());

    ///
    /// \brief Deep copy of a MeshFace
    /// \return A deep copy of a MeshFace
    ///
    biorbd::rigidbody::MeshFace DeepCopy() const;

    ///
    /// \brief Deep copy of a MeshFace into another one
    /// \param other The MeshFace to copy
    ///
    void DeepCopy(
            const biorbd::rigidbody::MeshFace& other);

    ///
    /// \brief Allows to assign/get using ()
    /// \param idx The index in the vector
    ///
    int &operator() (int idx);

    ///
    /// \brief set the MeshFace from a new point
    /// \param pts The new point to copy
    ///
    void setFace(const Eigen::Vector3i& pts);

    ///
    /// \brief Copy the face from another MeshFace
    /// \param other The other MeshFace
    ///
    void setFace(
            const biorbd::rigidbody::MeshFace& other);

    ///
    /// \brief convert the integer nature of the face to a double
    /// \return The vertex index in double format
    ///
    biorbd::utils::Vector3d faceAsDouble();

protected:
    std::shared_ptr<Eigen::Vector3i> m_face; ///< The face

};

}}

#endif // BIORBD_RIGIDBODY_MESH_FACE_H
