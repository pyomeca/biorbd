#ifndef BIORBD_RIGIDBODY_MESH_H
#define BIORBD_RIGIDBODY_MESH_H

#include "biorbdConfig.h"
#include <memory>
#include <vector>

namespace BIORBD_NAMESPACE
{
namespace utils
{
class Vector3d;
class RotoTrans;
class Path;
}

namespace rigidbody
{
class MeshFace;

///
/// \brief A class that holds the geometry of a segment
///
class BIORBD_API Mesh
{
public:
    ///
    /// \brief Construct mesh
    ///
    Mesh();

    ///
    /// \brief Construct mesh from another mesh
    /// \brief other The other mesh
    ///
    Mesh(
        const std::vector<utils::Vector3d>& other);

    ///
    /// \brief Construct mesh
    /// \param vertex The vertex of the geometry
    /// \param faces The faces of the geometry
    ///
    Mesh(
        const std::vector<utils::Vector3d>& vertex,
        const std::vector<MeshFace>& faces);

    ///
    /// \brief Deep copy of the mesh
    /// \return A copy of mesh
    ///
    Mesh DeepCopy() const;

    ///
    /// \brief Deep copy of the mesh
    /// \param other The mesh to copy
    ///
    void DeepCopy(
        const Mesh& other);

    ///
    /// \brief Set the patch color
    /// \param color The color
    ///
    void setColor(
            const utils::Vector3d& color);

    ///
    /// \brief Get the patch color
    /// \return The patch color
    ///
    utils::Vector3d& color() const;

    ///
    /// \brief Return if there is a mesh
    /// \return If there is a mesh
    bool hasMesh() const;

    ///
    /// \brief Add a point to the mesh
    /// \param node The point to add
    ///
    void addPoint(
        const utils::Vector3d& node);

    ///
    /// \brief Return the point of a specific index
    /// \param idx The index of the point
    /// \return The point of a specific index
    ///
    const utils::Vector3d& point(
        size_t idx) const;

    ///
    /// \brief Returns the number of vertex
    /// \return The number of vertex
    ///
    size_t nbVertex() const;

    ///
    /// \brief rotateVertex Apply the RT to the vertex
    /// \param rt The Transformation to apply to the mesh wrt the parent
    ///
    void rotate(
            const utils::RotoTrans& rt);

    ///
    /// \brief Get the rotation applied to the vertex
    /// \return The Transformation applied to the mesh wrt the parent
    ///
    utils::RotoTrans& getRotation() const;

    ///
    /// \brief Scale the vertex wrt to 0, 0, 0. Warnign, this function is
    /// applied when called, meaning that if it is called after rotate, it will
    /// results in surprising results!
    /// \param scaler The x, y, z values to scale
    ///
    void scale(
            const utils::Vector3d& scaler);

    ///
    /// \brief Get the scaling applied to the vertex wrt to 0, 0, 0.
    /// \return The scaling applied to the mesh wrt the parent
    ///
    utils::Vector3d& getScale() const;

    ///
    /// \brief Add a face patch to the mesh
    /// \param face The face patch to add
    ///
    void addFace(
        const MeshFace& face);

    ///
    /// \brief Add a face patch to the mesh
    /// \param face The face patch to add
    ///
    void addFace(const std::vector<int> &face);

    ///
    /// \brief Return the faces of the mesh
    /// \return The faces of the mesh
    ///
    const std::vector<MeshFace>& faces() const;

    ///
    /// \brief Return the face of the mesh of a specified idx
    /// \param idx Position
    /// \return The face of the mesh of a specified idx
    ///
    const MeshFace& face(
        size_t idx) const;

    ///
    /// \brief Return the number of faces
    /// \return The number of faces
    ///
    size_t nbFaces();

    ///
    /// \brief Set the path of the underlying mesh file
    /// \param path Path for the mesh file
    ///
    void setPath(
        const utils::Path& path);

    ///
    /// \brief Return the path of the mesh file
    /// \return The path of the mesh file
    ///
    const utils::Path& path() const;

protected:
    std::shared_ptr<utils::RotoTrans> m_rotation; ///< The rotation
    std::shared_ptr<std::vector<utils::Vector3d>> m_vertex; ///< The vertex
    std::shared_ptr<std::vector<MeshFace>>
            m_faces; ///< The faces
    std::shared_ptr<utils::Path> m_pathFile; ///< The path to the mesh file
    std::shared_ptr<utils::Vector3d> m_patchColor; ///< The color of faces
    std::shared_ptr<utils::Vector3d> m_scale; ///< The scale
};

}
}

#endif // BIORBD_RIGIDBODY_MESH_H
