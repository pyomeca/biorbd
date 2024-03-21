#ifndef BIORBD_RIGIDBODY_SOFT_CONTACT_NODE_H
#define BIORBD_RIGIDBODY_SOFT_CONTACT_NODE_H

#include "biorbdConfig.h"
#include "RigidBody/NodeSegment.h"
#include "rbdl/rbdl_math.h"

namespace BIORBD_NAMESPACE
{

namespace utils
{
class SpatialVector;
}

namespace rigidbody
{
class Joints;
class GeneralizedCoordinates;
class GeneralizedVelocity;

///
/// \brief A point attached to a contact node, generally speaking a skin marker
///
class BIORBD_API SoftContactNode : public rigidbody::NodeSegment
{
public:
    ///
    /// \brief Construct a contact node
    ///
    SoftContactNode();

    ///
    /// \brief Construct a contact node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    ///
    SoftContactNode(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z);

    ///
    /// \brief Construct a contact node from another node
    /// \param other The other node
    ///
    SoftContactNode(
        const utils::Vector3d& other);

    ///
    /// \brief Construct a contact node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param parentID The index of the parent contact
    ///
    SoftContactNode(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z,
        const utils::String& name,
        const utils::String& parentName,
        int parentID);

    ///
    /// \brief Construct a contact node
    /// \param node The position of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param parentID The index of the parent contact
    ///
    SoftContactNode(
        const utils::Vector3d& node,
        const utils::String& name,
        const utils::String& parentName,
        int parentID);

    ///
    /// \brief Deep copy of the contact node
    /// \param other The contact node to copy
    ///
    void DeepCopy(const SoftContactNode& other);

    ///
    /// \brief Get the force in a spatial vector at the origin of the world base coordinates
    /// \param model The model
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized velocities
    /// \param updateKin If the kinematics should be updated
    /// \return The Spatial vector
    ///
    virtual utils::SpatialVector computeForceAtOrigin(
            Joints& model,
            const GeneralizedCoordinates& Q,
            const GeneralizedVelocity& Qdot,
            bool updateKin = true);

    ///
    /// \brief Get the force in a spatial vector at the center of mass of the underlying segment
    /// \param x The position of the contact in global reference frame
    /// \param dx The velocity of the contact in global reference frame
    /// \return The Spatial vector
    ///
    virtual utils::Vector3d computeForce(
            const utils::Vector3d& x,
            const utils::Vector3d& dx,
            const utils::Vector3d& angularVelocity) const = 0;

    ///
    /// \brief Get the application point relative to the plane
    /// \param x The position of the contact in global reference frame
    /// \return The application point
    ///
    virtual utils::Vector3d applicationPoint(
            const utils::Vector3d& x) const = 0;

protected:
    ///
    /// \brief Set the type of the contact node
    ///
    void setType();

    std::shared_ptr<std::pair<utils::Vector3d, utils::Vector3d>> m_contactPlane; ///< The contact plane that interface with the node in global reference frame

};

}
}

#endif // BIORBD_RIGIDBODY_SOFT_CONTACT_NODE_H
