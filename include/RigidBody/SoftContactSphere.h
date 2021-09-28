#ifndef BIORBD_RIGIDBODY_SOFT_CONTACT_SPHERE_H
#define BIORBD_RIGIDBODY_SOFT_CONTACT_SPHERE_H

#include "biorbdConfig.h"
#include "RigidBody/SoftContactNode.h"

namespace BIORBD_NAMESPACE
{

namespace rigidbody
{

///
/// \brief A point attached to a contact node, generally speaking a skin marker
///
class BIORBD_API SoftContactSphere : public rigidbody::SoftContactNode
{
public:
    ///
    /// \brief Construct a contact node
    ///
    SoftContactSphere();

    ///
    /// \brief Construct a contact node
    ///
    SoftContactSphere(const rigidbody::SoftContactNode&);

    ///
    /// \brief Construct a contact node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    /// \param radius The radius of the sphere
    /// \param stiffness The stiffness of the contact sphere
    /// \param damping The damping factor of the contact sphere
    ///
    SoftContactSphere(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z,
        const utils::Scalar& radius,
        const utils::Scalar& stiffness,
        const utils::Scalar& damping);

    ///
    /// \brief Construct a contact node from another node
    /// \param other The other node
    /// \param radius The radius of the sphere
    /// \param stiffness The stiffness of the contact sphere
    /// \param damping The damping factor of the contact sphere
    ///
    SoftContactSphere(
        const utils::Vector3d& other,
        const utils::Scalar& radius,
        const utils::Scalar& stiffness,
        const utils::Scalar& damping);

    ///
    /// \brief Construct a contact node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    /// \param radius The radius of the sphere
    /// \param stiffness The stiffness of the contact sphere
    /// \param damping The damping factor of the contact sphere
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param parentID The index of the parent contact
    ///
    SoftContactSphere(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z,
        const utils::Scalar& radius,
        const utils::Scalar& stiffness,
        const utils::Scalar& damping,
        const utils::String& name,
        const utils::String& parentName,
        int parentID);

    ///
    /// \brief Construct a contact node
    /// \param node The position of the node
    /// \param radius The radius of the sphere
    /// \param stiffness The stiffness of the contact sphere
    /// \param damping The damping factor of the contact sphere
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param parentID The index of the parent contact
    ///
    SoftContactSphere(
        const utils::Vector3d& node,
        const utils::Scalar& radius,
        const utils::Scalar& stiffness,
        const utils::Scalar& damping,
        const utils::String& name,
        const utils::String& parentName,
        int parentID);

    ///
    /// \brief Deep copy of the contact node
    /// \return A deep copy of the contact node
    ///
    SoftContactSphere DeepCopy() const;

    ///
    /// \brief Deep copy of the contact node
    /// \param other The contact node to copy
    ///
    void DeepCopy(const SoftContactSphere& other);

    ///
    /// \brief Set a new value for the radius
    /// \param radius The new value for the radius
    ///
    void setRadius(const utils::Scalar& radius);

    ///
    /// \brief Return the value of the radius
    /// \return The value of the radius
    ///
    utils::Scalar radius() const;

    ///
    /// \brief Set a new value for the stiffness
    /// \param stiffness The new value for the stiffness
    ///
    void setStiffness(const utils::Scalar& stiffness);

    ///
    /// \brief Return the value of the stiffness
    /// \return The value of the stiffness
    ///
    utils::Scalar stiffness() const;

    ///
    /// \brief Set a new value for the damping
    /// \param damping The new value for the damping
    ///
    void setDamping(const utils::Scalar& damping);

    ///
    /// \brief Return the value of the damping
    /// \return The value of the damping
    ///
    utils::Scalar damping() const;

    ///
    /// \brief Get the force in a spatial vector at the center of mass of the underlying segment
    /// \param x The position of the contact in global reference frame
    /// \param dx The velocity of the contact in global reference frame
    /// \return The Spatial vector
    ///
    virtual utils::Vector3d computeForce(
            const utils::Vector3d& x,
            const utils::Vector3d& dx) const;

    ///
    /// \brief Get the application point relative to the plane
    /// \param x The position of the contact in global reference frame
    /// \return The application point
    ///
    virtual utils::Vector3d applicationPoint(
            const utils::Vector3d& x) const;

protected:
    ///
    /// \brief Set the type of the contact node
    ///
    void setType();

    std::shared_ptr<utils::Scalar> m_radius; ///< The radius of the contact sphere
    std::shared_ptr<utils::Scalar> m_stiffness; ///< The stiffness of the contact sphere
    std::shared_ptr<utils::Scalar> m_damping; ///< The damping factor of the contact sphere

    std::shared_ptr<utils::Scalar> m_muStatic; ///< Static coefficient of friction (mu)
    std::shared_ptr<utils::Scalar> m_muDynamic; ///< Dynamic coefficient of friction (mu)
    std::shared_ptr<utils::Scalar> m_muViscous; ///< Vicous coefficient of friction (mu)
    std::shared_ptr<utils::Scalar> m_transitionVelocity; ///< Transition velocity factor of Hunt-Crossley's model
};

}
}

#endif // BIORBD_RIGIDBODY_SOFT_CONTACT_SPHERE_H
