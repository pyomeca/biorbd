#ifndef BIORBD_RIGIDBODY_SOFT_CONTACTS_H
#define BIORBD_RIGIDBODY_SOFT_CONTACTS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"

namespace BIORBD_NAMESPACE
{
namespace utils {
class String;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class SoftContactNode;
class NodeSegment;

///
/// \brief Holder for the biorbd contact set
///
class BIORBD_API SoftContacts
{
public:
    ///
    /// \brief Construct a contact biorbd set
    ///
    SoftContacts();

    ///
    /// \brief Virtual destructor
    ///
    virtual ~SoftContacts(){}

    ///
    /// \brief Deep copy of the contacts
    /// \return Deep copy of the contacts
    ///
    SoftContacts DeepCopy() const;

    ///
    /// \brief Deep copy of the contacts
    /// \param other The contacts to copy from
    ///
    void DeepCopy(const SoftContacts& other);

    ///
    /// \brief Get the soft contacts in a list of spatial vector of dimension 6xNdof
    /// \param
    /// \return The soft contacts
    ///
    std::vector<RigidBodyDynamics::Math::SpatialVector>* softContactToSpatialVector(
            const GeneralizedCoordinates& Q,
            const GeneralizedVelocity& QDot) const;

    ///
    /// \brief Combine the current forces to an external force vector
    /// \param externalForces The external forces to combine to
    ///
    void combineToExternalForce(
            std::vector<RigidBodyDynamics::Math::SpatialVector>* externalForces = nullptr) const;

    ///
    /// \brief Return the name of the soft contact
    /// \param i The index of the contact
    /// \return The name of the soft contact
    ///
    utils::String softContactName(unsigned int i);

    ///
    /// \brief Return the names of the soft contact
    /// \param i The index of the contact
    /// \return The names of the soft contact
    ///
    std::vector<utils::String> softContactNames();

    ///
    /// \brief Add a new contact to the contact set
    /// \param contact The contact to add
    ///
    void addSoftContact(
        const SoftContactNode& contact);

    ///
    /// \brief Return a specified contact
    /// \param idx The index of the marker
    /// \return The contact of index idx
    ///
    SoftContactNode& softContact(
        unsigned int  idx);

    ///
    /// \brief Return a specified contact at a given position Q
    /// \param Q The generalized coordinates
    /// \param idx The index of the marker
    /// \param updateKin If the model should be updated
    /// \return The contact of index idx
    ///
    NodeSegment softContact(
        const GeneralizedCoordinates &Q,
        unsigned int  idx,
        bool updateKin = true);

    ///
    /// \brief Return a the contacts at a given position Q
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \return The contacts
    ///
    std::vector<NodeSegment> softContacts(
        const GeneralizedCoordinates &Q,
        bool updateKin = true);

    ///
    /// \brief Return the velocity of a contact
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the contact set
    /// \param updateKin If the model should be updated
    /// \return The velocity of the contact
    ///
    NodeSegment softContactVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        unsigned int idx,
        bool updateKin = true);


    ///
    /// \brief Return the velocity of all the contacts
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the model should be updated
    /// \return The velocity of all the contacts
    ///
    std::vector<NodeSegment> softContactsVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin = true);

    ///
    /// \brief Return the number of contacts
    /// \return The number of contacts
    ///
    unsigned int nbSoftContacts() const;

    ///
    /// \brief Return all the soft contacts indices of a segment
    /// \param idx The index of the segment
    /// \return All the soft contacts of a segment
    ///
    std::vector<size_t> segmentSoftContactIdx(
            unsigned int  idx) const;

protected:
    std::shared_ptr<std::vector<std::shared_ptr<SoftContactNode>>> m_softContacts; ///< The contacts

};

}
}

#endif // BIORBD_RIGIDBODY_SOFT_CONTACTS_H
