#ifndef BIORBD_RIGIDBODY_SOFT_CONTACTS_H
#define BIORBD_RIGIDBODY_SOFT_CONTACTS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace BIORBD_NAMESPACE
{

namespace utils {
class NodeSegment;

}

namespace rigidbody
{

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
    /// \brief Add a new contact to the contact set
    /// \param contact The contact to add
    ///
    void addContact(
        const ContactNode& contact);

    ///
    /// \brief Return a specified contact
    /// \param idx The index of the marker
    /// \return The contact of index idx
    ///
    NodeSegment SoftContacts(
        unsigned int  idx);

    ///
    /// \brief Return all the contacts at a given position Q
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \return All the contacts in the global reference frame
    ///
    std::vector<NodeSegment> SoftContacts(
        const GeneralizedCoordinates &Q,
        bool removeAxis = true,
        bool updateKin = true);

    ///
    /// \brief Return all the contacts in their respective parent reference frame
    /// \return All the contacts
    ///
    std::vector<NodeSegment> SoftContacts();

    ///
    /// \brief Return the velocity of a contact
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the contact set
    /// \param updateKin If the model should be updated
    /// \return The velocity of the contact
    ///
    NodeSegment contactVelocity(
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
    std::vector<NodeSegment> contactsVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin = true);

    ///
    /// \brief Return the number of contacts
    /// \return The number of contacts
    ///
    unsigned int nbSoftContacts() const;

protected:
    std::shared_ptr<std::vector<NodeSegment>>
            m_contacts; ///< The contacts

};

}
}

#endif // BIORBD_RIGIDBODY_SOFT_CONTACTS_H
