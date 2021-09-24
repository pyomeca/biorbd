#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContacts.h"

using namespace BIORBD_NAMESPACE;

rigidbody::SoftContacts::SoftContacts()
{

}

rigidbody::SoftContacts rigidbody::SoftContacts::DeepCopy() const
{

}

void rigidbody::SoftContacts::DeepCopy(
        const rigidbody::SoftContacts &other)
{

}

void rigidbody::SoftContacts::addContact(
        const ContactNode &contact)
{

}

utils::NodeSegment rigidbody::SoftContacts::SoftContacts(
        unsigned int idx)
{

}

std::vector<utils::NodeSegment> rigidbody::SoftContacts::SoftContacts(
        const rigidbody::GeneralizedCoordinates &Q, bool removeAxis, bool updateKin)
{

}

std::vector<utils::NodeSegment> rigidbody::SoftContacts::SoftContacts()
{

}

utils::NodeSegment rigidbody::SoftContacts::contactVelocity(
            const rigidbody::GeneralizedCoordinates &Q,
            const rigidbody::GeneralizedVelocity &Qdot,
            unsigned int idx,
            bool updateKin)
{

}

std::vector<utils::NodeSegment> rigidbody::SoftContacts::contactsVelocity(
            const rigidbody::GeneralizedCoordinates &Q,
            const rigidbody::GeneralizedVelocity &Qdot,
            bool updateKin)
{

}

unsigned int rigidbody::SoftContacts::nbSoftContacts() const
{
    return m_contacts;
}
