#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContacts.h"

#include "Utils/String.h"
#include "RigidBody/SoftContactNode.h"
#include "RigidBody/SoftContactSphere.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/Segment.h"

#include "Utils/String.h"
#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

rigidbody::SoftContacts::SoftContacts():
    m_softContacts(std::make_shared<std::vector<std::shared_ptr<SoftContactNode>>>())
{

}

rigidbody::SoftContacts rigidbody::SoftContacts::DeepCopy() const
{
    rigidbody::SoftContacts copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::SoftContacts::DeepCopy(
        const rigidbody::SoftContacts &other)
{
    m_softContacts->resize(other.m_softContacts->size());
    for (unsigned int i=0; i<other.m_softContacts->size(); ++i) {
        (*m_softContacts)[i] = std::make_shared<rigidbody::SoftContactNode>();
        *(*m_softContacts)[i] = (*other.m_softContacts)[i]->DeepCopy();
    }
}

utils::String rigidbody::SoftContacts::softContactName(
        unsigned int i)
{
    return (*m_softContacts)[i]->utils::Node::name();
}

std::vector<utils::String> rigidbody::SoftContacts::softContactNames()
{
    std::vector<utils::String> out;
    for (auto& c : *m_softContacts){
        out.push_back(c->utils::Node::name());
    }
    return out;
}

void rigidbody::SoftContacts::addSoftContact(
        const rigidbody::SoftContactNode &contact)
{
    if (contact.typeOfNode() == utils::NODE_TYPE::SOFT_CONTACT_SPHERE) {
        m_softContacts->push_back(std::make_shared<rigidbody::SoftContactSphere>(contact));
    } else {
        utils::Error::raise(utils::String("The ") + contact.typeOfNode() + " does not exist");
    }
}

rigidbody::SoftContactNode& rigidbody::SoftContacts::softContact(
        unsigned int idx)
{
    return *(*m_softContacts)[idx];
}

rigidbody::NodeSegment rigidbody::SoftContacts::softContact(
        const rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx,
        bool updateKin)
{
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::SoftContactNode& sc(softContact(idx));
    unsigned int id = model.GetBodyId(sc.parent().c_str());

    return rigidbody::NodeSegment(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, sc, updateKin));
}

std::vector<rigidbody::NodeSegment> rigidbody::SoftContacts::softContacts(
        const GeneralizedCoordinates &Q,
        bool updateKin)
{
    std::vector<rigidbody::NodeSegment> out;
    for (size_t i = 0; i<m_softContacts->size(); ++i){
        out.push_back(softContact(Q, i, updateKin));
#ifndef BIORBD_USE_CASADI_MATH
        updateKin = false;
#endif
    }
    return out;
}

rigidbody::NodeSegment rigidbody::SoftContacts::softContactVelocity(
            const rigidbody::GeneralizedCoordinates &Q,
            const rigidbody::GeneralizedVelocity &Qdot,
            unsigned int idx,
            bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::SoftContactNode& sc(softContact(idx));
    unsigned int id(model.GetBodyId(sc.parent().c_str()));

    // Calculate the velocity of the point
    return rigidbody::NodeSegment(RigidBodyDynamics::CalcPointVelocity(
            model, Q, Qdot, id, sc, updateKin));
}

std::vector<rigidbody::NodeSegment> rigidbody::SoftContacts::softContactsVelocity(
            const rigidbody::GeneralizedCoordinates &Q,
            const rigidbody::GeneralizedVelocity &Qdot,
            bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbSoftContacts(); ++i) {
        pos.push_back(softContactVelocity(Q, Qdot, i, updateKin));
        updateKin = false;
    }

    return pos;
}

unsigned int rigidbody::SoftContacts::nbSoftContacts() const
{
    return m_softContacts->size();
}

std::vector<rigidbody::NodeSegment> rigidbody::SoftContacts::segmentSoftContact(
        const GeneralizedCoordinates &Q,
        unsigned int idx,
        bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Name of the segment to find
    const utils::String& name(model.segment(idx).name());

    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbSoftContacts(); ++i) // Go through all the markers and select the right ones
        if (!(*m_softContacts)[i]->parent().compare(name)) {
            pos.push_back(softContact(Q,i,updateKin));
            updateKin = false;
        }

    return pos;
}

