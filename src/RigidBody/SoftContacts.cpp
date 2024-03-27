#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContacts.h"

#include "Utils/String.h"
#include "RigidBody/SoftContactNode.h"
#include "RigidBody/SoftContactSphere.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/Segment.h"

#include "Utils/UtilsEnum.h"
#include "Utils/String.h"
#include "Utils/Error.h"
#include "Utils/SpatialVector.h"

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
    for (size_t i=0; i<other.m_softContacts->size(); ++i) {
        if ((*other.m_softContacts)[i]->typeOfNode() == utils::NODE_TYPE::SOFT_CONTACT_SPHERE){
            (*m_softContacts)[i] = std::make_shared<rigidbody::SoftContactSphere>();
        } else {
            utils::Error::raise("DeepCopy failed");
        }
        (*m_softContacts)[i]->DeepCopy(*((*other.m_softContacts)[i]));
    }
}

utils::String rigidbody::SoftContacts::softContactName(
        size_t i)
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
        size_t idx)
{
    return *(*m_softContacts)[idx];
}

rigidbody::NodeSegment rigidbody::SoftContacts::softContact(
        const rigidbody::GeneralizedCoordinates &Q,
        size_t idx,
        bool updateKin)
{
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    const rigidbody::SoftContactNode& sc(softContact(idx));
    return rigidbody::NodeSegment(model.CalcBodyToBaseCoordinates(Q, sc.parent(), sc, updateKin));
}

std::vector<rigidbody::NodeSegment> rigidbody::SoftContacts::softContacts(
        const GeneralizedCoordinates &Q,
        bool updateKin)
{
    std::vector<rigidbody::NodeSegment> out;
    for (int i = 0; i < static_cast<int>(m_softContacts->size()); ++i){
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
            size_t idx,
            bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Calculate the velocity of the point
    const rigidbody::SoftContactNode& sc(softContact(idx));
    return rigidbody::NodeSegment(
        model.CalcPointVelocity(Q, Qdot, sc.parent(), sc, updateKin));
}

rigidbody::NodeSegment rigidbody::SoftContacts::softContactAngularVelocity(
            const rigidbody::GeneralizedCoordinates &Q,
            const rigidbody::GeneralizedVelocity &Qdot,
            size_t idx,
            bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Calculate the velocity of the point
    const rigidbody::SoftContactNode& sc(softContact(idx));
    return rigidbody::NodeSegment(
        model.CalcPointVelocity6D(Q, Qdot, sc.parent(), sc, updateKin).block(0, 0, 3, 1));
}

std::vector<rigidbody::NodeSegment> rigidbody::SoftContacts::softContactsVelocity(
            const rigidbody::GeneralizedCoordinates &Q,
            const rigidbody::GeneralizedVelocity &Qdot,
            bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbSoftContacts(); ++i) {
        pos.push_back(softContactVelocity(Q, Qdot, i, updateKin));
        updateKin = false;
    }

    return pos;
}

std::vector<rigidbody::NodeSegment> rigidbody::SoftContacts::softContactsAngularVelocity(
            const rigidbody::GeneralizedCoordinates &Q,
            const rigidbody::GeneralizedVelocity &Qdot,
            bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbSoftContacts(); ++i) {
        pos.push_back(softContactAngularVelocity(Q, Qdot, i, updateKin));
        updateKin = false;
    }

    return pos;
}

size_t rigidbody::SoftContacts::nbSoftContacts() const
{
    return m_softContacts->size();
}

std::vector<size_t> rigidbody::SoftContacts::segmentSoftContactIdx(
        size_t idx) const
{
    // Assuming that this is also a joint type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<const rigidbody::Joints &>(*this);

    // Name of the segment to find
    const utils::String& name(model.segment(idx).name());

    std::vector<size_t> indices;
    for (size_t i=0; i<nbSoftContacts(); ++i) // Go through all the markers and select the right ones
        if (!(*m_softContacts)[i]->parent().compare(name)) {
            indices.push_back(i);
        }

    return indices;
}

