#define BIORBD_API_EXPORTS
#include "RigidBody/Contacts.h"

#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Error.h"
#include "Utils/ExternalForceSet.h"
#include "Utils/Vector3d.h"
#include "Utils/RotoTrans.h"
#include "Utils/Rotation.h"
#include "Utils/SpatialVector.h"
#include "RigidBody/Joints.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"

using namespace BIORBD_NAMESPACE;

rigidbody::Contacts::Contacts() :
    RigidBodyDynamics::ConstraintSet (),
    m_nbreConstraint(std::make_shared<unsigned int>(0)),
    m_isBinded(std::make_shared<bool>(false)),
    m_rigidContacts(std::make_shared<std::vector<rigidbody::NodeSegment>>()),
    m_nbLoopConstraint(std::make_shared<unsigned int>(0))
{

}

rigidbody::Contacts rigidbody::Contacts::DeepCopy() const
{
    rigidbody::Contacts copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::Contacts::DeepCopy(const rigidbody::Contacts
        &other)
{
    static_cast<RigidBodyDynamics::ConstraintSet&>(*this) = other;
    *m_nbLoopConstraint = *other.m_nbLoopConstraint;
    *m_nbreConstraint = *other.m_nbreConstraint;
    *m_isBinded = *other.m_isBinded;
    *m_rigidContacts = *other.m_rigidContacts;
}

unsigned int rigidbody::Contacts::AddConstraint(
    unsigned int body_id,
    const utils::Vector3d& body_point,
    const utils::Vector3d& world_normal,
    const utils::String& name)
{
    ++*m_nbreConstraint;

    // Check world_normal points to what axis
    utils::String axis = "";
    SCALAR_TO_DOUBLE(has_x, world_normal[0]);
    SCALAR_TO_DOUBLE(has_y, world_normal[1]);
    SCALAR_TO_DOUBLE(has_z, world_normal[2]);
    if (has_x != 0){
        axis += "x";
    }
    if (has_y != 0){
        axis += "y";
    }
    if (has_z != 0){
        axis += "z";
    }

    m_rigidContacts->push_back(NodeSegment(body_point, name, "",true,false, axis.c_str(),body_id));
    return RigidBodyDynamics::ConstraintSet::AddContactConstraint(body_id,
            body_point, world_normal, name.c_str());
}
unsigned int rigidbody::Contacts::AddConstraint(
    unsigned int body_id,
    const utils::Vector3d& body_point,
    const utils::String& axis,
    const utils::String& name)
{
    unsigned int ret(0);
    for (unsigned int i=0; i<axis.length(); ++i) {
        ++*m_nbreConstraint;
        if      (axis.tolower()[i] == 'x'){
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                       body_id, body_point, utils::Vector3d(1,0,0), (name + "_X").c_str());
        }
        else if (axis.tolower()[i] == 'y'){
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                       body_id, body_point, utils::Vector3d(0,1,0), (name + "_Y").c_str());
        }
        else if (axis.tolower()[i] == 'z'){
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                       body_id, body_point, utils::Vector3d(0,0,1), (name + "_Z").c_str());
        }
        else {
            utils::Error::raise("Wrong axis!");
        }
    }
    m_rigidContacts->push_back(NodeSegment(body_point, name, "",true,false, axis,body_id));
    return ret;
}

unsigned int rigidbody::Contacts::AddLoopConstraint(
    unsigned int body_id_predecessor,
    unsigned int body_id_successor,
    const utils::RotoTrans &X_predecessor,
    const utils::RotoTrans &X_successor,
    const utils::SpatialVector &axis,
    const utils::String &name,
    bool enableStabilization,
    double stabilizationParam)
{
    ++*m_nbreConstraint;
    ++*m_nbLoopConstraint;
    return RigidBodyDynamics::ConstraintSet::AddLoopConstraint(
               body_id_predecessor, body_id_successor,
               RigidBodyDynamics::Math::SpatialTransform(X_predecessor.rot(),
                       X_predecessor.trans()),
               RigidBodyDynamics::Math::SpatialTransform(X_successor.rot(),
                       X_successor.trans()),
               utils::SpatialVector(axis),
               enableStabilization, stabilizationParam, name.c_str());
}

std::vector< utils::SpatialVector > rigidbody::Contacts::calcLoopConstraintForces(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau
)
{
    rigidbody::Joints& model = dynamic_cast<rigidbody::Joints&>(*this);
    return calcLoopConstraintForces(Q, Qdot, Tau, utils::ExternalForceSet(model));
}
std::vector< utils::SpatialVector > rigidbody::Contacts::calcLoopConstraintForces(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedTorque &Tau,
    const utils::ExternalForceSet &externalForces
)
{

    // all in the world frame
    bool resolveAllInRootFrame = true;

    // outputs
    std::vector< unsigned int > constraintBodyIdsOutput;
    std::vector< RigidBodyDynamics::Math::SpatialVector > updatedConstraintForcesOutput;
    std::vector< RigidBodyDynamics::Math::SpatialTransform > updatedConstraintBodyFramesOutput;

    // retrieve the model and the contacts
    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*>(this)->getConstraints();
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
    model.ForwardDynamicsConstraintsDirect(Q, Qdot, Tau, CS, externalForces);

    std::vector< utils::SpatialVector > output;
    for (int i=0; i<static_cast<int>(*m_nbLoopConstraint); i++) {
        constraintBodyIdsOutput.clear();
        updatedConstraintBodyFramesOutput.clear();
        updatedConstraintForcesOutput.clear();

        CS.calcForces(
                    i,
                    model,
                    Q,
                    Qdot,
                    constraintBodyIdsOutput,
                    updatedConstraintBodyFramesOutput,
                    updatedConstraintForcesOutput,
                    resolveAllInRootFrame,
                    false
                    );

        // save all the forces in the global reference frame applied on the predecessor segment
        output.push_back(updatedConstraintForcesOutput[0]);
    }
    return output;

}


rigidbody::Contacts::~Contacts()
{

}

rigidbody::Contacts &rigidbody::Contacts::getConstraints()
{
    if (!*m_isBinded) {
        // Assuming that this is also a Joints type (via BiorbdModel)
        const rigidbody::Joints &model =
            dynamic_cast<rigidbody::Joints &>(*this);
        Bind(model);
        *m_isBinded = true;
    }
    return *this;
}

bool rigidbody::Contacts::hasContacts() const
{
    if (*m_nbreConstraint>0) return
            true;
    else {
        return false;
    }
}

bool rigidbody::Contacts::hasLoopConstraints() const
{
    if (*m_nbLoopConstraint>0) return
            true;
    else {
        return false;
    }
}

unsigned int rigidbody::Contacts::nbContacts() const
{
    return *m_nbreConstraint;
}

unsigned int rigidbody::Contacts::nbLoopConstraints() const
{
    return *m_nbLoopConstraint;
}

std::vector<utils::String> rigidbody::Contacts::contactNames()
{
    std::vector<utils::String> names;
    for (auto name : RigidBodyDynamics::ConstraintSet::name) {
        names.push_back(name);
    }
    return names;
}

utils::String rigidbody::Contacts::contactName(unsigned int i)
{
    utils::Error::check(i<*m_nbreConstraint,
                                "Idx for contact names is too high..");
    return RigidBodyDynamics::ConstraintSet::name[i];
}

std::vector<int> rigidbody::Contacts::rigidContactAxisIdx(unsigned int contact_idx) const
{
    std::vector<int> list;

    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model =
        dynamic_cast<const rigidbody::Joints &>(*this);

    const utils::String& axis = rigidContact(contact_idx).axesToRemove();

    for (unsigned int i=0; i<axis.length(); ++i) {

        if      (axis.tolower()[i] == 'x'){
            list.push_back(0);
        }
        else if (axis.tolower()[i] == 'y'){
            list.push_back(1);
        }
        else if (axis.tolower()[i] == 'z'){
            list.push_back(2);
        }

    }
    return list;
}


std::vector<utils::Vector3d>
rigidbody::Contacts::constraintsInGlobal(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    // Output variable
    std::vector<utils::Vector3d> tp;


    // On each control, apply the rotation and save the position
    for (unsigned int i=0; i<contactConstraints.size(); ++i) {
        for (unsigned int j=0; j<contactConstraints[i]->getConstraintSize(); ++j) {
            tp.push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                             model, Q, contactConstraints[i]->getBodyIds()[0],
                             contactConstraints[i]->getBodyFrames()[0].r, updateKin));
#ifndef BIORBD_USE_CASADI_MATH
            updateKin = false;
#endif
        }
    }

    return tp;
}

utils::Vector3d rigidbody::Contacts::rigidContact(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::NodeSegment& c = rigidContact(idx);

    // Calculate the acceleration of the contact
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
            model, Q, c.parentId(), c, updateKin);
}

std::vector<utils::Vector3d>
rigidbody::Contacts::rigidContacts(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    // Output variable
    std::vector<utils::Vector3d> tp;

    // On each control, apply the rotation and save the position
    for (const rigidbody::NodeSegment& c : *m_rigidContacts) {
        tp.push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(
            model, Q, c.parentId(), c, updateKin)
        );
#ifndef BIORBD_USE_CASADI_MATH
        updateKin = false;
#endif
    }

    return tp;
}

utils::Vector3d rigidbody::Contacts::rigidContactVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::NodeSegment& c = rigidContact(idx);

    // Calculate the acceleration of the contact
    return RigidBodyDynamics::CalcPointVelocity(
            model, Q, Qdot, c.parentId(), c, updateKin);
}

std::vector<utils::Vector3d> rigidbody::Contacts::rigidContactsVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    // Output variable
    std::vector<utils::Vector3d> tp;

    // On each control, apply the Q, Qdot, Qddot and save the acceleration
    for (const rigidbody::NodeSegment& c : *m_rigidContacts) {
        tp.push_back(RigidBodyDynamics::CalcPointVelocity(
            model, Q, Qdot, c.parentId(), c, updateKin)
        );
#ifndef BIORBD_USE_CASADI_MATH
    updateKin = false;
#endif
    }

    return tp;
}

utils::Vector3d rigidbody::Contacts::rigidContactAcceleration(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::NodeSegment& c = rigidContact(idx);

    // Calculate the acceleration of the contact
    return RigidBodyDynamics::CalcPointAcceleration(
            model, Q, Qdot, Qddot, c.parentId(), c, updateKin);
}

std::vector<utils::Vector3d> rigidbody::Contacts::rigidContactsAcceleration(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    // Output variable
    std::vector<utils::Vector3d> tp;

    // On each control, apply the Q, Qdot, Qddot and save the acceleration
    for (const rigidbody::NodeSegment& c : *m_rigidContacts) {
        tp.push_back(RigidBodyDynamics::CalcPointAcceleration(
            model, Q, Qdot, Qddot, c.parentId(), c, updateKin)
        );
#ifndef BIORBD_USE_CASADI_MATH
    updateKin = false;
#endif
    }

    return tp;
}


utils::Vector rigidbody::Contacts::getForce() const
{
    return static_cast<utils::Vector>(this->force);
}

const std::vector<rigidbody::NodeSegment> &rigidbody::Contacts::rigidContacts() const
{
    return *m_rigidContacts;
}

const rigidbody::NodeSegment &rigidbody::Contacts::rigidContact(unsigned int idx) const
{
    return (*m_rigidContacts)[idx];
}

int rigidbody::Contacts::nbRigidContacts() const
{
    return m_rigidContacts->size();
}

int rigidbody::Contacts::contactSegmentBiorbdId(
        int idx) const
{
    utils::Error::check(idx < nbRigidContacts(),
                                "Idx for rigid contact Segment Id is too high..");

    // Assuming that this is also a joint type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<const rigidbody::Joints &>(*this);

    const rigidbody::NodeSegment& c = rigidContact(idx);

    return model.getBodyRbdlIdToBiorbdId(c.parentId());
}

std::vector<size_t> rigidbody::Contacts::segmentRigidContactIdx(
        int segment_idx) const
{
    // Output variable
    std::vector<size_t> indices;

    // On each rigidcontact, verify if it belongs to the segment specified
    for (int i=0; i<nbRigidContacts(); ++i)
        {
        if (contactSegmentBiorbdId(i) == segment_idx) {
            indices.push_back(i);
        }
    }

    return indices;
}

std::vector<RigidBodyDynamics::Math::SpatialVector>* rigidbody::Contacts::rigidContactToSpatialVector(
        const GeneralizedCoordinates& Q,
        std::vector<utils::Vector> *f_contacts,
        bool updateKin)
{
    if (!f_contacts){
        return nullptr;
    }

    std::vector<utils::SpatialVector> sp_tp = rigidContactToSpatialVector(Q, *f_contacts, updateKin);
    if (sp_tp.size() == 0) {
        return nullptr;
    }


    std::vector<RigidBodyDynamics::Math::SpatialVector>* out = new std::vector<RigidBodyDynamics::Math::SpatialVector>();
    for (auto& sp : sp_tp) {
        out->push_back(sp);
    }
    return out;
}

std::vector<utils::SpatialVector> rigidbody::Contacts::rigidContactToSpatialVector(
        const GeneralizedCoordinates& Q,
        std::vector<utils::Vector> f_contacts,
        bool updateKin)
{

    std::vector<utils::SpatialVector> out = std::vector<utils::SpatialVector>();
    if (f_contacts.size() == 0 || nbRigidContacts() == 0) {
        return out;
    }

    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints& model = dynamic_cast<rigidbody::Joints&>(*this);

#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        model.UpdateKinematicsCustom(&Q, nullptr, nullptr);
        updateKin = false;
    }

    utils::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
    out.push_back(sp_zero);
    for (size_t i = 0; i < model.nbSegment(); ++i){

        unsigned int nbRigidContactSegment = segmentRigidContactIdx(i).size();
        utils::SpatialVector tp(0.,0.,0.,0.,0.,0.);

        for (size_t j = 0; j < nbRigidContactSegment; ++j)
        {
            // Index of rigid contact
            unsigned int contact_index = segmentRigidContactIdx(i)[j];
            // Find the application point of the force
            utils::Vector3d x = rigidContact(Q, contact_index, updateKin);
            // Find the list of sorted index of normal enabled in .bioMod
            std::vector<int> rca_idx = rigidContactAxisIdx(contact_index);
            // Add the contribution of the force of this point
            tp += computeForceAtOrigin(x, rca_idx, f_contacts[contact_index]);
        }

        // Put all the force at zero before the last dof of the segment
        for (int j = 0; j < static_cast<int>(model.segment(i).nbDof()) - 1; ++j){
            out.push_back(sp_zero);
        }
        // Put all the force on the last dof of the segment
        out.push_back(tp);
    }
    return out;
}

utils::SpatialVector rigidbody::Contacts::computeForceAtOrigin(
        utils::Vector3d applicationPoint,
        std::vector<int> sortedAxisIndex,
        utils::Vector f_contact)
{

    utils::Vector3d force(0., 0., 0.);

    for  (size_t j = 0; j < sortedAxisIndex.size(); ++j)
    {
        // Fill only if contact normal is enabled in .bioMod
        // sorted in .BioMod
        unsigned int cur_axis=sortedAxisIndex[j];
        force.block(cur_axis, 0, 1, 1) = f_contact.block(j, 0, 1, 1);
    }

    utils::SpatialVector out(0., 0., 0., 0., 0., 0.);
    out.block(0, 0, 3, 1) = force.cross(- applicationPoint); // Transport to Origin (Bour's formula)
    out.block(3, 0, 3, 1) = force;

    return out;
}
