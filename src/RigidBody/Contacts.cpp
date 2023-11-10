#define BIORBD_API_EXPORTS
#include "RigidBody/Contacts.h"

#include <rbdl/Kinematics.h>
#include "BiorbdModel.h"
#include "Utils/String.h"
#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/RotoTrans.h"
#include "Utils/Rotation.h"
#include "Utils/SpatialVector.h"
#include "Utils/String.h"

#include "RigidBody/ExternalForceSet.h"
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
    m_nbreConstraint(std::make_shared<size_t>(0)),
    m_isBinded(std::make_shared<bool>(false)),
    m_rigidContacts(std::make_shared<std::vector<rigidbody::NodeSegment>>()),
    m_nbLoopConstraint(std::make_shared<size_t>(0))
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

size_t rigidbody::Contacts::AddConstraint(
    size_t body_id,
    const utils::Vector3d& body_point,
    const utils::Vector3d& world_normal,
    const utils::String& name,
    const utils::String& parentName
)
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

    m_rigidContacts->push_back(
        NodeSegment(body_point, name, parentName,true,false, swapAxes(axis), static_cast<int>(body_id))
    );
    return static_cast<size_t>(
        RigidBodyDynamics::ConstraintSet::AddContactConstraint(
            static_cast<unsigned int>(body_id), body_point, world_normal, name.c_str()
        )
    );
}

size_t rigidbody::Contacts::AddConstraint(
    size_t body_id,
    const utils::Vector3d& body_point,
    const utils::String& axis,
    const utils::String& name,
    const utils::String& parentName)
{
    size_t ret(0);
    for (size_t i=0; i<axis.length(); ++i) {
        ++*m_nbreConstraint;
        if      (axis.tolower()[i] == 'x'){
            ret += static_cast<size_t>(
                RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                    static_cast<unsigned int>(body_id), body_point, utils::Vector3d(1,0,0), (name + "_X").c_str()
                )
            );
        }
        else if (axis.tolower()[i] == 'y'){
            ret += static_cast<size_t>(
                RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                    static_cast<unsigned int>(body_id), body_point, utils::Vector3d(0,1,0), (name + "_Y").c_str())
            );
        }
        else if (axis.tolower()[i] == 'z'){
            ret += static_cast<size_t>(
                RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                    static_cast<unsigned int>(body_id), body_point, utils::Vector3d(0,0,1), (name + "_Z").c_str())
            );
        }
        else {
            utils::Error::raise("Wrong axis!");
        }
    }
    
    m_rigidContacts->push_back(NodeSegment(body_point, name, parentName, true, false, swapAxes(axis), static_cast<int>(body_id)));
    return ret;
}

size_t rigidbody::Contacts::AddLoopConstraint(
    size_t body_id_predecessor,
    size_t body_id_successor,
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
        static_cast<unsigned int>(body_id_predecessor), static_cast<unsigned int>(body_id_successor),
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
    rigidbody::ExternalForceSet forceSet(static_cast<BIORBD_NAMESPACE::Model&>(*this));
    return calcLoopConstraintForces(Q, Qdot, Tau, forceSet);
}
std::vector< utils::SpatialVector > rigidbody::Contacts::calcLoopConstraintForces(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedTorque &Tau,
    rigidbody::ExternalForceSet &externalForces
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

size_t rigidbody::Contacts::nbContacts() const
{
    return *m_nbreConstraint;
}

size_t rigidbody::Contacts::nbLoopConstraints() const
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

utils::String rigidbody::Contacts::contactName(size_t i)
{
    utils::Error::check(i<*m_nbreConstraint,
                                "Idx for contact names is too high..");
    return RigidBodyDynamics::ConstraintSet::name[i];
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
    for (size_t i=0; i<contactConstraints.size(); ++i) {
        for (size_t j=0; j<contactConstraints[i]->getConstraintSize(); ++j) {
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
    size_t idx,
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
    size_t idx,
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
    size_t idx,
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

const rigidbody::NodeSegment &rigidbody::Contacts::rigidContact(size_t idx) const
{
    return (*m_rigidContacts)[idx];
}

int rigidbody::Contacts::nbRigidContacts() const
{
    return static_cast<int>(m_rigidContacts->size());
}

int rigidbody::Contacts::contactSegmentBiorbdId(
        int idx) const
{
    utils::Error::check(idx < nbRigidContacts(), "Idx for rigid contact Segment Id is too high..");

    // Assuming that this is also a joint type (via BiorbdModel)
    const rigidbody::NodeSegment& c = rigidContact(idx);

    const rigidbody::Joints &model = dynamic_cast<const rigidbody::Joints &>(*this);
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

utils::String rigidbody::Contacts::swapAxes(
    const utils::String& axesToSwap
) const {
    utils::String out;
    std::string reference = "xyz";
    std::set_difference(reference.begin(), reference.end(), axesToSwap.begin(), axesToSwap.end(), std::back_inserter(out));
    return out;
}