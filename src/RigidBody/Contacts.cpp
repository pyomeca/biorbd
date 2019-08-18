#define BIORBD_API_EXPORTS
#include "RigidBody/Contacts.h"

#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Utils/Node.h"
#include "Utils/Attitude.h"
#include "RigidBody/Joints.h"

biorbd::rigidbody::Contacts::Contacts() :
    RigidBodyDynamics::ConstraintSet (),
    m_nbreConstraint(0),
    m_binded(false)
{

}

biorbd::rigidbody::Contacts::~Contacts()
{

}

unsigned int biorbd::rigidbody::Contacts::AddConstraint(
        unsigned int body_id,
        const biorbd::utils::Node& body_point,
        const biorbd::utils::Node& world_normal,
        const biorbd::utils::String& name,
        double acc){
    ++m_nbreConstraint;
    return RigidBodyDynamics::ConstraintSet::AddContactConstraint(body_id, body_point, world_normal, name.c_str(), acc);
}
unsigned int biorbd::rigidbody::Contacts::AddConstraint(
        unsigned int body_id,
        const biorbd::utils::Node& body_point,
        const biorbd::utils::String& axis,
        const biorbd::utils::String& name,
        double acc){
    unsigned int ret(0);
    for (unsigned int i=0; i<axis.length(); ++i){
        ++m_nbreConstraint;
        if      (axis.tolower()[i] == 'x')
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                        body_id, body_point, biorbd::utils::Node(1,0,0), (name + "_X").c_str(), acc);
        else if (axis.tolower()[i] == 'y')
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                        body_id, body_point, biorbd::utils::Node(0,1,0), (name + "_Y").c_str(), acc);
        else if (axis.tolower()[i] == 'z')
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(
                        body_id, body_point, biorbd::utils::Node(0,0,1), (name + "_Z").c_str(), acc);
        else
            biorbd::utils::Error::error(0,"Wrong axis!");
    }
    return ret;
}

unsigned int biorbd::rigidbody::Contacts::AddLoopConstraint(
        unsigned int body_id_predecessor,
        unsigned int body_id_successor,
        const biorbd::utils::Attitude &X_predecessor,
        const biorbd::utils::Attitude &X_successor,
        const biorbd::utils::Vector &axis,
        bool enableStabilization,
        double stabilizationParam,
        const biorbd::utils::String &name)
{
    ++m_nbreConstraint;
    return RigidBodyDynamics::ConstraintSet::AddLoopConstraint(
                body_id_predecessor, body_id_successor,
                RigidBodyDynamics::Math::SpatialTransform(X_predecessor.rot(), X_predecessor.trans()),
                RigidBodyDynamics::Math::SpatialTransform(X_successor.rot(), X_successor.trans()),
                RigidBodyDynamics::Math::SpatialVector(axis),
                enableStabilization, stabilizationParam, name.c_str());
}


biorbd::rigidbody::Contacts &biorbd::rigidbody::Contacts::getConstraints_nonConst(const biorbd::rigidbody::Joints &m){
    if (!m_binded){
        Bind(m);
        m_binded = true;
    }
    return *this;
}

biorbd::rigidbody::Contacts &biorbd::rigidbody::Contacts::getConstraints_nonConst()
{
    if (!m_binded)
        biorbd::utils::Error::error(0, "Please call getConstraints with biorbd::rigidbody::Joints model before!" );
    return *this;
}
const biorbd::rigidbody::Contacts &biorbd::rigidbody::Contacts::getConstraints() const {
    if (!m_binded)
        biorbd::utils::Error::error(0, "Please call getConstraints with biorbd::rigidbody::Joints model before!" );
    return *this;
}

bool biorbd::rigidbody::Contacts::hasContacts() const {
    if (m_nbreConstraint>0) return true; else return false;
}

unsigned int biorbd::rigidbody::Contacts::nContacts() const {
    return m_nbreConstraint;
}

biorbd::utils::String biorbd::rigidbody::Contacts::name(unsigned int i) {
    biorbd::utils::Error::error(i<m_nbreConstraint, "Idx for name is too high..");
    return RigidBodyDynamics::ConstraintSet::name[i];
}


std::vector<biorbd::utils::Node> biorbd::rigidbody::Contacts::constraintsInGlobal(
        biorbd::rigidbody::Joints& m,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin){
    if (updateKin)
        RigidBodyDynamics::UpdateKinematicsCustom(m, &Q, nullptr, nullptr);

    // Variable de sortie
    std::vector<biorbd::utils::Node> tp;


    // Sur chaque controle, appliquer la rotation et enregistrer sa position
    for (unsigned int i=0; i<size(); ++i)
        tp.push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(m, Q,body[i],point[i],false));

    return tp;
}

biorbd::utils::Vector biorbd::rigidbody::Contacts::getForce() const
{
    return this->force;
}
