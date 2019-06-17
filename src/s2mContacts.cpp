#define BIORBD_API_EXPORTS
#include "../include/s2mContacts.h"

s2mContacts::s2mContacts()
{
    m_nbreConstraint = 0;
    m_binded = false;
    //ctor
}

s2mContacts::~s2mContacts()
{
    //dtor
}

unsigned int s2mContacts::AddConstraint(unsigned int body_id, const RigidBodyDynamics::Math::Vector3d& body_point, const RigidBodyDynamics::Math::Vector3d& world_normal, const s2mString& name, double acc){
    ++m_nbreConstraint;
    return RigidBodyDynamics::ConstraintSet::AddContactConstraint(body_id, body_point, world_normal, name.c_str(), acc);
}
unsigned int s2mContacts::AddConstraint(unsigned int body_id, const RigidBodyDynamics::Math::Vector3d& body_point, const s2mString& axis, const s2mString& name, double acc){
    unsigned int ret(0);
    for (unsigned int i=0; i<axis.length(); ++i){
        ++m_nbreConstraint;
        if      (axis.tolower()[i] == 'x')
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(body_id, body_point, RigidBodyDynamics::Math::Vector3d(1,0,0), (name + "_X").c_str(), acc);
        else if (axis.tolower()[i] == 'y')
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(body_id, body_point, RigidBodyDynamics::Math::Vector3d(0,1,0), (name + "_Y").c_str(), acc);
        else if (axis.tolower()[i] == 'z')
            ret += RigidBodyDynamics::ConstraintSet::AddContactConstraint(body_id, body_point, RigidBodyDynamics::Math::Vector3d(0,0,1), (name + "_Z").c_str(), acc);
        else
            s2mError::s2mAssert(0,"Wrong axis!");
    }
    return ret;
}


RigidBodyDynamics::ConstraintSet& s2mContacts::getConstraints(const RigidBodyDynamics::Model& j){
    if (!m_binded){
        Bind(j);
        m_binded = true;
    }
    return *this;
}
RigidBodyDynamics::ConstraintSet& s2mContacts::getConstraints(){
    if (!m_binded)
        s2mError::s2mAssert(0, "Please call getConstraints with s2mJoints model before!" );
    return *this;
}


std::vector<Eigen::Vector3d> s2mContacts::constraintsInGlobal(s2mJoints& m, const s2mGenCoord &Q, const bool updateKin){
    if (updateKin)
        RigidBodyDynamics::UpdateKinematicsCustom(m, &Q, nullptr, nullptr);

    // Variable de sortie
    std::vector<Eigen::Vector3d> tp;


    // Sur chaque controle, appliquer la rotation et enregistrer sa position
    for (unsigned int i=0; i<size(); ++i)
        tp.push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(m, Q,body[i],point[i],false));

    return tp;
}
