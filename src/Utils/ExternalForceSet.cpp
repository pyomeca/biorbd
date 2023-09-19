#define BIORBD_API_EXPORTS
#include "Utils/ExternalForceSet.h"

#include "RigidBody/Joints.h"
#include "RigidBody/Segment.h"
#include "Utils/SpatialVector.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

utils::ExternalForceSet::ExternalForceSet(const rigidbody::Joints& model) :
    m_model(model),
    m_vectors(std::make_shared<std::vector<utils::SpatialVector>>()),
    m_rbdlFormattedForces(new std::vector<RigidBodyDynamics::Math::SpatialVector>())
{
    reset();
}

utils::ExternalForceSet::~ExternalForceSet(){
    delete m_rbdlFormattedForces;
}

void utils::ExternalForceSet::set(
    const std::string& segmentName,
    const utils::SpatialVector& v
) 
{
    int dofCount = 0; 
    for (size_t i = 0; i < m_model.nbSegment(); ++i) {
        auto& segment(m_model.segment(i));

        if (segment.name().compare(segmentName)) {
            dofCount += segment.nbDof();
            continue;
        }

        unsigned int nbDof = segment.nbDof();
        if (nbDof == 0) { 
            // It is not possible yet to add forces to a segment which does not have any dof
            throw "It is not possible to add forces to a segment without degree of freedom";
        }

        (*m_rbdlFormattedForces)[dofCount + nbDof] = v; // Do not subtract 1 since 0 is used for the base ground 
    }

}

#ifdef BIORBD_USE_CASADI_MATH

void utils::ExternalForceSet::set(
    std::string& segmentName,
    const casadi::MX& v
)
{

}

void utils::ExternalForceSet::set(
    std::string& segmentName,
    const RBDLCasadiMath::MX_Xd_SubMatrix& m
)
{

}

#endif


std::vector<RigidBodyDynamics::Math::SpatialVector>* utils::ExternalForceSet::toRbdl(
    const std::vector<utils::Vector>& externallyComputedForces) const
{
    return m_rbdlFormattedForces;
}


std::vector<RigidBodyDynamics::Math::SpatialVector>* utils::ExternalForceSet::toRbdlWithSoftContact(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    bool updateKin,
    const std::vector<utils::Vector>& externallyComputedForces
) const
{
    return m_rbdlFormattedForces;
}


utils::ExternalForceSet& utils::ExternalForceSet::combineExtForceAndSoftContact(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    bool updateKin,
    const std::vector<utils::Vector>& externallyComputedForces
) const 
{
    return utils::ExternalForceSet(m_model);
//#ifdef BIORBD_USE_CASADI_MATH
//    updateKin = true;
//#else
//    if (updateKin) {
//        UpdateKinematicsCustom(&Q, &QDot);
//    }
//    updateKin = false;
//#endif
//
//    std::vector<RigidBodyDynamics::Math::SpatialVector>* softContacts = dynamic_cast<rigidbody::SoftContacts*>(this)->softContactToSpatialVector(Q, QDot, updateKin);
//    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl = dispatchedForce(f_ext);
//    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_contacts_rbdl = dynamic_cast<rigidbody::Contacts*>(this)->rigidContactToSpatialVector(Q, f_contacts, updateKin);
//
//    if (!f_ext_rbdl && !softContacts && !f_contacts_rbdl) {
//        // Return a nullptr
//        return nullptr;
//    }
//
//    if (!f_ext_rbdl && !f_contacts_rbdl) {
//        // Return the softContacts (and nullptr if softContacts is nullptr)
//        return softContacts;
//    }
//
//    if (!softContacts && !f_contacts_rbdl) {
//        // Return the External forces
//        return f_ext_rbdl;
//    }
//
//    if (!f_ext_rbdl && !softContacts) {
//        // Return the contact forces
//        return f_contacts_rbdl;
//    }
//
//    if (!f_contacts_rbdl)
//    {
//        for (size_t i = 0; i < softContacts->size(); ++i) {
//            // Combine the external forces with the soft contacts
//            (*f_ext_rbdl)[i] += (*softContacts)[i];
//        }
//        return f_ext_rbdl;
//    }
//    if (!softContacts)
//    {
//        for (size_t i = 0; i < f_contacts_rbdl->size(); ++i) {
//            // Combine the external forces with the soft contacts
//            (*f_ext_rbdl)[i] += (*f_contacts_rbdl)[i];
//        }
//        return f_ext_rbdl;
//    }
//    if (!f_ext_rbdl)
//    {
//        for (size_t i = 0; i < f_contacts_rbdl->size(); ++i) {
//            // Combine the external forces with the soft contacts
//            (*softContacts)[i] += (*f_contacts_rbdl)[i];
//        }
//        return softContacts;
//    }
//    for (size_t i = 0; i < f_contacts_rbdl->size(); ++i) {
//        // Combine the external forces with the soft contacts
//        (*f_ext_rbdl)[i] += (*softContacts)[i];
//        (*f_ext_rbdl)[i] += (*f_contacts_rbdl)[i];
//    }
//
//    delete softContacts;
//    delete f_contacts_rbdl;
//    return f_ext_rbdl;
}

void utils::ExternalForceSet::reset()
{
    m_rbdlFormattedForces->clear();

    // Null Spatial vector nul to fill the final table
    utils::SpatialVector sv_zero(0., 0., 0., 0., 0., 0.);
    m_rbdlFormattedForces->push_back(sv_zero); // The first one is associated with the universe

    // Dispatch the forces
    for (size_t i = 0; i < m_model.nbSegment(); ++i) {
        unsigned int nDof(m_model.segment(i).nbDof());
        if (nDof == 0) continue;

        // For each segment
        for (unsigned int i = 0; i < nDof; ++i) {
            // Put a sv_zero on each DoF
            m_rbdlFormattedForces->push_back(sv_zero);
        }
    }
}
