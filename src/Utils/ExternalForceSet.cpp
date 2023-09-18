#define BIORBD_API_EXPORTS
#include "Utils/ExternalForceSet.h"

#include "RigidBody/Joints.h"
#include "Utils/SpatialVector.h"

using namespace BIORBD_NAMESPACE;

utils::ExternalForceSet::ExternalForceSet() :
    m_vectors(std::make_shared<std::vector<utils::SpatialVector>>())
{

}

utils::ExternalForceSet::~ExternalForceSet(){}

void utils::ExternalForceSet::set(
    const rigidbody::Joints& model,
    const std::string& segmentName,
    const utils::SpatialVector& v
) 
{
    model.GetBodyId(segmentName.c_str());
}

#ifdef BIORBD_USE_CASADI_MATH

void utils::ExternalForceSet::set(
    rigidbody::Joints& model,
    std::string& segmentName,
    const casadi::MX& v
)
{

}

void utils::ExternalForceSet::set(
    rigidbody::Joints& model,
    std::string& segmentName,
    const RBDLCasadiMath::MX_Xd_SubMatrix& m
)
{

}

#endif


//
//std::vector<RigidBodyDynamics::Math::SpatialVector>
//rigidbody::Joints::dispatchedForce(
//    std::vector<std::vector<utils::SpatialVector>>& spatialVector,
//    unsigned int frame) const
//{
//    // Iterator on the force table
//    std::vector<utils::SpatialVector>
//        sv2; // Gather in the same table the values at the same instant of different platforms
//    for (auto vec : spatialVector) {
//        sv2.push_back(vec[frame]);
//    }
//
//    // Call the equivalent function that only manages on instant
//    std::vector<RigidBodyDynamics::Math::SpatialVector>* sv_out = dispatchedForce(&sv2);
//    if (!sv_out) {
//        utils::Error::raise("Dispatch forces failed");
//    }
//    return *sv_out;
//}
//
//std::vector<RigidBodyDynamics::Math::SpatialVector>* rigidbody::Joints::dispatchedForce(
//    std::vector<utils::SpatialVector>* sv)
//    const  // a spatialVector per platform
//{
//    if (!sv) {
//        return nullptr;
//    }
//
//    // Output table
//    std::vector<RigidBodyDynamics::Math::SpatialVector>* sv_out = new std::vector<RigidBodyDynamics::Math::SpatialVector>();
//
//    // Null Spatial vector nul to fill the final table
//    utils::SpatialVector sv_zero(0., 0., 0., 0., 0., 0.);
//    sv_out->push_back(sv_zero); // The first one is associated with the universe
//
//    // Dispatch the forces
//    for (size_t i = 0; i < nbSegment(); ++i) {
//        auto& segment((*m_segments)[i]);
//        unsigned int nbDof = segment.nbDof();
//        if (nbDof == 0) { // Do not add anything if the nbDoF is zero
//            continue;
//        }
//
//        // For each segment
//        for (unsigned int i = 0; i < nbDof - 1; ++i) {
//            // Put a sv_zero on each DoF except the last one
//            sv_out->push_back(sv_zero);
//        }
//
//        if (segment.platformIdx() >= 0) {
//            // If the solid is in contact with the platform (!= -1)
//            sv_out->push_back((*sv)[static_cast<unsigned int>(segment.platformIdx())]);
//        }
//        else {
//            // Otherwise, put zero
//            sv_out->push_back(sv_zero);
//        }
//    }
//    return sv_out;
//}



std::vector<RigidBodyDynamics::Math::SpatialVector>* utils::ExternalForceSet::toRbdl(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    std::vector<utils::Vector>* contactsForces,
    bool updateKin,
    bool ignoreSoftContact
) const
{
    // TODO : Declare a pointer once, and reuse it. So there is no need to delete it until this class is destroy too?
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext = new std::vector<RigidBodyDynamics::Math::SpatialVector>();
    return f_ext;
}


utils::ExternalForceSet& utils::ExternalForceSet::combineExtForceAndSoftContact(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    std::vector<utils::Vector>* f_contacts,
    bool updateKin
) const 
{
    return utils::ExternalForceSet();
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
