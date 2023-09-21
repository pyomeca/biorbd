#define BIORBD_API_EXPORTS
#include "Utils/ExternalForceSet.h"

#include "BiorbdModel.h"
#include "RigidBody/Contacts.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/Joints.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/SoftContacts.h"
#include "RigidBody/SoftContactNode.h"

#include "Utils/SpatialVector.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

using namespace BIORBD_NAMESPACE;

utils::ExternalForceSet::ExternalForceSet(Model& model) :
    m_model(model),
    m_vectors(std::make_shared<std::vector<utils::SpatialVector>>()),
    m_rbdlFormattedForces(new std::vector<RigidBodyDynamics::Math::SpatialVector>()),
    m_externalPush(std::vector<std::pair<utils::Vector3d, rigidbody::NodeSegment>>())
{
    reset();
}

utils::ExternalForceSet::~ExternalForceSet(){
    delete m_rbdlFormattedForces;
}

void utils::ExternalForceSet::set(
    const utils::String& segmentName,
    const utils::SpatialVector& v
) 
{
    int dofCount(0); 
    for (int i = 0; i < static_cast<int>(m_model.nbSegment()); ++i) {
        auto& segment(m_model.segment(i));
        
        dofCount += segment.nbDof();

        if (segment.name().compare(segmentName)) continue;
        if (segment.nbDof() == 0) {
            throw "It is not possible to add forces to a segment without degree of freedom";
        }

        (*m_rbdlFormattedForces)[dofCount] = v; // Do not subtract 1 since 0 is used for the base ground 
    }
}

#ifdef BIORBD_USE_CASADI_MATH

void utils::ExternalForceSet::set(
    utils::String& segmentName,
    const casadi::MX& v
)
{

}

void utils::ExternalForceSet::set(
    utils::String& segmentName,
    const RBDLCasadiMath::MX_Xd_SubMatrix& m
)
{

}

#endif

void utils::ExternalForceSet::addExternalPush(
    const utils::Vector3d& forces,
    const utils::String& segmentName,
    const utils::Vector3d& position
) 
{
    addExternalPush(forces, rigidbody::NodeSegment(utils::Vector3d(position, segmentName, segmentName)));
}

void utils::ExternalForceSet::addExternalPush(
    const utils::Vector3d& forces,
    const rigidbody::NodeSegment& position
)
{
    m_externalPush.push_back(std::make_pair(forces, position));
}


std::vector<RigidBodyDynamics::Math::SpatialVector>* utils::ExternalForceSet::toRbdl() const
{
    return m_rbdlFormattedForces;
}




void utils::ExternalForceSet::applyForces(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    bool updateKin,
    bool includeSoftContacts,
    bool includePushForces
) 
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        m_model.UpdateKinematicsCustom(&Q, includeSoftContacts ? &QDot : nullptr, nullptr);
    }

    if (includePushForces) combineExternalPushes(Q);
    if (includeSoftContacts) combineSoftContactForces(Q, QDot);
}

void utils::ExternalForceSet::applyPushForces(
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin
)
{
    applyForces(Q, rigidbody::GeneralizedVelocity(), updateKin, false);
}

void utils::ExternalForceSet::reset()
{
    m_rbdlFormattedForces->clear();

    // Null Spatial vector nul to fill the final table
    utils::SpatialVector sv_zero(0., 0., 0., 0., 0., 0.);
    m_rbdlFormattedForces->push_back(sv_zero); // The first one is associated with the universe

    // Dispatch the forces
    for (int i = 0; i < static_cast<int>(m_model.nbSegment()); ++i) {
        unsigned int nDof(m_model.segment(i).nbDof());
        for (unsigned int i = 0; i < nDof; ++i) {
            m_rbdlFormattedForces->push_back(sv_zero); // Put a sv_zero on each DoF
        }
    }

    // Reset other elements of the class too
    m_externalPush.clear();
}

void utils::ExternalForceSet::combineExternalPushes(
    const rigidbody::GeneralizedCoordinates& Q)
{
    // NOTE: since combineExternalPushes is necessarily called from internal as protected method
    // we assume updateKinematics was already done
#ifdef BIORBD_USE_CASADI_MATH
    bool updateKin = true;
#else
    bool updateKin = false;
#endif

    // Do not waste time computing forces on empty vector
    if (m_externalPush.size() == 0) return;

    int dofCount(0);
    for (int i = 0; i < static_cast<int>(m_model.nbSegment()); ++i) {
        const rigidbody::Segment& segment(m_model.segment(i));
        dofCount += segment.nbDof();
        
        for (auto& e : m_externalPush) {    
            const rigidbody::NodeSegment& position = e.second;
            if (position.parent().compare(segment.name())) continue;

            const utils::Vector3d& forces = e.first;
            rigidbody::NodeSegment positionInGlobal(
                RigidBodyDynamics::CalcBodyToBaseCoordinates(m_model, Q, segment.id(), position, updateKin),
                position.Node::name(),
                position.parent(),
                position.isTechnical(),
                position.isAnatomical(),
                position.axesToRemove(),
                position.parentId()
            );
            
            // Add the force to the force vector (do not subtract 1 because 0 is the base)
            (*m_rbdlFormattedForces)[dofCount] += transportForceAtOrigin(forces, positionInGlobal);
        }
    }
}

void utils::ExternalForceSet::combineSoftContactForces(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot
)
{
    // NOTE: since combineSoftContactForces is necessarily called from internal as protected method
    // we assume updateKinematics was already done
#ifdef BIORBD_USE_CASADI_MATH
    bool updateKin = true;
#else
    bool updateKin = false;
#endif

    // Do not waste time computing forces on empty vector
    if (m_model.nbSoftContacts() == 0) return;


    int dofCount(0);
    for (int i = 0; i < static_cast<int>(m_model.nbSegment()); ++i) {
        const rigidbody::Segment& segment(m_model.segment(i));
        dofCount += segment.nbDof();
    
        for (int j = 0; j < m_model.nbSoftContacts(); j++) {
            rigidbody::SoftContactNode& contact(m_model.softContact(j));
            if (contact.parent().compare(segment.name())) continue;

            // Add the force to the force vector (do not subtract 1 because 0 is the base)
            (*m_rbdlFormattedForces)[dofCount] += contact.computeForceAtOrigin(m_model, Q, QDot, updateKin);
        }
    }
}

utils::SpatialVector utils::ExternalForceSet::transportForceAtOrigin(
    const utils::Vector3d& forces,
    const rigidbody::NodeSegment& position
)
{
    // Fill only if direction is enabled
    utils::Vector3d force(0., 0., 0.);
    for (auto axis : position.availableAxesIndices()){
        force.block(axis, 0, 1, 1) = forces.block(axis, 0, 1, 1);
    }

    // Transport to Origin (Bour's formula)
    utils::SpatialVector out(0., 0., 0., 0., 0., 0.);
    out.block(0, 0, 3, 1) = force.cross(-position); 
    out.block(3, 0, 3, 1) = force;

    return out;
}
