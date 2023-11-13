#define BIORBD_API_EXPORTS
#include "RigidBody/ExternalForceSet.h"

#include "BiorbdModel.h"
#include "RigidBody/Contacts.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/Joints.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/SoftContacts.h"
#include "RigidBody/SoftContactNode.h"

#include "Utils/Error.h"
#include "Utils/SpatialVector.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"
#include "Utils/Rotation.h"
#include "Utils/RotoTransNode.h"

using namespace BIORBD_NAMESPACE;

rigidbody::ExternalForceSet::ExternalForceSet(
    Model& model, 
    bool useTranslationalForces, 
    bool useSoftContacts
) :
    m_model(model),
    m_useTranslationalForces(useTranslationalForces),
    m_useSoftContacts(useSoftContacts),
    m_externalForces(std::vector<utils::SpatialVector>()),
    m_externalForcesInLocal(rigidbody::ExternalForceSet::LocalForcesInternal()),
    m_translationalForces(std::vector<std::pair<utils::Vector3d, rigidbody::NodeSegment>>())
{
    setZero();
}

rigidbody::ExternalForceSet::~ExternalForceSet(){

}

void rigidbody::ExternalForceSet::add(
    const utils::String& segmentName,
    const utils::SpatialVector& vector
) 
{
    // Add 1 since 0 is used for the base ground 
    size_t dofIndex = m_model.segment(segmentName).getLastDofIndexInGeneralizedCoordinates(m_model) + 1;
    m_externalForces[dofIndex] += vector; 
}

void rigidbody::ExternalForceSet::add(
    const utils::String& segmentName,
    const utils::SpatialVector& vector, 
    const utils::Vector3d& pointOfApplication
)
{
    utils::SpatialVector atOrigin = transportAtOrigin(
        vector, 
        rigidbody::NodeSegment(utils::Vector3d(pointOfApplication, segmentName, segmentName))
    );
    add(segmentName, atOrigin);
}

void rigidbody::ExternalForceSet::addInSegmentReferenceFrame(
    const utils::String& segmentName,
    const utils::SpatialVector& vector,
    const utils::Vector3d& pointOfApplication
) {
    m_externalForcesInLocal.addNode(
        vector, 
        utils::RotoTransNode(
            utils::RotoTrans(utils::Matrix3d::Identity(), pointOfApplication), segmentName, segmentName
        )
    );
}

void rigidbody::ExternalForceSet::addTranslationalForce(
    const utils::Vector3d& force,
    const utils::String& segmentName,
    const utils::Vector3d& pointOfApplication
) 
{
    addTranslationalForce(force, rigidbody::NodeSegment(utils::Vector3d(pointOfApplication, segmentName, segmentName)));
}

void rigidbody::ExternalForceSet::addTranslationalForce(
    const utils::Vector3d& force,
    const rigidbody::NodeSegment& pointOfApplication
)
{
    if (!m_useTranslationalForces) throw std::runtime_error("It is not possible to add translational forces if useTranslationalForces was set to false");
    m_translationalForces.push_back(std::make_pair(force, pointOfApplication));
}

std::vector<RigidBodyDynamics::Math::SpatialVector> rigidbody::ExternalForceSet::computeRbdlSpatialVectors() {
    if (hasExternalForceInLocalReferenceFrame()) throw std::runtime_error("local reference frame requires Q when computing the Spatial Vectors");
    if (m_useTranslationalForces) throw std::runtime_error("useTranslationalForce requires Q when computing the Spatial Vectors");
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires Q and QDot when computing the Spatial Vectors");
    return computeRbdlSpatialVectors(rigidbody::GeneralizedCoordinates(m_model), rigidbody::GeneralizedVelocity(m_model), false);
}

std::vector<RigidBodyDynamics::Math::SpatialVector> rigidbody::ExternalForceSet::computeRbdlSpatialVectors(
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin
) {
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires QDot when computing the Spatial Vectors");
    return computeRbdlSpatialVectors(Q, rigidbody::GeneralizedVelocity(m_model), updateKin);
}

std::vector<RigidBodyDynamics::Math::SpatialVector> rigidbody::ExternalForceSet::computeRbdlSpatialVectors(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    bool updateKin
) {
    std::vector<utils::SpatialVector> tp(computeSpatialVectors(Q, QDot, updateKin));
    std::vector<RigidBodyDynamics::Math::SpatialVector> out;
    for (const auto& value : tp) {
        out.push_back(value);
    }
    return out;
}

std::vector<utils::SpatialVector> rigidbody::ExternalForceSet::computeSpatialVectors() {
    if (hasExternalForceInLocalReferenceFrame()) throw std::runtime_error("local reference frame requires Q when computing the Spatial Vectors");
    if (m_useTranslationalForces) throw std::runtime_error("useTranslationalForce requires Q when computing the Spatial Vectors");
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires Q and QDot when computing the Spatial Vectors");
    return computeSpatialVectors(rigidbody::GeneralizedCoordinates(m_model), rigidbody::GeneralizedVelocity(m_model), false);
}

std::vector<utils::SpatialVector> rigidbody::ExternalForceSet::computeSpatialVectors(
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin
) {
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires QDot when computing the Spatial Vectors");
    return computeSpatialVectors(Q, rigidbody::GeneralizedVelocity(), updateKin);
}

std::vector<utils::SpatialVector> rigidbody::ExternalForceSet::computeSpatialVectors(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    bool updateKin    
) 
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        m_model.UpdateKinematicsCustom(&Q, m_useSoftContacts ? &QDot : nullptr, nullptr);
    }

    std::vector<utils::SpatialVector> out;
    for (const auto& value : m_externalForces) {
        out.push_back(value);
    }
    if (hasExternalForceInLocalReferenceFrame()) combineLocalReferenceFrameForces(Q, out);
    if (m_useTranslationalForces) combineTranslationalForces(Q, out);
    if (m_useSoftContacts) combineSoftContactForces(Q, QDot, out);

    return out;
}


bool rigidbody::ExternalForceSet::hasExternalForceInLocalReferenceFrame() const {
    return m_externalForcesInLocal.size() > 0;
}

void rigidbody::ExternalForceSet::setZero()
{
    m_externalForces.clear();

    // Null Spatial vector nul to fill the final table
    utils::SpatialVector sv_zero(0., 0., 0., 0., 0., 0.);
    m_externalForces.push_back(sv_zero); // The first one is associated with the universe

    // Dispatch the forces
    for (size_t i = 0; i < m_model.nbSegment(); ++i) {
        size_t nDof(m_model.segment(i).nbDof());
        for (size_t i = 0; i < nDof; ++i) {
            m_externalForces.push_back(sv_zero); // Put a sv_zero on each DoF
        }
    }

    // Reset other elements of the class too
    m_translationalForces.clear();
    m_externalForcesInLocal.clear();
}

void rigidbody::ExternalForceSet::combineLocalReferenceFrameForces(
    const rigidbody::GeneralizedCoordinates& Q,
    std::vector<utils::SpatialVector>& out
)
{
    // NOTE: since combineExternalPushes is necessarily called from internal as protected method
    // we assume updateKinematics was already done
    bool updateKin(false);
    #ifdef BIORBD_USE_CASADI_MATH
updateKin = true;
    #endif
    const auto& allGlobalJcs = m_model.allGlobalJCS(Q, updateKin);

    for (size_t i = 0; i < m_externalForcesInLocal.size(); i++) {
        const auto& pair(m_externalForcesInLocal.get(static_cast<int>(i)));

        // Aliases to have a better referencing of the variables
        const utils::SpatialVector& vector(pair.first);
        const utils::RotoTransNode& node(pair.second);

        // Traverse the segment hierarchy from root to child to root until we get to a segment with at least one dof
        // as it is not possible to add forces on segment without degree of freedom
        const rigidbody::Segment* segment = &m_model.segment(node.parent());
        utils::RotoTrans segmentRotoTrans = utils::RotoTrans::Identity();
        do {
            segmentRotoTrans *= allGlobalJcs[m_model.getBodyBiorbdId(segment->name())];
            
            if (segment->nbDof() > 0) break;
            utils::Error::check(segment->parent().compare("root"), node.parent() + " should be attached to at least one segment with a degree of freedom.");
            segment = &m_model.segment(segment->parent());
        } while(true);
        const utils::RotoTransNode nodeInGrf(segmentRotoTrans * node);
        
        const utils::Rotation& rotationInGrf(nodeInGrf.rot());
        const utils::Vector3d& pointOfApplication(nodeInGrf.trans());

        // Rotate the forces in global reference frame (Grf)
        utils::Vector3d forceInGrf(vector.force());
        forceInGrf.applyRT(rotationInGrf);

        utils::Vector3d momentInGrf(vector.moment());
        momentInGrf.applyRT(rotationInGrf);

        // Transport the force to the global reference frame (Add 1 to account for the undeclared root)
        size_t dofIndex = m_model.segment(node.parent()).getLastDofIndexInGeneralizedCoordinates(m_model) + 1;
        out[dofIndex] += transportAtOrigin(utils::SpatialVector(momentInGrf, forceInGrf), pointOfApplication);
    }
    return;
}

void rigidbody::ExternalForceSet::combineTranslationalForces(
    const rigidbody::GeneralizedCoordinates& Q, 
    std::vector<utils::SpatialVector>& out
) const
{
    // NOTE: since combineExternalPushes is necessarily called from internal as protected method
    // we assume updateKinematics was already done
#ifdef BIORBD_USE_CASADI_MATH
    bool updateKin = true;
#else
    bool updateKin = false;
#endif

    // Do not waste time computing forces on empty vector
    if (m_translationalForces.size() == 0) return;

    for (size_t i = 0; i <m_model.nbSegment(); ++i) {
        const rigidbody::Segment& segment(m_model.segment(i));
        // Add 1 to account for the undeclared root
        size_t dofIndex = segment.getLastDofIndexInGeneralizedCoordinates(m_model) + 1;
        
        for (auto& e : m_translationalForces) {    
            const rigidbody::NodeSegment& pointOfApplication = e.second;
            if (pointOfApplication.parent().compare(segment.name())) continue;

            const utils::Vector3d& force = e.first;
            rigidbody::NodeSegment pointOfApplicationInGlobal(
                RigidBodyDynamics::CalcBodyToBaseCoordinates(
                    m_model, Q, static_cast<unsigned int>(segment.id()), pointOfApplication, updateKin
                ),
                pointOfApplication.Node::name(),
                pointOfApplication.parent(),
                pointOfApplication.isTechnical(),
                pointOfApplication.isAnatomical(),
                pointOfApplication.axesToRemove(),
                pointOfApplication.parentId()
            );
            
            // Add the force to the force vector (do not subtract 1 because 0 is the base)
            out[dofIndex] += transportForceAtOrigin(force, pointOfApplicationInGlobal);
        }
    }
}

void rigidbody::ExternalForceSet::combineSoftContactForces(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    std::vector<utils::SpatialVector>& out
) const
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


    for (size_t i = 0; i < m_model.nbSegment(); ++i) {
        const rigidbody::Segment& segment(m_model.segment(i));
        // Add 1 to account for the undeclared root
        size_t dofIndex = segment.getLastDofIndexInGeneralizedCoordinates(m_model) + 1;
    
        for (size_t j = 0; j < m_model.nbSoftContacts(); j++) {
            rigidbody::SoftContactNode& contact(m_model.softContact(j));
            if (contact.parent().compare(segment.name())) continue;

            // Add the force to the force vector (do not subtract 1 because 0 is the base)
            out[dofIndex] += contact.computeForceAtOrigin(m_model, Q, QDot, updateKin);
        }
    }
}

utils::SpatialVector rigidbody::ExternalForceSet::transportForceAtOrigin(
    const utils::Vector3d& force,
    const rigidbody::NodeSegment& pointOfApplication
) const
{
    // Fill only if direction is enabled
    utils::Vector3d newForce(0., 0., 0.);
    for (auto axis : pointOfApplication.availableAxesIndices()){
        newForce.block(axis, 0, 1, 1) = force.block(axis, 0, 1, 1);
    }

    // Transport to Origin (Bour's formula)
    utils::SpatialVector out(0., 0., 0., 0., 0., 0.);
    out.block(0, 0, 3, 1) = newForce.cross(-pointOfApplication);
    out.block(3, 0, 3, 1) = newForce;

    return out;
}

utils::SpatialVector rigidbody::ExternalForceSet::transportAtOrigin(
    const utils::SpatialVector& v,
    const rigidbody::NodeSegment& pointOfApplication
) const
{
    // Transport to Origin (Bour's formula)
    utils::SpatialVector out(transportForceAtOrigin(v.force(), pointOfApplication));
    out.block(0, 0, 3, 1) += v.moment();

    return out;
}
