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

std::vector<RigidBodyDynamics::Math::SpatialVector> rigidbody::ExternalForceSet::computeRbdlSpatialVectors(
    rigidbody::Joints& updatedModel
) {
    if (hasExternalForceInLocalReferenceFrame()) throw std::runtime_error("local reference frame requires Q when computing the Spatial Vectors");
    if (m_useTranslationalForces) throw std::runtime_error("useTranslationalForce requires Q when computing the Spatial Vectors");
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires Q and Qdot when computing the Spatial Vectors");
    return computeRbdlSpatialVectors(
        updatedModel, 
        rigidbody::GeneralizedCoordinates(updatedModel), 
        rigidbody::GeneralizedVelocity(updatedModel)
    );
}

std::vector<RigidBodyDynamics::Math::SpatialVector> rigidbody::ExternalForceSet::computeRbdlSpatialVectors(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q
) {
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires Qdot when computing the Spatial Vectors");
    return computeRbdlSpatialVectors(updatedModel, Q, rigidbody::GeneralizedVelocity(updatedModel));
}

std::vector<RigidBodyDynamics::Math::SpatialVector> rigidbody::ExternalForceSet::computeRbdlSpatialVectors(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot
) {
    std::vector<utils::SpatialVector> tp(computeSpatialVectors(updatedModel, Q, Qdot));
    std::vector<RigidBodyDynamics::Math::SpatialVector> out;
    for (const auto& value : tp) {
        out.push_back(value);
    }
    return out;
}

std::vector<utils::SpatialVector> rigidbody::ExternalForceSet::computeSpatialVectors(
    rigidbody::Joints& updatedModel
) {
    if (hasExternalForceInLocalReferenceFrame()) throw std::runtime_error("local reference frame requires Q when computing the Spatial Vectors");
    if (m_useTranslationalForces) throw std::runtime_error("useTranslationalForce requires Q when computing the Spatial Vectors");
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires Q and Qdot when computing the Spatial Vectors");
    return computeSpatialVectors(
        updatedModel,
        rigidbody::GeneralizedCoordinates(updatedModel), 
        rigidbody::GeneralizedVelocity(updatedModel)
    );
}

std::vector<utils::SpatialVector> rigidbody::ExternalForceSet::computeSpatialVectors(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q
) {
    if (m_useSoftContacts) throw std::runtime_error("useSoftContacts requires Qdot when computing the Spatial Vectors");
    return computeSpatialVectors(updatedModel, Q, rigidbody::GeneralizedVelocity());
}

std::vector<utils::SpatialVector> rigidbody::ExternalForceSet::computeSpatialVectors(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot) 
{
    std::vector<utils::SpatialVector> out;
    for (const auto& value : m_externalForces) {
        out.push_back(value);
    }
    if (hasExternalForceInLocalReferenceFrame()) combineLocalReferenceFrameForces(updatedModel, Q, out);
    if (m_useTranslationalForces) combineTranslationalForces(updatedModel, Q, out);
    if (m_useSoftContacts) combineSoftContactForces(updatedModel, Q, Qdot, out);

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
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    std::vector<utils::SpatialVector>& out
)
{
    const auto& allGlobalJcs = updatedModel.allGlobalJCS(Q, false);

    for (size_t i = 0; i < m_externalForcesInLocal.size(); i++) {
        const auto& pair(m_externalForcesInLocal.get(static_cast<int>(i)));

        // Aliases to have a better referencing of the variables
        const utils::SpatialVector& vector(pair.first);
        const utils::RotoTransNode& node(pair.second);

        // Traverse the segment hierarchy from root to child to root until we get to a segment with at least one dof
        // as it is not possible to add forces on segment without degree of freedom
        const rigidbody::Segment* segment = &updatedModel.segment(node.parent());
        utils::RotoTrans segmentRotoTrans = utils::RotoTrans::Identity();
        do {
            segmentRotoTrans *= allGlobalJcs[updatedModel.getBodyBiorbdId(segment->name())];
            
            if (segment->nbDof() > 0) break;
            utils::Error::check(segment->parent().compare("root"), node.parent() + " should be attached to at least one segment with a degree of freedom.");
            segment = &updatedModel.segment(segment->parent());
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
        size_t dofIndex = updatedModel.segment(node.parent()).getLastDofIndexInGeneralizedCoordinates(updatedModel) + 1;
        out[dofIndex] += transportAtOrigin(utils::SpatialVector(momentInGrf, forceInGrf), pointOfApplication);
    }
    return;
}

void rigidbody::ExternalForceSet::combineTranslationalForces(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q, 
    std::vector<utils::SpatialVector>& out
) const
{
    // Do not waste time computing forces on empty vector
    if (m_translationalForces.size() == 0) return;

    for (auto& e : m_translationalForces) {    
        const rigidbody::NodeSegment& pointOfApplication = e.second;
        const rigidbody::Segment& segment(updatedModel.segment(pointOfApplication.parent()));
        size_t dofIndex = segment.getLastDofIndexInGeneralizedCoordinates(updatedModel) + 1;

        const utils::Vector3d& force = e.first;
        rigidbody::NodeSegment pointOfApplicationInGlobal(
            updatedModel.CalcBodyToBaseCoordinates(Q, static_cast<unsigned int>(segment.id()), pointOfApplication, false),
            pointOfApplication.Node::name(),
            pointOfApplication.parent(),
            pointOfApplication.isTechnical(),
            pointOfApplication.isAnatomical(),
            pointOfApplication.axesToRemoveAsString(),
            pointOfApplication.parentId()
        );
            
        // Add the force to the force vector (do not subtract 1 because 0 is the base)
        out[dofIndex] += transportForceAtOrigin(force, pointOfApplicationInGlobal);
    }
}

void rigidbody::ExternalForceSet::combineSoftContactForces(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    std::vector<utils::SpatialVector>& out
) const
{
    // Do not waste time computing forces on empty vector
    if (m_model.nbSoftContacts() == 0) return;
    
    for (size_t j = 0; j < m_model.nbSoftContacts(); j++) {
        rigidbody::SoftContactNode& contact(m_model.softContact(j));
        const rigidbody::Segment& segment(updatedModel.segment(contact.parent()));
        size_t dofIndex = segment.getLastDofIndexInGeneralizedCoordinates(updatedModel) + 1;

        // Add the force to the force vector (do not subtract 1 because 0 is the base)
        out[dofIndex] += contact.computeForceAtOrigin(updatedModel, Q, Qdot, false);
    }
}

utils::SpatialVector rigidbody::ExternalForceSet::transportForceAtOrigin(
    const utils::Vector3d& force,
    const rigidbody::NodeSegment& pointOfApplication) const
{
    utils::Vector3d forceTp;
    if (pointOfApplication.nbAxesToRemove() > 0){
        // Fill only if direction is enabled
        forceTp = utils::Vector3d(0., 0., 0.);
        for (auto axis : pointOfApplication.availableAxesIndices()){
            forceTp.block(axis, 0, 1, 1) = force.block(axis, 0, 1, 1);
        }
    } else {
        forceTp = force;
    }
    
    // Transport to Origin (Bour's formula)
    utils::SpatialVector out(0., 0., 0., 0., 0., 0.);
    out.block(0, 0, 3, 1) = forceTp.cross(-pointOfApplication);
    out.block(3, 0, 3, 1) = forceTp;

    return out;
}

utils::SpatialVector rigidbody::ExternalForceSet::transportAtOrigin(
    const utils::SpatialVector& v,
    const rigidbody::NodeSegment& pointOfApplication) const
{
    // Transport to Origin (Bour's formula)
    utils::SpatialVector out(transportForceAtOrigin(v.force(), pointOfApplication));
    out.block(0, 0, 3, 1) += v.moment();

    return out;
}
