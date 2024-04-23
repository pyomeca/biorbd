#define BIORBD_API_EXPORTS
#include "RigidBody/Joints.h"

#include <rbdl/rbdl_utils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/Matrix3d.h"
#include "Utils/Quaternion.h"
#include "Utils/RotoTrans.h"
#include "Utils/Rotation.h"
#include "Utils/Scalar.h"
#include "Utils/SpatialVector.h"
#include "Utils/SpatialTransform.h"
#include "Utils/String.h"

#include "RigidBody/ExternalForceSet.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Segment.h"
#include "RigidBody/Markers.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/MeshFace.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/Contacts.h"
#include "RigidBody/SoftContacts.h"


using namespace BIORBD_NAMESPACE;

rigidbody::Joints::Joints() :
    RigidBodyDynamics::Model(),
    m_segments(std::make_shared<std::vector<rigidbody::Segment>>()),
    m_nbRoot(std::make_shared<size_t>(0)),
    m_nbDof(std::make_shared<size_t>(0)),
    m_nbQ(std::make_shared<size_t>(0)),
    m_nbQdot(std::make_shared<size_t>(0)),
    m_nbQddot(std::make_shared<size_t>(0)),
    m_nRotAQuat(std::make_shared<size_t>(0)),
    m_isKinematicsComputed(std::make_shared<bool>(false)),
    m_totalMass(std::make_shared<utils::Scalar>(0))
{
    // Redefining gravity so it is on z by default
    this->gravity = utils::Vector3d (0, 0, -9.81);
}

rigidbody::Joints::Joints(const rigidbody::Joints &other) :
    RigidBodyDynamics::Model(other),
    m_segments(other.m_segments),
    m_nbRoot(other.m_nbRoot),
    m_nbDof(other.m_nbDof),
    m_nbQ(other.m_nbQ),
    m_nbQdot(other.m_nbQdot),
    m_nbQddot(other.m_nbQddot),
    m_nRotAQuat(other.m_nRotAQuat),
    m_isKinematicsComputed(other.m_isKinematicsComputed),
    m_totalMass(other.m_totalMass)
{

}

rigidbody::Joints::~Joints()
{

}

rigidbody::Joints rigidbody::Joints::DeepCopy() const
{
    rigidbody::Joints copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::Joints::DeepCopy(const rigidbody::Joints &other)
{
    static_cast<RigidBodyDynamics::Model&>(*this) = other;
    m_segments->resize(other.m_segments->size());
    for (size_t i=0; i<other.m_segments->size(); ++i) {
        (*m_segments)[i] = (*other.m_segments)[i].DeepCopy();
    }
    *m_nbRoot = *other.m_nbRoot;
    *m_nbDof = *other.m_nbDof;
    *m_nbQ = *other.m_nbQ;
    *m_nbQdot = *other.m_nbQdot;
    *m_nbQddot = *other.m_nbQddot;
    *m_nRotAQuat = *other.m_nRotAQuat;
    *m_isKinematicsComputed = *other.m_isKinematicsComputed;
    *m_totalMass = *other.m_totalMass;
}

size_t rigidbody::Joints::nbGeneralizedTorque() const
{
    return nbQddot();
}
size_t rigidbody::Joints::nbDof() const
{
    return *m_nbDof;
}

std::vector<utils::String> rigidbody::Joints::nameDof() const
{
    std::vector<utils::String> names;
    for (size_t i = 0; i < nbSegment(); ++i) {
        for (size_t j = 0; j < segment(i).nbDof(); ++j) {
            names.push_back(segment(i).name() + "_" + segment(i).nameDof(j));
        }
    }
    // Append Quaternion Q
    for (size_t i = 0; i < nbSegment(); ++i) {
        if (segment(i).isRotationAQuaternion()) {
            names.push_back(segment(i).name() + "_" + segment(i).nameDof(3));
        }
    }
    return names;
}

size_t rigidbody::Joints::nbQ() const
{
    return *m_nbQ;
}
size_t rigidbody::Joints::nbQdot() const
{
    return *m_nbQdot;
}
size_t rigidbody::Joints::nbQddot() const
{
    return *m_nbQddot;
}
size_t rigidbody::Joints::nbRoot() const
{
    return *m_nbRoot;
}

utils::Scalar rigidbody::Joints::mass() const
{
    return *m_totalMass;
}

size_t rigidbody::Joints::AddSegment(
    const utils::String &segmentName,
    const utils::String &parentName,
    const utils::String &translationSequence,
    const utils::String &rotationSequence,
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QdotRanges,
    const std::vector<utils::Range>& QddotRanges,
    const std::vector<utils::Scalar>& jointDampings,
    const rigidbody::SegmentCharacteristics& characteristics,
    const utils::RotoTrans& referenceFrame)
{
    rigidbody::Segment tp(
        *this, segmentName, parentName, translationSequence,
        rotationSequence, QRanges, QdotRanges, QddotRanges, jointDampings,
        characteristics,
        utils::SpatialTransform(referenceFrame.rot().transpose(), referenceFrame.trans())
    );
    if (this->GetBodyId(parentName.c_str()) == std::numeric_limits<unsigned int>::max()) {
        *m_nbRoot += tp.nbDof();    // If the segment name is "Root", add the number of DoF of root
    }
    *m_nbDof += tp.nbDof();
    *m_nbQ += tp.nbQ();
    *m_nbQdot += tp.nbQdot();
    *m_nbQddot += tp.nbQddot();

    if (tp.isRotationAQuaternion()) {
        ++*m_nRotAQuat;
    }

    *m_totalMass += characteristics.mMass; // Add the segment mass to the total body mass
    m_segments->push_back(tp);
    return 0;
}

size_t rigidbody::Joints::AddSegment(
    const utils::String &segmentName,
    const utils::String &parentName,
    const utils::String &seqR,
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QdotRanges,
    const std::vector<utils::Range>& QddotRanges,
    const std::vector<utils::Scalar>& jointDampings,
    const rigidbody::SegmentCharacteristics& characteristics,
    const utils::RotoTrans& referenceFrame)
{
    rigidbody::Segment tp(
        *this, 
        segmentName, 
        parentName, 
        seqR, 
        QRanges, 
        QdotRanges, 
        QddotRanges,
        jointDampings,
        characteristics, 
        utils::SpatialTransform(referenceFrame.rot().transpose(), referenceFrame.trans())
    );
    if (this->GetBodyId(parentName.c_str()) == std::numeric_limits<unsigned int>::max()) {
        *m_nbRoot += tp.nbDof();    //  If the name of the segment is "Root", add the number of DoF of root
    }
    *m_nbDof += tp.nbDof();

    *m_totalMass += characteristics.mMass; // Add the segment mass to the total body mass
    m_segments->push_back(tp);
    return 0;
}

utils::Vector3d rigidbody::Joints::getGravity() const
{
    return gravity;
}

void rigidbody::Joints::setGravity(
    const utils::Vector3d &newGravity)
{
    gravity = newGravity;
}

void rigidbody::Joints::updateSegmentCharacteristics(
    size_t idx,
    const rigidbody::SegmentCharacteristics& characteristics)
{
    utils::Error::check(idx < m_segments->size(), "Asked for a wrong segment (out of range)");
    (*m_segments)[idx].updateCharacteristics(*this, characteristics);
}

rigidbody::Segment& rigidbody::Joints::segment(
    size_t idx) const
{
    utils::Error::check(idx < m_segments->size(),
                                "Asked for a wrong segment (out of range)");
    return (*m_segments)[idx];
}

rigidbody::Segment &rigidbody::Joints::segment(
    const utils::String & name) const
{
    return segment(static_cast<size_t>(getBodyBiorbdId(name.c_str())));
}

std::vector<rigidbody::Segment>& rigidbody::Joints::segments() const
{
    return *m_segments;
}

rigidbody::GeneralizedTorque rigidbody::Joints::computeDampedTau(
    const rigidbody::GeneralizedVelocity& Qdot) const
{
    rigidbody::GeneralizedTorque dampings(*this);

    size_t count(0);
    for (auto& segment : *m_segments) {
        for (auto& damping : segment.jointDampings()) {
            dampings[count] = damping * Qdot[count];
            count++;
        }
    }
    return dampings;
}

size_t rigidbody::Joints::nbSegment() const
{
    return m_segments->size();
}

int rigidbody::Joints::getBodyBiorbdId(
        const utils::String &segmentName) const
{
    for (size_t i=0; i<m_segments->size(); ++i)
        if (!(*m_segments)[i].name().compare(segmentName)) {
            return static_cast<int>(i);
        }
    return -1;
}

int rigidbody::Joints::getBodyRbdlId(
        const utils::String &segmentName) const
{
    return GetBodyId(segmentName.c_str());
}

int rigidbody::Joints::getBodyRbdlIdToBiorbdId(
        const int idx) const
{
    // Assuming that this is also a joint type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<const rigidbody::Joints &>(*this);
    std::string bodyName = model.GetBodyName(idx);
    return model.getBodyBiorbdId(bodyName);
}

size_t rigidbody::Joints::getBodyBiorbdIdToRbdlId(
        const int idx) const
{
    return (*m_segments)[idx].id();
}

std::vector<std::vector<size_t> > rigidbody::Joints::getDofSubTrees()
{
    // initialize subTrees
    std::vector<std::vector<size_t> > subTrees;
    std::vector<size_t> subTree_empty;
    for (size_t j=0; j<this->mu.size(); ++j) {
        subTrees.push_back(subTree_empty);
    }

    // Get all dof without parent
    std::vector<size_t> dof_with_no_parent_id;
    for (size_t i=1; i<this->mu.size(); ++i) { // begin at 1 because 0 is its own parent in rbdl.
      if (this->lambda[i]==0) {
          dof_with_no_parent_id.push_back(i);
      }
    }

    // Get all subtrees of dofs without parents
    for (size_t i=0; i<dof_with_no_parent_id.size(); ++i) {
        size_t dof_id = dof_with_no_parent_id[i];

        // initialize subTrees_temp
        std::vector<std::vector<size_t> > subTrees_temp;
        for (size_t j=0; j<this->mu.size(); ++j) {
          subTrees_temp.push_back(subTree_empty);
        }

        std::vector<std::vector<size_t> > subTrees_temp_filled = recursiveDofSubTrees(subTrees_temp, dof_id);
        for (size_t j=0; j<subTrees_temp.size(); ++j) {
            if (subTrees_temp_filled[j].empty()) {
                continue;
            } else {
                subTrees[j].insert(subTrees[j].end(),
                                     subTrees_temp_filled[j].begin(),
                                     subTrees_temp_filled[j].end());
            }
        }

    }

    subTrees.erase(subTrees.begin());

    return  subTrees;
}

std::vector<std::vector<size_t> > rigidbody::Joints::recursiveDofSubTrees(
        std::vector<std::vector<size_t> >subTrees,
        size_t idx)
{
    size_t q_index_i = this->mJoints[idx].q_index;
    subTrees[idx].push_back(q_index_i);

    std::vector<std::vector<size_t> > subTrees_filled;
    subTrees_filled = subTrees;

    std::vector<unsigned int> child_idx = this->mu[idx];

    if (child_idx.size() > 0){
       for (size_t i=0; i<child_idx.size(); ++i) {
            size_t cur_child_id = child_idx[i];
            subTrees_filled = recursiveDofSubTrees(subTrees_filled, cur_child_id);
            std::vector<size_t> subTree_child = subTrees_filled[cur_child_id];

            subTrees_filled[idx].insert(subTrees_filled[idx].end(),
                                 subTree_child.begin(),
                                 subTree_child.end());
      }
    }

    return subTrees_filled;
}

std::vector<utils::RotoTrans> rigidbody::Joints::allGlobalJCS(
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    std::vector<utils::RotoTrans> out;
    for (size_t i=0; i<m_segments->size(); ++i) {
        out.push_back(model.globalJCS(Q, i, false));
    }
    return out;
}

utils::RotoTrans rigidbody::Joints::globalJCS(
    const rigidbody::GeneralizedCoordinates &Q,
    const utils::String &name, 
    bool updateKin)
{
    return globalJCS(Q, static_cast<size_t>(getBodyBiorbdId(name)), updateKin);
}

utils::RotoTrans rigidbody::Joints::globalJCS(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t idx, 
    bool updateKin)
{
    return this->CalcBodyWorldTransformation(Q, (*m_segments)[idx].id(), updateKin);
}

std::vector<utils::RotoTrans> rigidbody::Joints::localJCS()
const
{
    std::vector<utils::RotoTrans> out;
    for (size_t i=0; i<m_segments->size(); ++i) {
        out.push_back(localJCS(i));
    }
    return out;
}
utils::RotoTrans rigidbody::Joints::localJCS(
    const utils::String &name) const
{
    return localJCS(static_cast<size_t>(getBodyBiorbdId(name.c_str())));
}

utils::RotoTrans rigidbody::Joints::localJCS(
    const size_t idx) const
{
    return (*m_segments)[idx].localJCS();
}

utils::Vector3d rigidbody::Joints::CalcBodyToBaseCoordinates(
    const rigidbody::GeneralizedCoordinates& Q,
    utils::String segmentName,
    const utils::Vector3d &pointInLocal,
    bool updateKin)
{
    return this->CalcBodyToBaseCoordinates(Q, this->GetBodyId(segmentName.c_str()), pointInLocal, updateKin);
}

utils::Vector3d rigidbody::Joints::CalcBodyToBaseCoordinates(
    const rigidbody::GeneralizedCoordinates& Q,
    unsigned int bodyId,
    const utils::Vector3d &pointInLocal,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    return RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, bodyId, pointInLocal, false);
}

utils::Vector3d rigidbody::Joints::CalcPointVelocity(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    utils::String segmentName,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
    return this->CalcPointVelocity(Q, Qdot, this->GetBodyId(segmentName.c_str()), pointInLocal, updateKin);
}

utils::Vector3d rigidbody::Joints::CalcPointVelocity(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    unsigned int bodyId,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);

    return RigidBodyDynamics::CalcPointVelocity(model, Q, Qdot, bodyId, pointInLocal, false);
}

utils::SpatialVector rigidbody::Joints::CalcPointVelocity6D(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    utils::String segmentName,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
    return this->CalcPointVelocity6D(Q, Qdot, this->GetBodyId(segmentName.c_str()), pointInLocal, updateKin);
}

utils::SpatialVector rigidbody::Joints::CalcPointVelocity6D(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    unsigned int bodyId,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);

    return RigidBodyDynamics::CalcPointVelocity6D(model, Q, Qdot, bodyId, pointInLocal, false);
}


utils::Vector3d rigidbody::Joints::CalcPointAcceleration(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedAcceleration& Qddot,
    utils::String segmentName,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
    return this->CalcPointAcceleration(Q, Qdot, Qddot, this->GetBodyId(segmentName.c_str()), pointInLocal, updateKin);
}

utils::Vector3d rigidbody::Joints::CalcPointAcceleration(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedAcceleration& Qddot,
    unsigned int bodyId,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(
        updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr, updateKin ? &Qddot : nullptr);

    return RigidBodyDynamics::CalcPointAcceleration(model, Q, Qdot, Qddot, bodyId, pointInLocal, false);
}

utils::Matrix rigidbody::Joints::CalcPointJacobian(
    const rigidbody::GeneralizedCoordinates& Q,
    utils::String segmentName,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
    return this->CalcPointJacobian(Q, this->GetBodyId(segmentName.c_str()), pointInLocal, updateKin);
}

utils::Matrix rigidbody::Joints::CalcPointJacobian(
    const rigidbody::GeneralizedCoordinates& Q,
    unsigned int bodyId,
    const utils::Vector3d& pointInLocal,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    utils::Matrix out(3, this->nbQ());
    out.setZero();
    RigidBodyDynamics::CalcPointJacobian(model, Q, bodyId, pointInLocal, out, false);
    return out;
}

std::vector<rigidbody::NodeSegment> rigidbody::Joints::projectPoint(
    const rigidbody::GeneralizedCoordinates& Q,
    const std::vector<rigidbody::NodeSegment>& v,
    bool updateKin)
{
    // Assuming that this is also a marker type (via BiorbdModel)
    const rigidbody::Markers& marks = dynamic_cast<rigidbody::Markers&>(*this);

    // Security check
    utils::Error::check(
        marks.nbMarkers() == v.size(), "Number of marker must be equal to number of Vector3d");

#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    std::vector<rigidbody::NodeSegment> out;
    for (size_t i = 0; i < marks.nbMarkers(); ++i) {
        rigidbody::NodeSegment tp(marks.marker(i));
        if (tp.nbAxesToRemove() != 0) {
            // Project the marker by removing an axis by using the current pose of the model
            utils::Vector3d pos = v[i].applyRT(updatedModel.globalJCS(Q, tp.parent(), false).transpose());
            
            out.push_back(updatedModel.projectPoint(Q, pos, false));

        } else {
            // If there isn't any projection to perform, just use the actual marker
            out.push_back(v[i]);
        }
    }
    return out;
}

rigidbody::NodeSegment rigidbody::Joints::projectPoint(
    const rigidbody::GeneralizedCoordinates &Q,
    const utils::Vector3d &v,
    int segmentIdx,
    const utils::String& axesToRemove,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    // Create a marker
    const utils::String& segmentName(segment(static_cast<size_t>(segmentIdx)).name());
    rigidbody::NodeSegment node( 
        v.applyRT(updatedModel.globalJCS(Q, static_cast<size_t>(segmentIdx), false).transpose()),
        "tp", 
        segmentName, 
        true, 
        true,
        axesToRemove, 
        static_cast<int>(GetBodyId(segmentName.c_str()))
    );

    // Project and then reset in global
    return updatedModel.projectPoint(Q, node, false);
}

rigidbody::NodeSegment rigidbody::Joints::projectPoint(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::NodeSegment &n,
    bool updateKin)
{
    // Assuming that this is also a Marker type (via BiorbdModel)
    return dynamic_cast<rigidbody::Markers &>(*this).marker(Q, n, updateKin, true);
}

utils::Matrix rigidbody::Joints::projectPointJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    rigidbody::NodeSegment node,
    bool updateKin)
{
    // If the point has not been projected, there is no effect
    if (node.nbAxesToRemove() == 0) {
        // Return the value
        return utils::Matrix::Zero(3, static_cast<unsigned int>(nbQ()));
    }

#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    // Assuming that this is also a Marker type (via BiorbdModel)
    rigidbody::Markers &marks = dynamic_cast<rigidbody::Markers &> (*this);

    // Jacobian of the marker
    node.applyRT(updatedModel.globalJCS(Q, node.parent(), false).transpose());

    utils::Matrix G_tp(marks.markersJacobian(Q, node.parent(), utils::Vector3d(0,0,0), false));
    utils::Matrix JCor(utils::Matrix::Zero(9, static_cast<unsigned int>(nbQ())));
    updatedModel.CalcMatRotJacobian(Q, GetBodyId(node.parent().c_str()), utils::Matrix3d::Identity(), JCor, false);
    for (size_t n=0; n<3; ++n){
        if (node.isAxisKept(n)) {
            G_tp += JCor.block(static_cast<unsigned int>(n)*3,0,3, static_cast<unsigned int>(nbQ())) * node(static_cast<unsigned int>(n));
        }
    }

    return G_tp;
}

utils::Matrix rigidbody::Joints::projectPointJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    const utils::Vector3d &v,
    int segmentIdx,
    const utils::String& axesToRemove,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    // Find the point
    const rigidbody::NodeSegment& p(
        updatedModel.projectPoint(Q, v, segmentIdx, axesToRemove, false));
    updateKin = false;

    // Return the value
    return updatedModel.projectPointJacobian(Q, p, false);
}

std::vector<utils::Matrix> rigidbody::Joints::projectPointJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    const std::vector<rigidbody::NodeSegment> &v,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    // Gather the points
    const std::vector<rigidbody::NodeSegment>& tp(updatedModel.projectPoint(Q, v, false));
    updateKin = false;

    // Calculate the Jacobian if the point is not projected
    std::vector<utils::Matrix> G;

    for (size_t i=0; i<tp.size(); ++i) {
        // Actual marker
        G.push_back(updatedModel.projectPointJacobian(Q, rigidbody::NodeSegment(v[i]), false));
    }
    return G;
}

utils::SpatialTransform rigidbody::Joints::CalcBodyWorldTransformation (
    const rigidbody::GeneralizedCoordinates &Q,
    const size_t segmentIdx,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    if (segmentIdx >= updatedModel.fixed_body_discriminator) {
        size_t fbody_id = segmentIdx - static_cast<size_t>(updatedModel.fixed_body_discriminator);
        size_t parent_id = static_cast<size_t>(updatedModel.mFixedBodies[fbody_id].mMovableParent);
        utils::RotoTrans parentRT(
            updatedModel.X_base[parent_id].E.transpose(),
            updatedModel.X_base[parent_id].r);
        utils::RotoTransNode bodyRT(
            utils::RotoTrans(
                updatedModel.mFixedBodies[fbody_id].mParentTransform.E.transpose(),
                updatedModel.mFixedBodies[fbody_id].mParentTransform.r)
            , "", "");
        const utils::RotoTrans& transfo_tp = parentRT * bodyRT;
        return utils::SpatialTransform (transfo_tp.rot(), transfo_tp.trans());
    }

    return utils::SpatialTransform (updatedModel.X_base[segmentIdx].E.transpose(), updatedModel.X_base[segmentIdx].r);
}


// Get a segment's angular velocity
utils::Vector3d rigidbody::Joints::segmentAngularVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    size_t idx,
    bool updateKin)
{
    // Calculate the velocity of the point (0 0 0)
    return this->CalcPointVelocity6D(Q, Qdot, segment(idx).name(), utils::Vector3d(0, 0, 0), updateKin).block(0, 0, 3, 1);
}

utils::Vector3d rigidbody::Joints::CoM(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    // For each segment, find the CoM (CoM = sum(segment_mass * pos_com_seg) / total mass)
    const std::vector<rigidbody::NodeSegment>& com_segment(CoMbySegment(Q, updateKin));
    utils::Vector3d com(0, 0, 0);
    for (size_t i=0; i<com_segment.size(); ++i) {
        com += (*m_segments)[i].characteristics().mMass * com_segment[i];
    }

    // Divide by total mass
    com = com/this->mass();

    // Return the CoM
    return com;
}

utils::Vector3d rigidbody::Joints::angularMomentum(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    return CalcAngularMomentum(Q, Qdot, updateKin);
}

utils::Matrix rigidbody::Joints::massMatrix (
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    utils::Matrix massMatrix(static_cast<unsigned int>(nbQ()), static_cast<unsigned int>(nbQ()));
    massMatrix.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(updatedModel, Q, massMatrix, false);
    return massMatrix;
}

utils::Matrix rigidbody::Joints::massMatrixInverse (
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    int i = 0; // for loop purpose
    int j = 0; // for loop purpose
    utils::Matrix Minv(updatedModel.dof_count, updatedModel.dof_count);
    Minv.setZero();


    // First Forward Pass
    for (i = 1; i < updatedModel.mBodies.size(); i++) {
      updatedModel.I[i].setSpatialMatrix(updatedModel.IA[i]);
    }
    // End First Forward Pass

    // set F (n x 6 x n)
    utils::Matrix F_i(6, updatedModel.dof_count);
    F_i.setZero();
    // Fill a vector of matrix (6 x n)
    std::vector<utils::Matrix> F;
    for (i = 1; i < updatedModel.mBodies.size(); i++)
    {
        F.push_back(F_i);
    }

    // Backward Pass
    std::vector<std::vector<size_t>> subTrees = getDofSubTrees();
    for (i = static_cast<int>(updatedModel.mBodies.size() - 1); i > 0; i--)
    {    
        unsigned int q_index_i = static_cast<size_t>(updatedModel.mJoints[i].q_index);
        const std::vector<size_t>& sub_tree = subTrees[q_index_i];

        updatedModel.U[i] = updatedModel.IA[i] * updatedModel.S[i];
        updatedModel.d[i] = updatedModel.S[i].dot(updatedModel.U[i]);

        Minv(q_index_i, q_index_i) = 1.0 / (updatedModel.d[i]);

        for (j = 0; j < sub_tree.size(); j++) {
              utils::SpatialVector Ftemp(F[q_index_i].block(0, static_cast<unsigned int>(sub_tree[j]), 6, 1));
              Minv(q_index_i, static_cast<unsigned int>(sub_tree[j])) -= (1.0/updatedModel.d[i]) * updatedModel.S[i].transpose() * Ftemp;
        }

        size_t lambda = static_cast<size_t>(updatedModel.lambda[i]);
        size_t lambda_q_i = static_cast<size_t>(updatedModel.mJoints[lambda].q_index);
        if (lambda != 0) {
            for (j = 0; j < sub_tree.size(); j++) {
                F[q_index_i].block(0, static_cast<unsigned int>(sub_tree[j]), 6, 1) += updatedModel.U[i] * Minv.block(q_index_i, static_cast<unsigned int>(sub_tree[j]), 1, 1);

                F[lambda_q_i].block(0, static_cast<unsigned int>(sub_tree[j]), 6, 1) += updatedModel.X_lambda[i].toMatrixTranspose() * F[q_index_i].block(0, static_cast<unsigned int>(sub_tree[j]), 6, 1);
            }

            RigidBodyDynamics::Math::SpatialMatrix Ia = updatedModel.IA[i]
                - updatedModel.U[i]
                * (updatedModel.U[i] / updatedModel.d[i]).transpose();

#ifdef BIORBD_USE_CASADI_MATH
          updatedModel.IA[lambda]
            += updatedModel.X_lambda[i].toMatrixTranspose()
            * Ia * updatedModel.X_lambda[i].toMatrix();

#else
          updatedModel.IA[lambda].noalias()
            += updatedModel.X_lambda[i].toMatrixTranspose()
            * Ia * updatedModel.X_lambda[i].toMatrix();
#endif
        }
    }
    // End Backward Pass

    // Second Forward Pass
    for (i = 1; i < updatedModel.mBodies.size(); i++) {
      unsigned int q_index_i = updatedModel.mJoints[i].q_index;
      unsigned int lambda = updatedModel.lambda[i];
      unsigned int lambda_q_i = updatedModel.mJoints[lambda].q_index;

      utils::SpatialTransform X_lambda = updatedModel.X_lambda[i];

        if (lambda != 0){
            for (j = q_index_i; j < static_cast<int>(updatedModel.dof_count); j++) {
                utils::SpatialVector Ftemp(F[lambda_q_i].block(0, j, 6, 1));
                Minv(q_index_i, j) -=
                        (1.0/updatedModel.d[i]) * (updatedModel.U[i].transpose() * X_lambda.toMatrix()) * Ftemp;
            }

        }
        // F[i,:,i:] = np.outer(S,Minv[i,i:]) // could be simplified (S * M[q_index_i,q_index_i:]^T)
        for (j = q_index_i; j < static_cast<int>(updatedModel.dof_count); j++) {
                    F[q_index_i].block(0, j, 6, 1) = updatedModel.S[i] * Minv.block(q_index_i, j, 1, 1); // outer product
        }


        if (lambda != 0){
            //  F[i,:,i:] += Xmat.transpose() * F[lambda,:,i:]
            for (j = q_index_i; j < static_cast<int>(updatedModel.dof_count); j++) {
                F[q_index_i].block(0, j, 6, 1) +=
                        X_lambda.toMatrix() * F[lambda_q_i].block(0, j, 6, 1);
            }

        }
    }
    // End of Second Forward Pass
    // Fill in full matrix (currently only upper triangular)
    for (j = 0; j < static_cast<int>(updatedModel.dof_count); j++)
    {
        for (i = 0; i < static_cast<int>(updatedModel.dof_count); i++)
        {
            if (j < i) {
                    Minv(i, j) = Minv(j, i);
            }
        }
    }

    return Minv;
}

utils::Vector3d rigidbody::Joints::CoMdot(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);

    // For each segment, find the CoM
    utils::Vector3d com_dot(0,0,0);

    // CoMdot = sum(mass_seg * Jacobian * qdot)/total_mass
    for (const auto& segment : *m_segments) {
        utils::Matrix Jac = updatedModel.CalcPointJacobian(Q, segment.name(), segment.characteristics().mCenterOfMass, false);
        com_dot += ((Jac*Qdot) * segment.characteristics().mMass);
    }
    // Divide by total mass
    com_dot = com_dot/mass();

    // Return the velocity of CoM
    return com_dot;
}

utils::Vector3d rigidbody::Joints::CoMddot(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
    utils::Scalar mass;
    utils::Vector3d com, com_ddot;
    this->CalcCenterOfMass(
        Q, Qdot, &Qddot, mass, com, nullptr, &com_ddot, nullptr, nullptr, updateKin);

    // Return the acceleration of CoM
    return com_ddot;
}

utils::Matrix rigidbody::Joints::CoMJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    // Total jacobian
    utils::Matrix JacTotal(utils::Matrix::Zero(3,this->dof_count));

    // CoMdot = sum(mass_seg * Jacobian * qdot)/total_mass
    for (auto segment : *m_segments) {
        utils::Matrix Jac = updatedModel.CalcPointJacobian(Q, segment.name(), segment.characteristics().mCenterOfMass, false);
        JacTotal += segment.characteristics().mMass*Jac;
    }

    // Divide by total mass
    JacTotal /= this->mass();

    // Return the Jacobian of CoM
    return JacTotal;
}

void rigidbody::Joints::CalcCenterOfMass(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedAcceleration* Qddot,
    utils::Scalar& mass,
    utils::Vector3d& com,
    utils::Vector3d* comVelocity,
    utils::Vector3d* comAcceleration,
    utils::Vector3d* angularMomentum,
    utils::Vector3d* changeOfAngularMomentum,
    bool updateKin ) 
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr, updateKin ? Qddot : nullptr);

    RigidBodyDynamics::Utils::CalcCenterOfMass(
        updatedModel, 
        Q, 
        Qdot, 
        Qddot, 
        mass, 
        com, 
        comVelocity, 
        comAcceleration, 
        angularMomentum, 
        changeOfAngularMomentum, 
        false
    );

}

std::vector<rigidbody::NodeSegment> rigidbody::Joints::CoMbySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    std::vector<rigidbody::NodeSegment> out;
    for (size_t i=0; i<m_segments->size(); ++i) {
        out.push_back(updatedModel.CoMbySegment(Q, i, false));
    }
    return out;
}

utils::Matrix rigidbody::Joints::CoMbySegmentInMatrix(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> allCoM(CoMbySegment(Q, updateKin));

    utils::Matrix CoMs(3, static_cast<int>(allCoM.size()));
    for (size_t i=0; i<allCoM.size(); ++i) {
        CoMs.block(0, static_cast<unsigned int>(i), 3, 1) = allCoM[i];
    }
    return CoMs;
}


utils::Vector3d rigidbody::Joints::CoMbySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const size_t idx,
    bool updateKin)
{
    utils::Error::check(idx < m_segments->size(),"Choosen segment doesn't exist");

    return this->CalcBodyToBaseCoordinates(
        Q, static_cast<unsigned int>((*m_segments)[idx].id()), (*m_segments)[idx].characteristics().mCenterOfMass, updateKin
    );
}


std::vector<utils::Vector3d> rigidbody::Joints::CoMdotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);

    std::vector<utils::Vector3d> out;
    for (size_t i=0; i<m_segments->size(); ++i) {
        out.push_back(updatedModel.CoMdotBySegment(Q, Qdot, i, false));
    }
    return out;
}


utils::Vector3d rigidbody::Joints::CoMdotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const size_t idx,
    bool updateKin)
{
    // Position of the center of mass of segment i
    utils::Error::check(idx < m_segments->size(), "Choosen segment doesn't exist");

    return this->CalcPointVelocity(
        Q, 
        Qdot, 
        static_cast<unsigned int>((*m_segments)[idx].id()), 
        (*m_segments)[idx].characteristics().mCenterOfMass,
        updateKin
    );
}


std::vector<utils::Vector3d> rigidbody::Joints::CoMddotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr, updateKin ? &Qddot : nullptr);

    std::vector<utils::Vector3d> out;
    for (size_t i=0; i<m_segments->size(); ++i) {
        out.push_back(updatedModel.CoMddotBySegment(Q, Qdot, Qddot, i, false));
    }
    return out;
}


utils::Vector3d rigidbody::Joints::CoMddotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    const size_t idx,
    bool updateKin)
{
    utils::Error::check(idx < m_segments->size(), "Choosen segment doesn't exist");

    return this->CalcPointAcceleration(
        Q, 
        Qdot, 
        Qddot, 
        static_cast<unsigned int>((*m_segments)[idx].id()),
        (*m_segments)[idx].characteristics().mCenterOfMass,
        updateKin
    );
}

std::vector<std::vector<utils::Vector3d>> rigidbody::Joints::meshPoints(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    // Find the position of the segments
    const std::vector<utils::RotoTrans>& RT(updatedModel.allGlobalJCS(Q, false));

    // For all the segments
    std::vector<std::vector<utils::Vector3d>> out;
    for (size_t i=0; i<nbSegment(); ++i) {
        out.push_back(updatedModel.meshPoints(RT,i));
    }
    return out;
}
std::vector<utils::Vector3d> rigidbody::Joints::meshPoints(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t i,
    bool updateKin)
{
    // Find the position of the segments
    const std::vector<utils::RotoTrans>& RT(allGlobalJCS(Q, updateKin));

    return meshPoints(RT, i);
}

std::vector<utils::Matrix>
rigidbody::Joints::meshPointsInMatrix(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    const std::vector<utils::RotoTrans>& RT(allGlobalJCS(Q, updateKin));

    std::vector<utils::Matrix> all_points;
    for (size_t i=0; i<m_segments->size(); ++i) {
        utils::Matrix mat(3, mesh(i).nbVertex());
        for (size_t j=0; j<mesh(i).nbVertex(); ++j) {
            utils::Vector3d tp (mesh(i).point(j));
            tp.applyRT(RT[i]);
            mat.block(0, static_cast<unsigned int>(j), 3, 1) = tp;
        }
        all_points.push_back(mat);
    }
    return all_points;
}
std::vector<utils::Vector3d> rigidbody::Joints::meshPoints(
    const std::vector<utils::RotoTrans> &RT,
    size_t i) const
{

    // Gather the position of the meshings
    std::vector<utils::Vector3d> v;
    for (size_t j=0; j<mesh(i).nbVertex(); ++j) {
        utils::Vector3d tp (mesh(i).point(j));
        tp.applyRT(RT[i]);
        v.push_back(tp);
    }

    return v;
}

std::vector<std::vector<rigidbody::MeshFace>>
        rigidbody::Joints::meshFaces() const
{
    // Gather the position of the meshings for all the segments
    std::vector<std::vector<rigidbody::MeshFace>> v_all;
    for (size_t j=0; j<nbSegment(); ++j) {
        v_all.push_back(meshFaces(j));
    }
    return v_all;
}
const std::vector<rigidbody::MeshFace>
&rigidbody::Joints::meshFaces(size_t idx) const
{
    // Find the position of the meshings for a segment i
    return mesh(idx).faces();
}

std::vector<rigidbody::Mesh> rigidbody::Joints::mesh() const
{
    std::vector<rigidbody::Mesh> segmentOut;
    for (size_t i=0; i<nbSegment(); ++i) {
        segmentOut.push_back(mesh(i));
    }
    return segmentOut;
}

const rigidbody::Mesh &rigidbody::Joints::mesh(
    size_t idx) const
{
    return segment(idx).characteristics().mesh();
}

utils::Vector3d rigidbody::Joints::CalcAngularMomentum (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    utils::Vector3d com, angularMomentum;
    utils::Scalar mass;
    this->CalcCenterOfMass(
        Q, 
        Qdot, 
        nullptr, 
        mass, 
        com, 
        nullptr, 
        nullptr, 
        &angularMomentum, 
        nullptr, 
        updateKin
    );
    return angularMomentum;
}

utils::Vector3d rigidbody::Joints::CalcAngularMomentum (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
    utils::Vector3d com,  angularMomentum;
    utils::Scalar mass;
    this->CalcCenterOfMass(
        Q, 
        Qdot, 
        &Qddot, 
        mass, 
        com, 
        nullptr, 
        nullptr,
        &angularMomentum, 
        nullptr, 
        updateKin
    );
    return angularMomentum;
}

std::vector<utils::Vector3d>
rigidbody::Joints::CalcSegmentsAngularMomentum (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    utils::Scalar mass;
    utils::Vector3d com;
    this->CalcCenterOfMass (
        Q, 
        Qdot, 
        nullptr, 
        mass, 
        com, 
        nullptr, 
        nullptr, 
        nullptr, 
        nullptr, 
        updateKin
    );
    utils::SpatialTransform X_to_COM (RigidBodyDynamics::Math::Xtrans(com));

    std::vector<utils::Vector3d> h_segment;
    for (size_t i = 1; i < this->mBodies.size(); i++) {
        this->Ic[i] = this->I[i];
        this->hc[i] = this->Ic[i].toMatrix() * this->v[i];

        utils::SpatialVector h = this->X_lambda[i].applyTranspose (
                    this->hc[i]);
        if (this->lambda[i] != 0) {
            size_t j(i);
            do {
                j = this->lambda[j];
                h = this->X_lambda[j].applyTranspose (h);
            } while (this->lambda[j]!=0);
        }
        h = X_to_COM.applyAdjoint (h);
        h_segment.push_back(utils::Vector3d(h[0],h[1],h[2]));
    }

    return h_segment;
}

std::vector<utils::Vector3d>
rigidbody::Joints::CalcSegmentsAngularMomentum (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
    utils::Scalar mass;
    utils::Vector3d com;
    this->CalcCenterOfMass (
        Q, 
        Qdot, 
        &Qddot, 
        mass, 
        com,
        nullptr, 
        nullptr, 
        nullptr, 
        nullptr,
        updateKin
    );
    utils::SpatialTransform X_to_COM (RigidBodyDynamics::Math::Xtrans(com));

    std::vector<utils::Vector3d> h_segment;
    for (size_t i = 1; i < this->mBodies.size(); i++) {
        this->Ic[i] = this->I[i];
        this->hc[i] = this->Ic[i].toMatrix() * this->v[i];

        utils::SpatialVector h = this->X_lambda[i].applyTranspose (
                    this->hc[i]);
        if (this->lambda[i] != 0) {
            size_t j(i);
            do {
                j = this->lambda[j];
                h = this->X_lambda[j].applyTranspose (h);
            } while (this->lambda[j]!=0);
        }
        h = X_to_COM.applyAdjoint (h);
        h_segment.push_back(utils::Vector3d(h[0],h[1],h[2]));
    }

    return h_segment;
}

size_t rigidbody::Joints::nbQuat() const
{
    return *m_nRotAQuat;
}

rigidbody::GeneralizedVelocity rigidbody::Joints::computeQdot(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedCoordinates &Qdot,
    const utils::Scalar &k_stab)
{
    rigidbody::GeneralizedVelocity QdotOut(static_cast<int>(Q.size()));
    // Verify if there are quaternions, if not the derivate is directly Qdot
    if (!m_nRotAQuat) {
        QdotOut = Qdot;
        return QdotOut;
    }
    unsigned int cmpQuat(0);
    unsigned int cmpDof(0);
    for (size_t i=0; i<nbSegment(); ++i) {
        const rigidbody::Segment& segment_i = segment(i);
        if (segment_i.isRotationAQuaternion()) {
            // Extraire le quaternion
            utils::Quaternion quat_tp(
                Q(Q.size() - static_cast<unsigned int>(*m_nRotAQuat+cmpQuat)),
                Q.block(cmpDof + static_cast<unsigned int>(segment_i.nbDofTrans()), 0, 3, 1),
                k_stab);

            // Qdot for translation is actual Qdot
            QdotOut.block(cmpDof, 0, static_cast<unsigned int>(segment_i.nbDofTrans()), 1)
                = Qdot.block(cmpDof, 0, static_cast<unsigned int>(segment_i.nbDofTrans()), 1);

            // Get the 4d derivative for the quaternion part
            quat_tp.derivate(Qdot.block(cmpDof+ static_cast<unsigned int>(segment_i.nbDofTrans()), 0, 3, 1));
            QdotOut.block(cmpDof+ static_cast<unsigned int>(segment_i.nbDofTrans()), 0, 3, 1) = quat_tp.block(1,0,3,1);
            QdotOut(Q.size()- static_cast<unsigned int>(*m_nRotAQuat+cmpQuat)) = quat_tp(
                        0);// Placer dans le vecteur de sortie

            // Increment the number of done quaternions
            ++cmpQuat;
        } else {
            // If it's a normal, do what it usually does
            QdotOut.block(cmpDof, 0, static_cast<unsigned int>(segment_i.nbDof()), 1) =
                Qdot.block(cmpDof, 0, static_cast<unsigned int>(segment_i.nbDof()), 1);
        }
        cmpDof += static_cast<unsigned int>(segment_i.nbDof());
    }
    return QdotOut;
}

utils::Scalar rigidbody::Joints::KineticEnergy(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);

    return RigidBodyDynamics::Utils::CalcKineticEnergy(updatedModel, Q, Qdot, false);
}


utils::Scalar rigidbody::Joints::PotentialEnergy(
        const rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    return RigidBodyDynamics::Utils::CalcPotentialEnergy(updatedModel, Q, false);
}

utils::Scalar rigidbody::Joints::Lagrangian(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);

    utils::Scalar kinetic(RigidBodyDynamics::Utils::CalcKineticEnergy(updatedModel, Q, Qdot, false));
    utils::Scalar potential(RigidBodyDynamics::Utils::CalcPotentialEnergy(updatedModel, Q, false));
    return kinetic - potential;
}


utils::Scalar rigidbody::Joints::TotalEnergy(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);

    utils::Scalar kinetic(RigidBodyDynamics::Utils::CalcKineticEnergy(updatedModel, Q, Qdot, false));
    utils::Scalar potential(RigidBodyDynamics::Utils::CalcPotentialEnergy(updatedModel, Q, false));
    return kinetic + potential;
}

rigidbody::GeneralizedTorque rigidbody::Joints::InverseDynamics(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedAcceleration& Qddot
)
{
    rigidbody::ExternalForceSet forceSet(static_cast<BIORBD_NAMESPACE::Model&>(*this));
    return InverseDynamics(Q, Qdot, Qddot, forceSet);
}
rigidbody::GeneralizedTorque rigidbody::Joints::InverseDynamics(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedAcceleration& Qddot,
    rigidbody::ExternalForceSet& externalForces
)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(&Q, &Qdot);

    rigidbody::GeneralizedTorque Tau(nbGeneralizedTorque());
    auto fExt = externalForces.computeRbdlSpatialVectors(updatedModel, Q, Qdot);

    RigidBodyDynamics::InverseDynamics(updatedModel, Q, Qdot, Qddot, Tau, &fExt);
    return Tau - computeDampedTau(Qdot);
}

rigidbody::GeneralizedTorque rigidbody::Joints::NonLinearEffect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot
)
{
    rigidbody::ExternalForceSet forceSet(static_cast<BIORBD_NAMESPACE::Model&>(*this));
    return NonLinearEffect(Q, Qdot, forceSet);
}
rigidbody::GeneralizedTorque rigidbody::Joints::NonLinearEffect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    rigidbody::ExternalForceSet& externalForces
)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(&Q, &Qdot);

    rigidbody::GeneralizedTorque Tau(updatedModel);
    auto fExt = externalForces.computeRbdlSpatialVectors(updatedModel, Q, Qdot);
    RigidBodyDynamics::NonlinearEffects(updatedModel, Q, Qdot, Tau, &fExt);
    return Tau;
}

rigidbody::GeneralizedAcceleration rigidbody::Joints::ForwardDynamics(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau
)
{
    rigidbody::ExternalForceSet forceSet(static_cast<BIORBD_NAMESPACE::Model&>(*this));
    return ForwardDynamics(Q, Qdot, Tau, forceSet);
}
rigidbody::GeneralizedAcceleration rigidbody::Joints::ForwardDynamics(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau,
    rigidbody::ExternalForceSet& externalForces
)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(&Q, &Qdot);
    
    rigidbody::GeneralizedAcceleration Qddot(updatedModel);
    auto fExt = externalForces.computeRbdlSpatialVectors(updatedModel, Q, Qdot);
    rigidbody::GeneralizedTorque dampedTau = Tau - computeDampedTau(Qdot);
    
    RigidBodyDynamics::ForwardDynamics(updatedModel, Q, Qdot, dampedTau, Qddot, &fExt);
    return Qddot;
}

rigidbody::GeneralizedAcceleration rigidbody::Joints::ForwardDynamicsFreeFloatingBase(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedAcceleration& QddotJoints)
{

    utils::Error::check(QddotJoints.size() == this->nbQddot() - this->nbRoot(),
                        "Size of QddotJ must be equal to number of Qddot - number of root coordinates.");
    
    utils::Error::check(this->nbRoot() > 0, "Must have a least one degree of freedom on root.");

    rigidbody::GeneralizedAcceleration Qddot(this->nbQddot());
    rigidbody::GeneralizedAcceleration QRootDDot;
    rigidbody::GeneralizedTorque MassMatrixNlEffects;

    utils::Matrix massMatrixRoot = this->massMatrix(Q).block(0, 0, static_cast<unsigned int>(this->nbRoot()), static_cast<unsigned int>(this->nbRoot()));

    Qddot.block(0, 0, static_cast<unsigned int>(this->nbRoot()), 1) = utils::Vector(this->nbRoot()).setZero();
    Qddot.block(static_cast<unsigned int>(this->nbRoot()), 0, static_cast<unsigned int>(this->nbQddot()-this->nbRoot()), 1) = QddotJoints;

    MassMatrixNlEffects = InverseDynamics(Q, Qdot, Qddot);

#ifdef BIORBD_USE_CASADI_MATH
    auto linsol = casadi::Linsol("linsol", "symbolicqr", massMatrixRoot.sparsity());
    QRootDDot = linsol.solve(massMatrixRoot, -MassMatrixNlEffects.block(0, 0, static_cast<unsigned int>(this->nbRoot()), 1));
#else
    QRootDDot = massMatrixRoot.llt().solve(-MassMatrixNlEffects.block(0, 0, static_cast<unsigned int>(this->nbRoot()), 1));
#endif

    return QRootDDot;
}


rigidbody::GeneralizedAcceleration rigidbody::Joints::ForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau,
    bool updateKin
)
{
    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*>(this)->getConstraints();
    return ForwardDynamicsConstraintsDirect(Q, Qdot, Tau, CS, updateKin);
}

rigidbody::GeneralizedAcceleration rigidbody::Joints::ForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau,
    rigidbody::ExternalForceSet& externalForces,
    bool updateKin
)
{
    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*>(this)->getConstraints();
    return this->ForwardDynamicsConstraintsDirect(Q, Qdot, Tau, CS, externalForces, updateKin);
}

rigidbody::GeneralizedAcceleration rigidbody::Joints::ForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau,
    rigidbody::Contacts& CS,
    bool updateKin
)
{
    rigidbody::ExternalForceSet forceSet(static_cast<BIORBD_NAMESPACE::Model&>(*this));
    return ForwardDynamicsConstraintsDirect(Q, Qdot, Tau, CS, forceSet, updateKin);
}

rigidbody::GeneralizedAcceleration rigidbody::Joints::ForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau,
    rigidbody::Contacts& CS,
    rigidbody::ExternalForceSet& externalForces,
    bool updateKin
)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr, updateKin ? &Qdot : nullptr);
 
    auto fExt = externalForces.computeRbdlSpatialVectors(updatedModel, Q, Qdot);
    rigidbody::GeneralizedTorque dampedTau = Tau - computeDampedTau(Qdot);

    rigidbody::GeneralizedAcceleration Qddot(*this);
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(updatedModel, Q, Qdot, dampedTau, CS, Qddot, updateKin, &fExt);
    return Qddot;
}

utils::Vector rigidbody::Joints::ContactForcesFromForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau
)
{
    rigidbody::ExternalForceSet forceSet(static_cast<BIORBD_NAMESPACE::Model&>(*this));
    return ContactForcesFromForwardDynamicsConstraintsDirect(Q, Qdot, Tau, forceSet);
}
utils::Vector rigidbody::Joints::ContactForcesFromForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& Tau,
    rigidbody::ExternalForceSet& externalForces
)
{
    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*> (this)->getConstraints();
    this->ForwardDynamicsConstraintsDirect(Q, Qdot, Tau, CS, externalForces);
    return CS.getForce();
}

rigidbody::GeneralizedVelocity rigidbody::Joints::ComputeConstraintImpulsesDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QdotPre
)
{
    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*>(this)->getConstraints();
    if (CS.nbContacts() == 0) {
        return QdotPre;
    } else {
#ifdef BIORBD_USE_CASADI_MATH
        rigidbody::Joints model = this->DeepCopy();
#else
        rigidbody::Joints& model = *this;
#endif

        CS = dynamic_cast<rigidbody::Contacts*>(this)->getConstraints();

        rigidbody::GeneralizedVelocity QdotPost(model);
        RigidBodyDynamics::ComputeConstraintImpulsesDirect(model, Q, QdotPre, CS, QdotPost);
        return QdotPost;
    }
}

utils::Matrix3d rigidbody::Joints::bodyInertia (
        const rigidbody::GeneralizedCoordinates &q,
        bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(updateKin ? &q : nullptr);

    for (size_t i = 1; i < model.mBodies.size(); i++) {
        model.Ic[i] = model.I[i];
    }

    RigidBodyDynamics::Math::SpatialRigidBodyInertia Itot;

    for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
        size_t lambda = static_cast<size_t>(model.lambda[i]);

        if (lambda != 0) {
            model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose (model.Ic[i]);
        } else {
            Itot = Itot + model.X_lambda[i].applyTranspose (model.Ic[i]);
        }
    }

    utils::Vector3d com = Itot.h / Itot.m;
    return RigidBodyDynamics::Math::Xtrans(-com).applyTranspose(Itot).toMatrix().block(0, 0, 3, 3);
}

utils::Vector3d rigidbody::Joints::bodyAngularVelocity (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    utils::Vector3d com, angularMomentum;
    utils::Scalar mass;
    this->CalcCenterOfMass(
        Q, 
        Qdot, 
        nullptr, 
        mass, 
        com, 
        nullptr, 
        nullptr,
        &angularMomentum, 
        nullptr, 
        updateKin
    );
    utils::Matrix3d body_inertia = this->bodyInertia (Q, updateKin);
        
#ifdef BIORBD_USE_CASADI_MATH
    auto linsol = casadi::Linsol("linear_solver", "symbolicqr", body_inertia.sparsity());
    RigidBodyDynamics::Math::Vector3d out = linsol.solve(body_inertia, angularMomentum);
#else
    RigidBodyDynamics::Math::Vector3d out = body_inertia.colPivHouseholderQr().solve(angularMomentum);
#endif

    return out;
}

size_t rigidbody::Joints::getDofIndex(
    const utils::String& segmentName,
    const utils::String& dofName)
{
    size_t idx = 0;

    size_t iB = 0;
    bool found = false;
    while (1) {
        utils::Error::check(iB!=m_segments->size(), "Segment not found");

        if (segmentName.compare(  (*m_segments)[iB].name() )   ) {
            idx +=  (*m_segments)[iB].nbDof();
        } else {
            idx += (*m_segments)[iB].getDofIdx(dofName);
            found = true;
            break;
        }

        ++iB;
    }

    utils::Error::check(found, "Dof not found");
    return idx;
}

#ifdef BIORBD_USE_CASADI_MATH
rigidbody::Joints
#else
rigidbody::Joints&
#endif
rigidbody::Joints::UpdateKinematicsCustom(
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot,
    const rigidbody::GeneralizedAcceleration *Qddot)
{
    checkGeneralizedDimensions(Q, Qdot, Qddot);
    if (Q == nullptr && Qdot == nullptr && Qddot == nullptr) return *this;

#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints model = this->DeepCopy();
#else
    rigidbody::Joints& model = *this;
#endif
    RigidBodyDynamics::UpdateKinematicsCustom(model, Q, Qdot, Qddot);
    
    return model;
}

void rigidbody::Joints::CalcMatRotJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t segmentIdx,
    const utils::Matrix3d &rotation,
    utils::Matrix &G,
    bool updateKin)
{
#ifdef RBDL_ENABLE_LOGGING
    LOG << "-------- " << __func__ << " --------" << std::endl;
#endif


#ifdef BIORBD_USE_CASADI_MATH
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    model = this->UpdateKinematicsCustom(updateKin ? &Q : nullptr);
    updateKin = false;

    assert (G.rows() == 9 && G.cols() == model.qdot_size );

    std::vector<utils::Vector3d> axes;
    axes.push_back(utils::Vector3d(1,0,0));
    axes.push_back(utils::Vector3d(0,1,0));
    axes.push_back(utils::Vector3d(0,0,1));
    for (unsigned int iAxes=0; iAxes<3; ++iAxes) {
        utils::Matrix3d bodyMatRot (
            RigidBodyDynamics::CalcBodyWorldOrientation (model, Q, static_cast<unsigned int>(segmentIdx), updateKin).transpose());
        utils::SpatialTransform point_trans(
            utils::SpatialTransform (utils::Matrix3d::Identity(), bodyMatRot * rotation * *(axes.begin()+iAxes))
        );

        size_t reference_body_id = segmentIdx;
        if (model.IsFixedBodyId(static_cast<unsigned int>(segmentIdx))) {
            size_t fbody_id = segmentIdx - model.fixed_body_discriminator;
            reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
        }
        size_t j = reference_body_id;

        // e[j] is set to 1 if joint j contributes to the jacobian that we are
        // computing. For all other joints the column will be zero.
        while (j != 0) {
            unsigned int q_index = model.mJoints[j].q_index;
            // If it's not a DoF in translation (3 4 5 in model.S)
#ifdef BIORBD_USE_CASADI_MATH
            if (model.S[j].is_zero() && model.S[j](4).is_zero() && model.S[j](5).is_zero())
#else
            if (model.S[j](3)!=1.0 && model.S[j](4)!=1.0 && model.S[j](5)!=1.0)
#endif
            {
                utils::SpatialTransform X_base = model.X_base[j];
                X_base.r = utils::Vector3d(0,0,0); // Remove all concept of translation (only keep the rotation matrix)

                if (model.mJoints[j].mDoFCount == 3) {
                    G.block(iAxes*3, q_index, 3, 3) 
                        = ((point_trans * X_base.inverse()).toMatrix() * model.multdof3_S[j]).block(3,0,3,3);
                } else {
                    G.block(iAxes*3,q_index, 3, 1) 
                        = point_trans.apply(X_base.inverse().apply(model.S[j])).block(3,0,3,1);
                }
            }
            j = model.lambda[j]; // Pass to parent segment
        }
    }
}

utils::Matrix rigidbody::Joints::JacobianSegmentRotMat(
        const rigidbody::GeneralizedCoordinates &Q,
        size_t biorbdSegmentIdx,
        bool updateKin)
{
    size_t segmentIdx = getBodyBiorbdIdToRbdlId(static_cast<int>(biorbdSegmentIdx));

    utils::Matrix jacobianMat(utils::Matrix::Zero(9, static_cast<unsigned int>(nbQ())));
    this->CalcMatRotJacobian(Q, segmentIdx, utils::Matrix3d::Identity(), jacobianMat, updateKin);
    return jacobianMat;
}


void rigidbody::Joints::checkGeneralizedDimensions(
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot,
    const rigidbody::GeneralizedAcceleration *Qddot,
    const rigidbody::GeneralizedTorque *torque)
{
#ifndef SKIP_ASSERT
    if (Q) {
        utils::Error::check(
            Q->size() == nbQ(),
            "Wrong size for the Generalized Coordiates, " + 
            utils::String("expected ") + std::to_string(nbQ()) + " got " + std::to_string(Q->size()));
    }
    if (Qdot) {
        utils::Error::check(
            Qdot->size() == nbQdot(),
            "Wrong size for the Generalized Velocities, " +
            utils::String("expected ") + std::to_string(nbQdot()) + " got " + std::to_string(Qdot->size()));
    }
    if (Qddot) {
        utils::Error::check(
            Qddot->size() == nbQddot(),
            "Wrong size for the Generalized Accelerations, " +
            utils::String("expected ") + std::to_string(nbQddot()) + " got " + std::to_string(Qddot->size()));
    }

    if (torque) {
        utils::Error::check(
            torque->size() == nbGeneralizedTorque(),
            "Wrong size for the Generalized Torques, " +
            utils::String("expected ") + std::to_string(nbGeneralizedTorque()) + " got " + std::to_string(torque->size()));
    }
#endif
}
