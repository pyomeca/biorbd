#define BIORBD_API_EXPORTS
#include "RigidBody/Joints.h"

#include <rbdl/rbdl_utils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>
#include "Utils/Scalar.h"
#include "Utils/String.h"
#include "Utils/Quaternion.h"
#include "Utils/Matrix.h"
#include "Utils/Matrix3d.h"
#include "Utils/Error.h"
#include "Utils/RotoTrans.h"
#include "Utils/Rotation.h"
#include "Utils/SpatialVector.h"
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
    m_nbRoot(std::make_shared<unsigned int>(0)),
    m_nbDof(std::make_shared<unsigned int>(0)),
    m_nbQ(std::make_shared<unsigned int>(0)),
    m_nbQdot(std::make_shared<unsigned int>(0)),
    m_nbQddot(std::make_shared<unsigned int>(0)),
    m_nRotAQuat(std::make_shared<unsigned int>(0)),
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
    for (unsigned int i=0; i<other.m_segments->size(); ++i) {
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

unsigned int rigidbody::Joints::nbGeneralizedTorque() const
{
    return nbQddot();
}
unsigned int rigidbody::Joints::nbDof() const
{
    return *m_nbDof;
}

std::vector<utils::String> rigidbody::Joints::nameDof() const
{
    std::vector<utils::String> names;
    for (unsigned int i = 0; i < nbSegment(); ++i) {
        for (unsigned int j = 0; j < segment(i).nbDof(); ++j) {
            names.push_back(segment(i).name() + "_" + segment(i).nameDof(j));
        }
    }
    // Append Quaternion Q
    for (unsigned int i = 0; i < nbSegment(); ++i) {
        if (segment(i).isRotationAQuaternion()) {
            names.push_back(segment(i).name() + "_" + segment(i).nameDof(3));
        }
    }
    return names;
}

unsigned int rigidbody::Joints::nbQ() const
{
    return *m_nbQ;
}
unsigned int rigidbody::Joints::nbQdot() const
{
    return *m_nbQdot;
}
unsigned int rigidbody::Joints::nbQddot() const
{
    return *m_nbQddot;
}
unsigned int rigidbody::Joints::nbRoot() const
{
    return *m_nbRoot;
}

utils::Scalar rigidbody::Joints::mass() const
{
    return *m_totalMass;
}

unsigned int rigidbody::Joints::AddSegment(
    const utils::String &segmentName,
    const utils::String &parentName,
    const utils::String &translationSequence,
    const utils::String &rotationSequence,
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QDotRanges,
    const std::vector<utils::Range>& QDDotRanges,
    const rigidbody::SegmentCharacteristics& characteristics,
    const utils::RotoTrans& referenceFrame,
    int forcePlates)
{
    rigidbody::Segment tp(
        *this, segmentName, parentName, translationSequence,
        rotationSequence, QRanges, QDotRanges, QDDotRanges, characteristics,
        RigidBodyDynamics::Math::SpatialTransform(referenceFrame.rot().transpose(),
                referenceFrame.trans()),
        forcePlates);
    if (this->GetBodyId(parentName.c_str()) ==
            std::numeric_limits<unsigned int>::max()) {
        *m_nbRoot +=
            tp.nbDof();    // If the segment name is "Root", add the number of DoF of root
    }
    *m_nbDof += tp.nbDof();
    *m_nbQ += tp.nbQ();
    *m_nbQdot += tp.nbQdot();
    *m_nbQddot += tp.nbQddot();

    if (tp.isRotationAQuaternion()) {
        ++*m_nRotAQuat;
    }

    *m_totalMass +=
        characteristics.mMass; // Add the segment mass to the total body mass
    m_segments->push_back(tp);
    return 0;
}
unsigned int rigidbody::Joints::AddSegment(
    const utils::String &segmentName,
    const utils::String &parentName,
    const utils::String &seqR,
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QDotRanges,
    const std::vector<utils::Range>& QDDotRanges,
    const rigidbody::SegmentCharacteristics& characteristics,
    const utils::RotoTrans& referenceFrame,
    int forcePlates)
{
    rigidbody::Segment tp(
        *this, segmentName, parentName, seqR, QRanges, QDotRanges, QDDotRanges,
        characteristics, RigidBodyDynamics::Math::SpatialTransform(
            referenceFrame.rot().transpose(), referenceFrame.trans()),
        forcePlates);
    if (this->GetBodyId(parentName.c_str()) ==
            std::numeric_limits<unsigned int>::max()) {
        *m_nbRoot +=
            tp.nbDof();    //  If the name of the segment is "Root", add the number of DoF of root
    }
    *m_nbDof += tp.nbDof();

    *m_totalMass +=
        characteristics.mMass; // Add the segment mass to the total body mass
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
    unsigned int idx,
    const rigidbody::SegmentCharacteristics& characteristics)
{
    utils::Error::check(idx < m_segments->size(),
                                "Asked for a wrong segment (out of range)");
    (*m_segments)[idx].updateCharacteristics(*this, characteristics);
}

const rigidbody::Segment& rigidbody::Joints::segment(
    unsigned int idx) const
{
    utils::Error::check(idx < m_segments->size(),
                                "Asked for a wrong segment (out of range)");
    return (*m_segments)[idx];
}

const rigidbody::Segment &rigidbody::Joints::segment(
    const utils::String & name) const
{
    return segment(static_cast<unsigned int>(getBodyBiorbdId(name.c_str())));
}

const std::vector<rigidbody::Segment>& rigidbody::Joints::segments() const
{
    return *m_segments;
}

unsigned int rigidbody::Joints::nbSegment() const
{
    return static_cast<unsigned int>(m_segments->size());
}

std::vector<RigidBodyDynamics::Math::SpatialVector>
rigidbody::Joints::dispatchedForce(
    std::vector<std::vector<utils::SpatialVector>> &spatialVector,
    unsigned int frame) const
{
    // Iterator on the force table
    std::vector<utils::SpatialVector>
    sv2; // Gather in the same table the values at the same instant of different platforms
    for (auto vec : spatialVector) {
        sv2.push_back(vec[frame]);
    }

    // Call the equivalent function that only manages on instant
    std::vector<RigidBodyDynamics::Math::SpatialVector> *sv_out = dispatchedForce(&sv2);
    if (!sv_out){
        utils::Error::raise("Dispatch forces failed");
    }
    return *sv_out;
}

std::vector<RigidBodyDynamics::Math::SpatialVector> * rigidbody::Joints::dispatchedForce(
    std::vector<utils::SpatialVector> *sv)
const  // a spatialVector per platform
{
    if (!sv){
        return nullptr;
    }

    // Output table
    std::vector<RigidBodyDynamics::Math::SpatialVector> *sv_out = new std::vector<RigidBodyDynamics::Math::SpatialVector>();

    // Null Spatial vector nul to fill the final table
    utils::SpatialVector sv_zero(0.,0.,0.,0.,0.,0.);
    sv_out->push_back(sv_zero); // The first one is associated with the universe

    // Dispatch the forces
    for (size_t i=0; i<nbSegment(); ++i) {
        auto& segment ((*m_segments)[i]);
        unsigned int nbDof = segment.nbDof();
        if (nbDof == 0) { // Do not add anything if the nbDoF is zero
            continue;
        }

        // For each segment
        for (unsigned int i=0; i<nbDof-1; ++i) {
            // Put a sv_zero on each DoF except the last one
            sv_out->push_back(sv_zero);
        }

        if (segment.platformIdx() >= 0) {
            // If the solid is in contact with the platform (!= -1)
            sv_out->push_back((*sv)[static_cast<unsigned int>(segment.platformIdx())]);
        } else {
            // Otherwise, put zero
            sv_out->push_back(sv_zero);
        }
    }
    return sv_out;
}

std::vector<RigidBodyDynamics::Math::SpatialVector> * rigidbody::Joints::combineExtForceAndSoftContact(
        std::vector<utils::SpatialVector> *f_ext,
        std::vector<utils::Vector> *f_contacts,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& QDot,
        bool updateKin
        )
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#else
    if (updateKin){
        UpdateKinematicsCustom(&Q, &QDot);
    }
    updateKin = false;
#endif

    std::vector<RigidBodyDynamics::Math::SpatialVector>* softContacts = dynamic_cast<rigidbody::SoftContacts*>(this)->softContactToSpatialVector(Q, QDot, updateKin);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl = dispatchedForce(f_ext);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_contacts_rbdl = dynamic_cast<rigidbody::Contacts*>(this)->rigidContactToSpatialVector(Q, f_contacts, updateKin);

    if (!f_ext_rbdl && !softContacts && !f_contacts_rbdl){
        // Return a nullptr
        return nullptr;
    }

    if (!f_ext_rbdl && !f_contacts_rbdl){
        // Return the softContacts (and nullptr if softContacts is nullptr)
        return softContacts;
    }

    if (!softContacts && !f_contacts_rbdl){
        // Return the External forces
        return f_ext_rbdl;
    }

    if (!f_ext_rbdl && !softContacts){
        // Return the contact forces
        return f_contacts_rbdl;
    }

    if (!f_contacts_rbdl)
    {
    for (size_t i=0; i<softContacts->size(); ++i){
        // Combine the external forces with the soft contacts
        (*f_ext_rbdl)[i] += (*softContacts)[i];
    }
    return f_ext_rbdl;
    }
    if (!softContacts)
    {
    for (size_t i=0; i<f_contacts_rbdl->size(); ++i){
        // Combine the external forces with the soft contacts
        (*f_ext_rbdl)[i] += (*f_contacts_rbdl)[i];
    }
    return f_ext_rbdl;
    }
    if (!f_ext_rbdl)
    {
    for (size_t i=0; i<f_contacts_rbdl->size(); ++i){
        // Combine the external forces with the soft contacts
        (*softContacts)[i] += (*f_contacts_rbdl)[i];
    }
    return softContacts;
    }
    for (size_t i=0; i<f_contacts_rbdl->size(); ++i){
        // Combine the external forces with the soft contacts
        (*f_ext_rbdl)[i] += (*softContacts)[i];
        (*f_ext_rbdl)[i] += (*f_contacts_rbdl)[i];
    }

    delete softContacts;
    delete f_contacts_rbdl;
    return f_ext_rbdl;
}

int rigidbody::Joints::getBodyBiorbdId(
        const utils::String &segmentName) const
{
    for (int i=0; i<static_cast<int>(m_segments->size()); ++i)
        if (!(*m_segments)[static_cast<unsigned int>(i)].name().compare(segmentName)) {
            return i;
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

std::vector<std::vector<unsigned int> > rigidbody::Joints::getDofSubTrees()
{
    // initialize subTrees
    std::vector<std::vector<unsigned int> > subTrees;
    std::vector<unsigned int> subTree_empty;
    for (unsigned int j=0; j<this->mu.size(); ++j) {
        subTrees.push_back(subTree_empty);
    }

    // Get all dof without parent
    std::vector<unsigned int> dof_with_no_parent_id;
    for (unsigned int i=1; i<this->mu.size(); ++i) { // begin at 1 because 0 is its own parent in rbdl.
      if (this->lambda[i]==0) {
          dof_with_no_parent_id.push_back(i);
      }
    }

    // Get all subtrees of dofs without parents
    for (unsigned int i=0; i<dof_with_no_parent_id.size(); ++i) {
        unsigned int dof_id = dof_with_no_parent_id[i];

        // initialize subTrees_temp
        std::vector<std::vector<unsigned int> > subTrees_temp;
        for (unsigned int j=0; j<this->mu.size(); ++j) {
          subTrees_temp.push_back(subTree_empty);
        }

        std::vector<std::vector<unsigned int> > subTrees_temp_filled = recursiveDofSubTrees(subTrees_temp, dof_id);
        for (unsigned int j=0; j<subTrees_temp.size(); ++j) {
            if (subTrees_temp_filled[j].empty()) {
                continue;
            }
            else
            {
                subTrees[j].insert(subTrees[j].end(),
                                     subTrees_temp_filled[j].begin(),
                                     subTrees_temp_filled[j].end());
            }
        }

    }

    subTrees.erase(subTrees.begin());

    return  subTrees;
}

std::vector<std::vector<unsigned int> > rigidbody::Joints::recursiveDofSubTrees(
        std::vector<std::vector<unsigned int> >subTrees,
        unsigned int idx)
{
    unsigned int q_index_i = this->mJoints[idx].q_index;
    subTrees[idx].push_back(q_index_i);

    std::vector<std::vector<unsigned int> > subTrees_filled;
    subTrees_filled = subTrees;

    std::vector<unsigned int> child_idx = this->mu[idx];

    if (child_idx.size() > 0){
       for (unsigned int i=0; i<child_idx.size(); ++i) {
            unsigned int cur_child_id = child_idx[i];
            subTrees_filled = recursiveDofSubTrees(subTrees_filled, cur_child_id);
            std::vector<unsigned int> subTree_child = subTrees_filled[cur_child_id];

            subTrees_filled[idx].insert(subTrees_filled[idx].end(),
                                 subTree_child.begin(),
                                 subTree_child.end());
      }
    }

    return subTrees_filled;
}


std::vector<utils::RotoTrans> rigidbody::Joints::allGlobalJCS(
    const rigidbody::GeneralizedCoordinates &Q)
{
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return allGlobalJCS();
}

std::vector<utils::RotoTrans> rigidbody::Joints::allGlobalJCS()
const
{
    std::vector<utils::RotoTrans> out;
    for (unsigned int i=0; i<m_segments->size(); ++i) {
        out.push_back(globalJCS(i));
    }
    return out;
}

utils::RotoTrans rigidbody::Joints::globalJCS(
    const rigidbody::GeneralizedCoordinates &Q,
    const utils::String &name)
{
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return globalJCS(name);
}

utils::RotoTrans rigidbody::Joints::globalJCS(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx)
{
    // update the Kinematics if necessary
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return globalJCS(idx);
}

utils::RotoTrans rigidbody::Joints::globalJCS(
    const utils::String &name) const
{
    return globalJCS(static_cast<unsigned int>(getBodyBiorbdId(name)));
}

utils::RotoTrans rigidbody::Joints::globalJCS(
    unsigned int idx) const
{
    return CalcBodyWorldTransformation((*m_segments)[idx].id());
}

std::vector<utils::RotoTrans> rigidbody::Joints::localJCS()
const
{
    std::vector<utils::RotoTrans> out;

    for (unsigned int i=0; i<m_segments->size(); ++i) {
        out.push_back(localJCS(i));
    }

    return out;
}
utils::RotoTrans rigidbody::Joints::localJCS(
    const utils::String &name) const
{
    return localJCS(static_cast<unsigned int>(getBodyBiorbdId(name.c_str())));
}
utils::RotoTrans rigidbody::Joints::localJCS(
    const unsigned int idx) const
{
    return (*m_segments)[idx].localJCS();
}


std::vector<rigidbody::NodeSegment>
rigidbody::Joints::projectPoint(
    const rigidbody::GeneralizedCoordinates& Q,
    const std::vector<rigidbody::NodeSegment>& v,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom(&Q, nullptr, nullptr);
    }
    updateKin = false;

    // Assuming that this is also a marker type (via BiorbdModel)
    const rigidbody::Markers& marks =
        dynamic_cast<rigidbody::Markers&>(*this);

    // Sécurité
    utils::Error::check(marks.nbMarkers() == v.size(),
                                "Number of marker must be equal to number of Vector3d");

    std::vector<rigidbody::NodeSegment> out;
    for (unsigned int i = 0; i < marks.nbMarkers(); ++i) {
        rigidbody::NodeSegment tp(marks.marker(i));
        if (tp.nbAxesToRemove() != 0) {
            tp = v[i].applyRT(globalJCS(tp.parent()).transpose());
            // Prendre la position du nouveau marker avec les infos de celui du modèle
            out.push_back(projectPoint(Q, tp, updateKin));
        } else
            // S'il ne faut rien retirer (renvoyer tout de suite la même position)
        {
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
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom (&Q);
    }

    // Create a marker
    const utils::String& segmentName(segment(static_cast<unsigned int>
            (segmentIdx)).name());
    rigidbody::NodeSegment node( v.applyRT(globalJCS(
            static_cast<unsigned int>(segmentIdx)).transpose()), "tp",
                                         segmentName,
                                         true, true, axesToRemove, static_cast<int>(GetBodyId(segmentName.c_str())));

    // Project and then reset in global
    return projectPoint(Q, node, false);
}

rigidbody::NodeSegment rigidbody::Joints::projectPoint(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::NodeSegment &n,
    bool updateKin)
{
    // Assuming that this is also a Marker type (via BiorbdModel)
    return dynamic_cast<rigidbody::Markers &>(*this).marker(Q, n, true,
            updateKin);
}

utils::Matrix rigidbody::Joints::projectPointJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    rigidbody::NodeSegment node,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom (&Q);
    }
    updateKin = false;

    // Assuming that this is also a Marker type (via BiorbdModel)
    rigidbody::Markers &marks = dynamic_cast<rigidbody::Markers &>
                                        (*this);

    // If the point has not been projected, there is no effect
    if (node.nbAxesToRemove() != 0) {
        // Jacobian of the marker
        node.applyRT(globalJCS(node.parent()).transpose());
        utils::Matrix G_tp(marks.markersJacobian(Q, node.parent(),
                                   utils::Vector3d(0,0,0), updateKin));
        utils::Matrix JCor(utils::Matrix::Zero(9,nbQ()));
        CalcMatRotJacobian(Q, GetBodyId(node.parent().c_str()),
                           utils::Matrix3d::Identity(), JCor, updateKin);
        for (unsigned int n=0; n<3; ++n)
            if (node.isAxisKept(n)) {
                G_tp += JCor.block(n*3,0,3,nbQ()) * node(n);
            }

        return G_tp;
    } else {
        // Return the value
        return utils::Matrix::Zero(3,nbQ());
    }
}

utils::Matrix rigidbody::Joints::projectPointJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    const utils::Vector3d &v,
    int segmentIdx,
    const utils::String& axesToRemove,
    bool updateKin)
{
    // Find the point
    const rigidbody::NodeSegment& p(projectPoint(Q, v, segmentIdx,
                                            axesToRemove, updateKin));

    // Return the value
    return projectPointJacobian(Q, p, updateKin);
}

std::vector<utils::Matrix>
rigidbody::Joints::projectPointJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    const std::vector<rigidbody::NodeSegment> &v,
    bool updateKin)
{
    // Gather the points
    const std::vector<rigidbody::NodeSegment>& tp(projectPoint(Q, v,
            updateKin));

    // Calculate the Jacobian if the point is not projected
    std::vector<utils::Matrix> G;

    for (unsigned int i=0; i<tp.size(); ++i) {
        // Actual marker
        G.push_back(projectPointJacobian(Q, rigidbody::NodeSegment(v[i]),
                                         false));
    }
    return G;
}

RigidBodyDynamics::Math::SpatialTransform
rigidbody::Joints::CalcBodyWorldTransformation (
    const rigidbody::GeneralizedCoordinates &Q,
    const unsigned int segmentIdx,
    bool updateKin)
{
    // update the Kinematics if necessary
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom (&Q);
    }
    return CalcBodyWorldTransformation(segmentIdx);
}

RigidBodyDynamics::Math::SpatialTransform
rigidbody::Joints::CalcBodyWorldTransformation(
    const unsigned int segmentIdx) const
{
    if (segmentIdx >= this->fixed_body_discriminator) {
        unsigned int fbody_id = segmentIdx - this->fixed_body_discriminator;
        unsigned int parent_id = this->mFixedBodies[fbody_id].mMovableParent;
        utils::RotoTrans parentRT(
            this->X_base[parent_id].E.transpose(),
            this->X_base[parent_id].r);
        utils::RotoTrans bodyRT(
            this->mFixedBodies[fbody_id].mParentTransform.E.transpose(),
            this->mFixedBodies[fbody_id].mParentTransform.r);
        const utils::RotoTrans& transfo_tp = parentRT * bodyRT;
        return RigidBodyDynamics::Math::SpatialTransform (transfo_tp.rot(),
                transfo_tp.trans());
    }

    return RigidBodyDynamics::Math::SpatialTransform (
               this->X_base[segmentIdx].E.transpose(), this->X_base[segmentIdx].r);
}


// Get a segment's angular velocity
utils::Vector3d rigidbody::Joints::segmentAngularVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const utils::String& segmentName(segment(idx).name());
    unsigned int id(this->GetBodyId(segmentName.c_str()));

    // Calculate the velocity of the point
    return RigidBodyDynamics::CalcPointVelocity6D(
                *this, Q, Qdot, id, utils::Vector3d(0, 0, 0), updateKin).block(0, 0, 3, 1);
}

utils::Vector3d rigidbody::Joints::CoM(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    // For each segment, find the CoM (CoM = sum(segment_mass * pos_com_seg) / total mass)
    const std::vector<rigidbody::NodeSegment>& com_segment(CoMbySegment(Q,
            updateKin));
    utils::Vector3d com(0, 0, 0);
    for (unsigned int i=0; i<com_segment.size(); ++i) {
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
    updateKin = true;
#endif
    RigidBodyDynamics::Math::MatrixNd massMatrix(nbQ(), nbQ());
    massMatrix.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*this, Q, massMatrix, updateKin);
    return massMatrix;
}

utils::Matrix rigidbody::Joints::massMatrixInverse (
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    unsigned int i = 0; // for loop purpose
    unsigned int j = 0; // for loop purpose
    RigidBodyDynamics::Math::MatrixNd Minv(this->dof_count, this->dof_count);
    Minv.setZero();

#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom(&Q, nullptr, nullptr);
    }
    // First Forward Pass
    for (i = 1; i < this->mBodies.size(); i++) {

      this->I[i].setSpatialMatrix(this->IA[i]);
      }
    // End First Forward Pass

    // set F (n x 6 x n)
    RigidBodyDynamics::Math::MatrixNd F_i(6, this->dof_count);
    F_i.setZero();
    // Fill a vector of matrix (6 x n)
    std::vector<RigidBodyDynamics::Math::MatrixNd> F;
    for (i = 1; i < this->mBodies.size(); i++)
    {
        F.push_back(F_i);
    }

    // Backward Pass
    std::vector<std::vector<unsigned int>> subTrees = getDofSubTrees();
    for (i = this->mBodies.size() - 1; i > 0; i--)
    {    
        unsigned int q_index_i = this->mJoints[i].q_index;
        const std::vector<unsigned int>& sub_tree = subTrees[q_index_i];

        this->U[i] = this->IA[i] * this->S[i];
        this->d[i] = this->S[i].dot(this->U[i]);

        Minv(q_index_i, q_index_i) = 1.0 / (this->d[i]);

        for (j = 0; j < sub_tree.size(); j++) {
              const RigidBodyDynamics::Math::SpatialVector& Ftemp = F[q_index_i].block(0, sub_tree[j], 6, 1);
              Minv(q_index_i,sub_tree[j]) -= (1.0/this->d[i]) * this->S[i].transpose() * Ftemp;
        }

        unsigned int lambda = this->lambda[i];
        unsigned int lambda_q_i = this->mJoints[lambda].q_index;
        if (lambda != 0) {
            for (j = 0; j < sub_tree.size(); j++) {
                F[q_index_i].block(0, sub_tree[j], 6, 1) += this->U[i] * Minv.block(q_index_i, sub_tree[j], 1, 1);

                F[lambda_q_i].block(0, sub_tree[j], 6, 1) += this->X_lambda[i].toMatrixTranspose() * F[q_index_i].block(0, sub_tree[j], 6, 1);
            }

            RigidBodyDynamics::Math::SpatialMatrix Ia = this->IA[i]
                - this->U[i]
                * (this->U[i] / this->d[i]).transpose();

#ifdef BIORBD_USE_CASADI_MATH
          this->IA[lambda]
            += this->X_lambda[i].toMatrixTranspose()
            * Ia * this->X_lambda[i].toMatrix();

#else
          this->IA[lambda].noalias()
            += this->X_lambda[i].toMatrixTranspose()
            * Ia * this->X_lambda[i].toMatrix();
#endif
        }
    }
    // End Backward Pass

    // Second Forward Pass
    for (i = 1; i < this->mBodies.size(); i++) {
      unsigned int q_index_i = this->mJoints[i].q_index;
      unsigned int lambda = this->lambda[i];
      unsigned int lambda_q_i = this->mJoints[lambda].q_index;

      RigidBodyDynamics::Math::SpatialTransform X_lambda = this->X_lambda[i];

        if (lambda != 0){
            // Minv[i,i:] = Dinv[i]* (U[i,:].transpose() * Xmat) * F[lambda,:,i:])
            for (j = q_index_i; j < this->dof_count; j++) {
//                RigidBodyDynamics::Math::SpatialVector Ftemp = F[lambda_q_i].block(0, q_index_i, 6, 1);
                RigidBodyDynamics::Math::SpatialVector Ftemp = F[lambda_q_i].block(0, j, 6, 1);
                Minv(q_index_i, j) -=
                        (1.0/this->d[i]) * (this->U[i].transpose() * X_lambda.toMatrix()) * Ftemp;
            }

        }
        // F[i,:,i:] = np.outer(S,Minv[i,i:]) // could be simplified (S * M[q_index_i,q_index_i:]^T)
        for (j = q_index_i; j < this->dof_count; j++) {
                    F[q_index_i].block(0, j, 6, 1) = this->S[i] * Minv.block(q_index_i, j, 1, 1); // outer product
        }


        if (lambda != 0){
            //  F[i,:,i:] += Xmat.transpose() * F[lambda,:,i:]
            for (j = q_index_i; j < this->dof_count; j++) {
                F[q_index_i].block(0, j, 6, 1) +=
                        X_lambda.toMatrix() * F[lambda_q_i].block(0, j, 6, 1);
            }

        }
    }
    // End of Second Forward Pass
    // Fill in full matrix (currently only upper triangular)
    for (j = 0; j < this->dof_count; j++)
    {
        for (i = 0; i < this->dof_count; i++)
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
    updateKin = true;
#endif
    // For each segment, find the CoM
    utils::Vector3d com_dot(0,0,0);

    // CoMdot = sum(mass_seg * Jacobian * qdot)/mass totale
    utils::Matrix Jac(utils::Matrix(3,this->dof_count));
    for (auto segment : *m_segments) {
        Jac.setZero();
        RigidBodyDynamics::CalcPointJacobian(
            *this, Q, GetBodyId(segment.name().c_str()),
            segment.characteristics().mCenterOfMass, Jac, updateKin);
        com_dot += ((Jac*Qdot) * segment.characteristics().mMass);
        updateKin = false;
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
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    utils::Scalar mass;
    RigidBodyDynamics::Math::Vector3d com, com_ddot;
    RigidBodyDynamics::Utils::CalcCenterOfMass(
        *this, Q, Qdot, &Qddot, mass, com, nullptr, &com_ddot,
        nullptr, nullptr, updateKin);


    // Return the acceleration of CoM
    return com_ddot;
}

utils::Matrix rigidbody::Joints::CoMJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    // Total jacobian
    utils::Matrix JacTotal(utils::Matrix::Zero(3,this->dof_count));

    // CoMdot = sum(mass_seg * Jacobian * qdot)/mass total
    utils::Matrix Jac(utils::Matrix::Zero(3,this->dof_count));
    for (auto segment : *m_segments) {
        Jac.setZero();
        RigidBodyDynamics::CalcPointJacobian(
            *this, Q, GetBodyId(segment.name().c_str()),
            segment.characteristics().mCenterOfMass, Jac, updateKin);
        JacTotal += segment.characteristics().mMass*Jac;
        updateKin = false;
    }

    // Divide by total mass
    JacTotal /= this->mass();

    // Return the Jacobian of CoM
    return JacTotal;
}


std::vector<rigidbody::NodeSegment>
rigidbody::Joints::CoMbySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> out;
    for (unsigned int i=0; i<m_segments->size(); ++i) {
        out.push_back(CoMbySegment(Q,i,updateKin));
        updateKin = false;
    }
    return out;
}

utils::Matrix rigidbody::Joints::CoMbySegmentInMatrix(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> allCoM(CoMbySegment(Q, updateKin));
    utils::Matrix CoMs(3, allCoM.size());
    for (unsigned int i=0; i<allCoM.size(); ++i) {
        CoMs.block(0, i, 3, 1) = allCoM[i];
    }
    return CoMs;
}


utils::Vector3d rigidbody::Joints::CoMbySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const unsigned int idx,
    bool updateKin)
{
    utils::Error::check(idx < m_segments->size(),
                                "Choosen segment doesn't exist");
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
               *this, Q, (*m_segments)[idx].id(),
               (*m_segments)[idx].characteristics().mCenterOfMass, updateKin);
}


std::vector<utils::Vector3d> rigidbody::Joints::CoMdotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    std::vector<utils::Vector3d> out;
    for (unsigned int i=0; i<m_segments->size(); ++i) {
        out.push_back(CoMdotBySegment(Q,Qdot,i,updateKin));
        updateKin = false;
    }
    return out;
}


utils::Vector3d rigidbody::Joints::CoMdotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const unsigned int idx,
    bool updateKin)
{
    // Position of the center of mass of segment i
    utils::Error::check(idx < m_segments->size(),
                                "Choosen segment doesn't exist");
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    return CalcPointVelocity(
               *this, Q, Qdot, (*m_segments)[idx].id(),
               (*m_segments)[idx].characteristics().mCenterOfMass,updateKin);
}


std::vector<utils::Vector3d>
rigidbody::Joints::CoMddotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
    std::vector<utils::Vector3d> out;
    for (unsigned int i=0; i<m_segments->size(); ++i) {
        out.push_back(CoMddotBySegment(Q,Qdot,Qddot,i,updateKin));
        updateKin = false;
    }
    return out;
}


utils::Vector3d rigidbody::Joints::CoMddotBySegment(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    const unsigned int idx,
    bool updateKin)
{
    utils::Error::check(idx < m_segments->size(),
                                "Choosen segment doesn't exist");
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    return RigidBodyDynamics::CalcPointAcceleration(
               *this, Q, Qdot, Qddot, (*m_segments)[idx].id(),
               (*m_segments)[idx].characteristics().mCenterOfMass,updateKin);
}

std::vector<std::vector<utils::Vector3d>>
        rigidbody::Joints::meshPoints(
            const rigidbody::GeneralizedCoordinates &Q,
            bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom (&Q);
    }
    std::vector<std::vector<utils::Vector3d>> v;

    // Find the position of the segments
    const std::vector<utils::RotoTrans>& RT(allGlobalJCS());

    // For all the segments
    for (unsigned int i=0; i<nbSegment(); ++i) {
        v.push_back(meshPoints(RT,i));
    }

    return v;
}
std::vector<utils::Vector3d> rigidbody::Joints::meshPoints(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int i,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom (&Q);
    }

    // Find the position of the segments
    const std::vector<utils::RotoTrans>& RT(allGlobalJCS());

    return meshPoints(RT,i);
}

std::vector<utils::Matrix>
rigidbody::Joints::meshPointsInMatrix(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        UpdateKinematicsCustom (&Q);
    }
    const std::vector<utils::RotoTrans>& RT(allGlobalJCS());

    std::vector<utils::Matrix> all_points;
    for (unsigned int i=0; i<m_segments->size(); ++i) {
        utils::Matrix mat(3, mesh(i).nbVertex());
        for (unsigned int j=0; j<mesh(i).nbVertex(); ++j) {
            utils::Vector3d tp (mesh(i).point(j));
            tp.applyRT(RT[i]);
            mat.block(0, j, 3, 1) = tp;
        }
        all_points.push_back(mat);
    }
    return all_points;
}
std::vector<utils::Vector3d> rigidbody::Joints::meshPoints(
    const std::vector<utils::RotoTrans> &RT,
    unsigned int i) const
{

    // Gather the position of the meshings
    std::vector<utils::Vector3d> v;
    for (unsigned int j=0; j<mesh(i).nbVertex(); ++j) {
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
    for (unsigned int j=0; j<nbSegment(); ++j) {
        v_all.push_back(meshFaces(j));
    }
    return v_all;
}
const std::vector<rigidbody::MeshFace>
&rigidbody::Joints::meshFaces(unsigned int idx) const
{
    // Find the position of the meshings for a segment i
    return mesh(idx).faces();
}

std::vector<rigidbody::Mesh> rigidbody::Joints::mesh() const
{
    std::vector<rigidbody::Mesh> segmentOut;
    for (unsigned int i=0; i<nbSegment(); ++i) {
        segmentOut.push_back(mesh(i));
    }
    return segmentOut;
}

const rigidbody::Mesh &rigidbody::Joints::mesh(
    unsigned int idx) const
{
    return segment(idx).characteristics().mesh();
}

utils::Vector3d rigidbody::Joints::CalcAngularMomentum (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    RigidBodyDynamics::Math::Vector3d com,  angularMomentum;
    utils::Scalar mass;

    // Calculate the angular momentum with the function of the
    // position of the center of mass
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    RigidBodyDynamics::Utils::CalcCenterOfMass(
        *this, Q, Qdot, nullptr, mass, com, nullptr, nullptr,
        &angularMomentum, nullptr, updateKin);
    return angularMomentum;
}

utils::Vector3d rigidbody::Joints::CalcAngularMomentum (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
    // Definition of the variables
    RigidBodyDynamics::Math::Vector3d com,  angularMomentum;
    utils::Scalar mass;

    // Calculate the angular momentum with the function of the
    // position of the center of mass
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    RigidBodyDynamics::Utils::CalcCenterOfMass(
        *this, Q, Qdot, &Qddot, mass, com, nullptr, nullptr,
        &angularMomentum, nullptr, updateKin);

    return angularMomentum;
}

std::vector<utils::Vector3d>
rigidbody::Joints::CalcSegmentsAngularMomentum (
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    utils::Scalar mass;
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Utils::CalcCenterOfMass (
        *this, Q, Qdot, nullptr, mass, com, nullptr,
        nullptr, nullptr, nullptr, updateKin);
    RigidBodyDynamics::Math::SpatialTransform X_to_COM (
        RigidBodyDynamics::Math::Xtrans(com));

    std::vector<utils::Vector3d> h_segment;
    for (unsigned int i = 1; i < this->mBodies.size(); i++) {
        this->Ic[i] = this->I[i];
        this->hc[i] = this->Ic[i].toMatrix() * this->v[i];

        RigidBodyDynamics::Math::SpatialVector h = this->X_lambda[i].applyTranspose (
                    this->hc[i]);
        if (this->lambda[i] != 0) {
            unsigned int j(i);
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
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    utils::Scalar mass;
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Utils::CalcCenterOfMass (*this, Q, Qdot, &Qddot, mass, com,
            nullptr, nullptr, nullptr, nullptr,
            updateKin);
    RigidBodyDynamics::Math::SpatialTransform X_to_COM (
        RigidBodyDynamics::Math::Xtrans(com));

    std::vector<utils::Vector3d> h_segment;
    for (unsigned int i = 1; i < this->mBodies.size(); i++) {
        this->Ic[i] = this->I[i];
        this->hc[i] = this->Ic[i].toMatrix() * this->v[i];

        RigidBodyDynamics::Math::SpatialVector h = this->X_lambda[i].applyTranspose (
                    this->hc[i]);
        if (this->lambda[i] != 0) {
            unsigned int j(i);
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

unsigned int rigidbody::Joints::nbQuat() const
{
    return *m_nRotAQuat;
}

rigidbody::GeneralizedVelocity rigidbody::Joints::computeQdot(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedCoordinates &QDot,
    const double k_stab)
{
    rigidbody::GeneralizedVelocity QDotOut(Q.size());
    // Verify if there are quaternions, if not the derivate is directly QDot
    if (!m_nRotAQuat) {
        QDotOut = QDot;
        return QDotOut;
    }
    unsigned int cmpQuat(0);
    unsigned int cmpDof(0);
    for (unsigned int i=0; i<nbSegment(); ++i) {
        const rigidbody::Segment& segment_i = segment(i);
        if (segment_i.isRotationAQuaternion()) {
            // Extraire le quaternion
            utils::Quaternion quat_tp(
                Q(Q.size()-*m_nRotAQuat+cmpQuat),
                Q.block(cmpDof+segment_i.nbDofTrans(), 0, 3, 1),
                k_stab);

            // QDot for translation is actual QDot
            QDotOut.block(cmpDof, 0, segment_i.nbDofTrans(), 1)
                = QDot.block(cmpDof, 0, segment_i.nbDofTrans(), 1);

            // Get the 4d derivative for the quaternion part
            quat_tp.derivate(QDot.block(cmpDof+segment_i.nbDofTrans(), 0, 3, 1));
            QDotOut.block(cmpDof+segment_i.nbDofTrans(), 0, 3, 1) = quat_tp.block(1,0,3,1);
            QDotOut(Q.size()-*m_nRotAQuat+cmpQuat) = quat_tp(
                        0);// Placer dans le vecteur de sortie

            // Increment the number of done quaternions
            ++cmpQuat;
        } else {
            // If it's a normal, do what it usually does
            QDotOut.block(cmpDof, 0, segment_i.nbDof(), 1) = QDot.block(cmpDof, 0,
                    segment_i.nbDof(), 1);
        }
        cmpDof += segment_i.nbDof();
    }
    return QDotOut;
}

rigidbody::GeneralizedTorque rigidbody::Joints::InverseDynamics(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &QDot,
    const rigidbody::GeneralizedAcceleration &QDDot,
    std::vector<utils::SpatialVector>* f_ext,
    std::vector<utils::Vector> *f_contacts)
{
    rigidbody::GeneralizedTorque Tau(nbGeneralizedTorque());
    std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext_rbdl(combineExtForceAndSoftContact(f_ext, f_contacts, Q, QDot, true));
    RigidBodyDynamics::InverseDynamics(*this, Q, QDot, QDDot, Tau, f_ext_rbdl);
    if (f_ext_rbdl){
        delete f_ext_rbdl;
    }
    return Tau;
}

rigidbody::GeneralizedTorque rigidbody::Joints::NonLinearEffect(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &QDot,
    std::vector<utils::SpatialVector>* f_ext,
    std::vector<utils::Vector> *f_contacts)
{
    rigidbody::GeneralizedTorque Tau(*this);
    std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext_rbdl(combineExtForceAndSoftContact(f_ext, f_contacts, Q, QDot, true));
    RigidBodyDynamics::NonlinearEffects(*this, Q, QDot, Tau, f_ext_rbdl);
    if (f_ext_rbdl){
        delete f_ext_rbdl;
    }
    return Tau;
}

rigidbody::GeneralizedAcceleration
rigidbody::Joints::ForwardDynamics(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &QDot,
    const rigidbody::GeneralizedTorque &Tau,
    std::vector<utils::SpatialVector>* f_ext)
{

    bool updateKin = true;

    rigidbody::GeneralizedAcceleration QDDot(*this);
    std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext_rbdl(combineExtForceAndSoftContact(f_ext, nullptr, Q, QDot, updateKin));
    RigidBodyDynamics::ForwardDynamics(*this, Q, QDot, Tau, QDDot, f_ext_rbdl);
    if (f_ext_rbdl){
        delete f_ext_rbdl;
    }
    return QDDot;
}

rigidbody::GeneralizedAcceleration
rigidbody::Joints::ForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &QDot,
    const rigidbody::GeneralizedTorque &Tau,
    rigidbody::Contacts &CS,
    std::vector<utils::SpatialVector> *f_ext)
{
#ifdef BIORBD_USE_CASADI_MATH
    bool updateKin = true;
#else
    UpdateKinematicsCustom(&Q, &QDot);
    bool updateKin = false;  // Put this in parameters??
#endif

    rigidbody::GeneralizedAcceleration QDDot(*this);
    std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext_rbdl(combineExtForceAndSoftContact(f_ext, nullptr, Q, QDot, updateKin));
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(*this, Q, QDot, Tau, CS, QDDot, updateKin, f_ext_rbdl);
    if (f_ext_rbdl){
        delete f_ext_rbdl;
    }
    return QDDot;
}

utils::Vector
rigidbody::Joints::ContactForcesFromForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &QDot,
    const rigidbody::GeneralizedTorque &Tau,
    std::vector<utils::SpatialVector> *f_ext)
{
    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*>
                                     (this)->getConstraints();
    this->ForwardDynamicsConstraintsDirect(Q, QDot, Tau, CS, f_ext);
    return CS.getForce();
}

rigidbody::GeneralizedAcceleration
rigidbody::Joints::ForwardDynamicsConstraintsDirect(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &QDot,
    const rigidbody::GeneralizedTorque &Tau,
    std::vector<utils::SpatialVector> *f_ext)
{

    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*>(this)->getConstraints();
    return this->ForwardDynamicsConstraintsDirect(Q, QDot, Tau, CS, f_ext);
}

rigidbody::GeneralizedVelocity
rigidbody::Joints::ComputeConstraintImpulsesDirect(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDotPre
)
{
    rigidbody::Contacts CS = dynamic_cast<rigidbody::Contacts*>
                                     (this)->getConstraints();
    if (CS.nbContacts() == 0) {
        return QDotPre;
    } else {
        CS = dynamic_cast<rigidbody::Contacts*>(this)->getConstraints();

        rigidbody::GeneralizedVelocity QDotPost(*this);
        RigidBodyDynamics::ComputeConstraintImpulsesDirect(*this, Q, QDotPre, CS, QDotPost);
        return QDotPost;
    }
}

utils::Matrix3d
rigidbody::Joints::bodyInertia (
        const rigidbody::GeneralizedCoordinates &q,
        bool updateKin)
{
  if (updateKin) {
    this->UpdateKinematicsCustom (&q);
  }

  for (size_t i = 1; i < this->mBodies.size(); i++) {
    this->Ic[i] = this->I[i];
  }

  RigidBodyDynamics::Math::SpatialRigidBodyInertia Itot;

  for (size_t i = this->mBodies.size() - 1; i > 0; i--) {
    unsigned int lambda = this->lambda[i];

    if (lambda != 0) {
      this->Ic[lambda] = this->Ic[lambda] + this->X_lambda[i].applyTranspose (
                           this->Ic[i]);
    } else {
      Itot = Itot + this->X_lambda[i].applyTranspose (this->Ic[i]);
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
    RigidBodyDynamics::Math::Vector3d com, angularMomentum;
    utils::Scalar mass;
    // utils::Matrix3d body_inertia;

    // Calculate the angular velocity of the model around it's center of
    // mass from the angular momentum and the body inertia
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    RigidBodyDynamics::Utils::CalcCenterOfMass(
        *this, Q, Qdot, nullptr, mass, com, nullptr, nullptr,
        &angularMomentum, nullptr, updateKin);
    utils::Matrix3d body_inertia = bodyInertia (Q, updateKin);
        
#ifdef BIORBD_USE_CASADI_MATH
    auto linsol = casadi::Linsol("linear_solver", "symbolicqr", body_inertia.sparsity());
    RigidBodyDynamics::Math::Vector3d out = linsol.solve(body_inertia, angularMomentum);
#else
    RigidBodyDynamics::Math::Vector3d out = body_inertia.colPivHouseholderQr().solve(angularMomentum);
#endif

    return out;
}

unsigned int rigidbody::Joints::getDofIndex(
    const utils::String& segmentName,
    const utils::String& dofName)
{
    unsigned int idx = 0;

    unsigned int iB = 0;
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

void rigidbody::Joints::UpdateKinematicsCustom(
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot,
    const rigidbody::GeneralizedAcceleration *Qddot)
{
    checkGeneralizedDimensions(Q, Qdot, Qddot);
    RigidBodyDynamics::UpdateKinematicsCustom(*this, Q, Qdot, Qddot);
}

void rigidbody::Joints::CalcMatRotJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int segmentIdx,
    const utils::Matrix3d &rotation,
    RigidBodyDynamics::Math::MatrixNd &G,
    bool updateKin)
{
#ifdef RBDL_ENABLE_LOGGING
    LOG << "-------- " << __func__ << " --------" << std::endl;
#endif

#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    // update the Kinematics if necessary
    if (updateKin) {
        UpdateKinematicsCustom (&Q, nullptr, nullptr);
    }

    assert (G.rows() == 9 && G.cols() == this->qdot_size );

    std::vector<utils::Vector3d> axes;
    axes.push_back(utils::Vector3d(1,0,0));
    axes.push_back(utils::Vector3d(0,1,0));
    axes.push_back(utils::Vector3d(0,0,1));
    for (unsigned int iAxes=0; iAxes<3; ++iAxes) {
        utils::Matrix3d bodyMatRot (
            RigidBodyDynamics::CalcBodyWorldOrientation (*this, Q, segmentIdx,
                    false).transpose());
        RigidBodyDynamics::Math::SpatialTransform point_trans(
            RigidBodyDynamics::Math::SpatialTransform (
                utils::Matrix3d::Identity(),
                bodyMatRot * rotation * *(axes.begin()+iAxes)));


        unsigned int reference_body_id = segmentIdx;

        if (this->IsFixedBodyId(segmentIdx)) {
            unsigned int fbody_id = segmentIdx - this->fixed_body_discriminator;
            reference_body_id = this->mFixedBodies[fbody_id].mMovableParent;
        }

        unsigned int j = reference_body_id;

        // e[j] is set to 1 if joint j contributes to the jacobian that we are
        // computing. For all other joints the column will be zero.
        while (j != 0) {
            unsigned int q_index = this->mJoints[j].q_index;
            // If it's not a DoF in translation (3 4 5 in this->S)
#ifdef BIORBD_USE_CASADI_MATH
            if (this->S[j].is_zero() && this->S[j](4).is_zero() && this->S[j](5).is_zero())
#else
            if (this->S[j](3)!=1.0 && this->S[j](4)!=1.0 && this->S[j](5)!=1.0)
#endif
            {
                RigidBodyDynamics::Math::SpatialTransform X_base = this->X_base[j];
                X_base.r = utils::Vector3d(0,0,
                                                   0); // Remove all concept of translation (only keep the rotation matrix)

                if (this->mJoints[j].mDoFCount == 3) {
                    G.block(iAxes*3, q_index, 3,
                            3) = ((point_trans * X_base.inverse()).toMatrix() * this->multdof3_S[j]).block(
                                     3,0,3,3);
                } else {
                    G.block(iAxes*3,q_index, 3,
                            1) = point_trans.apply(X_base.inverse().apply(this->S[j])).block(3,0,3,1);
                }
            }
            j = this->lambda[j]; // Pass to parent segment
        }
    }
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
            "Wrong size for the Generalized Coordiates");
    }
    if (Qdot) {
        utils::Error::check(
            Qdot->size() == nbQdot(),
            "Wrong size for the Generalized Velocities");
    }
    if (Qddot) {
        utils::Error::check(
            Qddot->size() == nbQddot(),
            "Wrong size for the Generalized Accelerations");
    }

    if (torque) {
        utils::Error::check(
            torque->size() == nbGeneralizedTorque(),
            "Wrong size for the Generalized Torques");
    }
#endif
}
