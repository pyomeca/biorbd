#define BIORBD_API_EXPORTS
#include "RigidBody/Joints.h"

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>
#include "Utils/String.h"
#include "Utils/Quaternion.h"
#include "Utils/Matrix.h"
#include "Utils/Error.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Integrator.h"
#include "RigidBody/Segment.h"
#include "RigidBody/Markers.h"
#include "RigidBody/NodeBone.h"
#include "RigidBody/Patch.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"

biorbd::rigidbody::Joints::Joints() :
    RigidBodyDynamics::Model(),
    m_segments(std::make_shared<std::vector<biorbd::rigidbody::Segment>>()),
    m_nbRoot(std::make_shared<unsigned int>(0)),
    m_nbDof(std::make_shared<unsigned int>(0)),
    m_nbQ(std::make_shared<unsigned int>(0)),
    m_nbQdot(std::make_shared<unsigned int>(0)),
    m_nbQddot(std::make_shared<unsigned int>(0)),
    m_nRotAQuat(std::make_shared<unsigned int>(0)),
    m_isRootActuated(std::make_shared<bool>(true)),
    m_hasExternalForces(std::make_shared<bool>(false)),
    m_isKinematicsComputed(std::make_shared<bool>(false)),
    m_totalMass(std::make_shared<double>(0))
{
    m_integrator = std::make_shared<biorbd::rigidbody::Integrator>(*this);
    this->gravity = RigidBodyDynamics::Math::Vector3d (0, 0, -9.81);  // Redéfinition de la gravité pour qu'elle soit en z
}

biorbd::rigidbody::Joints::Joints(const biorbd::rigidbody::Joints &other) :
    RigidBodyDynamics::Model(other),
    m_segments(other.m_segments),
    m_integrator(other.m_integrator),
    m_nbRoot(other.m_nbRoot),
    m_nbDof(other.m_nbDof),
    m_nbQ(other.m_nbQ),
    m_nbQdot(other.m_nbQdot),
    m_nbQddot(other.m_nbQddot),
    m_nRotAQuat(other.m_nRotAQuat),
    m_isRootActuated(other.m_isRootActuated),
    m_hasExternalForces(other.m_hasExternalForces),
    m_isKinematicsComputed(other.m_isKinematicsComputed),
    m_totalMass(other.m_totalMass)
{

}

biorbd::rigidbody::Joints::~Joints()
{

}

biorbd::rigidbody::Joints biorbd::rigidbody::Joints::DeepCopy() const
{
    biorbd::rigidbody::Joints copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Joints::DeepCopy(const biorbd::rigidbody::Joints &other)
{
    static_cast<RigidBodyDynamics::Model&>(*this) = other;
    m_segments->resize(other.m_segments->size());
    for (unsigned int i=0; i<other.m_segments->size(); ++i)
        (*m_segments)[i] = (*other.m_segments)[i].DeepCopy();
    *m_integrator = other.m_integrator->DeepCopy();
    *m_nbRoot = *other.m_nbRoot;
    *m_nbDof = *other.m_nbDof;
    *m_nbQ = *other.m_nbQ;
    *m_nbQdot = *other.m_nbQdot;
    *m_nbQddot = *other.m_nbQddot;
    *m_nRotAQuat = *other.m_nRotAQuat;
    *m_isRootActuated = *other.m_isRootActuated;
    *m_hasExternalForces = *other.m_hasExternalForces;
    *m_isKinematicsComputed = *other.m_isKinematicsComputed;
    *m_totalMass = *other.m_totalMass;
}

unsigned int biorbd::rigidbody::Joints::nbGeneralizedTorque() const {
    return dof_count-nbRoot();
}
unsigned int biorbd::rigidbody::Joints::nbDof() const {
    return *m_nbDof;
}

std::vector<std::string> biorbd::rigidbody::Joints::nameDof() const
{
    std::vector<std::string> names;
    for (unsigned int i=0; i<nbSegment(); ++i){
        for (unsigned int j=0; j<Segment(i).nbDof(); ++j){
            names.push_back(Segment(i).name() + "_" + Segment(i).nameDof(j));
        }
    }
    return names;
}
unsigned int biorbd::rigidbody::Joints::nbQ() const {
    return *m_nbQ;
}
unsigned int biorbd::rigidbody::Joints::nbQdot() const {
    return *m_nbQdot;
}
unsigned int biorbd::rigidbody::Joints::nbQddot() const {
    return *m_nbQddot;
}
unsigned int biorbd::rigidbody::Joints::nbRoot() const {
    if (*m_isRootActuated)
        return 0;
    else
        return *m_nbRoot;
}
void biorbd::rigidbody::Joints::setIsRootActuated(bool a) {
    *m_isRootActuated = a;
}
bool biorbd::rigidbody::Joints::isRootActuated() const {
    return *m_isRootActuated;
}
void biorbd::rigidbody::Joints::setHasExternalForces(bool f) {
    *m_hasExternalForces = f;
}
bool biorbd::rigidbody::Joints::hasExternalForces() const {
    return *m_hasExternalForces;
}
double biorbd::rigidbody::Joints::mass() const {
    return *m_totalMass;
}




void biorbd::rigidbody::Joints::integrateKinematics(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedCoordinates& QDot,
        const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorque,
        double t0,
        double tend,
        double timeStep)
{
    biorbd::utils::Vector v(static_cast<unsigned int>(Q.rows()+QDot.rows()));
    v << Q,QDot;
    m_integrator->integrate(v, GeneralizedTorque, t0, tend, timeStep); // vecteur, t0, tend, pas, effecteurs
    *m_isKinematicsComputed = true;
}
void biorbd::rigidbody::Joints::getIntegratedKinematics(
        unsigned int step,
        biorbd::rigidbody::GeneralizedCoordinates &Q,
        biorbd::rigidbody::GeneralizedCoordinates &QDot)
{
    // If the cinematic has not been updated
    biorbd::utils::Error::check(*m_isKinematicsComputed, "ComputeKinematics must be call before calling updateKinematics");

    const biorbd::utils::Vector& tp(m_integrator->getX(step));
    for (unsigned int i=0; i< static_cast<unsigned int>(tp.rows()/2); i++){
        Q(i) = tp(i);
        QDot(i) = tp(i+tp.rows()/2);
    }
}
unsigned int biorbd::rigidbody::Joints::nbInterationStep() const
{
    return m_integrator->steps();
}


unsigned int biorbd::rigidbody::Joints::AddSegment(
        const biorbd::utils::String &segmentName, // Name of the segment
        const biorbd::utils::String &parentName, // Name of the segment's parent
        const biorbd::utils::String &translationSequence, // Translation sequence
        const biorbd::utils::String &rotationSequence, // Cardan sequence to classify the DoF in rotation
        const biorbd::rigidbody::SegmentCharacteristics& characteristics, // Mass, center of mass of the segment, inertia of segment, etc. 
        const RigidBodyDynamics::Math::SpatialTransform& centreOfRotation, // Transofrmation from parent to child
        int forcePlates)// Number of the force platform attached to the segment
{ 
    biorbd::rigidbody::Segment tp(*this, segmentName, parentName, translationSequence, rotationSequence, characteristics, centreOfRotation, forcePlates);
    if (this->GetBodyId(parentName.c_str()) == std::numeric_limits<unsigned int>::max())
        *m_nbRoot += tp.nbDof(); // If the segment name is "Root", add the number of DoF of root
    *m_nbDof += tp.nbDof();
    *m_nbQ += tp.nbQ();
    *m_nbQdot += tp.nbQdot();
    *m_nbQddot += tp.nbQddot();

    if (tp.isRotationAQuaternion())
        ++*m_nRotAQuat;
		
    *m_totalMass += characteristics.mMass; // Add the segment mass to the total body mass
    m_segments->push_back(tp);
    return 0;
}
unsigned int biorbd::rigidbody::Joints::AddSegment(
        const biorbd::utils::String &segmentName, // Segment name
        const biorbd::utils::String &parentName, // Segment's parent name
        const biorbd::utils::String &seqR, // Cardan sequence to classify the DoF in rotation
        const biorbd::rigidbody::SegmentCharacteristics& characteristics, // Mass, center of mass of segment, inertia of segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation from parent to child
        int forcePlates)// Number of the force platforme attached to the segment
{ 
    biorbd::rigidbody::Segment tp(*this, segmentName, parentName, seqR, characteristics, cor, forcePlates);
    if (this->GetBodyId(parentName.c_str()) == std::numeric_limits<unsigned int>::max())
        *m_nbRoot += tp.nbDof(); //  If the name of the segment is "Root", add the number of DoF of root
    *m_nbDof += tp.nbDof();
	
    *m_totalMass += characteristics.mMass; // Add the segment mass to the total body mass
    m_segments->push_back(tp);
    return 0;
}

const biorbd::rigidbody::Segment& biorbd::rigidbody::Joints::Segment(unsigned int idxSegment) const {
    biorbd::utils::Error::check(idxSegment < m_segments->size(), "Asked for a wrong segment (out of range)");
    return (*m_segments)[idxSegment];
}

const biorbd::rigidbody::Segment &biorbd::rigidbody::Joints::Segment(const biorbd::utils::String & nameSegment) const
{
    return Segment(static_cast<unsigned int>(GetBodyBiorbdId(nameSegment.c_str())));
}

unsigned int biorbd::rigidbody::Joints::nbSegment() const
{
     return static_cast<unsigned int>(m_segments->size());
}

std::vector<RigidBodyDynamics::Math::SpatialVector> biorbd::rigidbody::Joints::dispatchedForce(
        std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>> &spatialVector,
        unsigned int frame) const
{
    // Iterator on the force table
    std::vector<RigidBodyDynamics::Math::SpatialVector> sv2; // Gather in the same table the values at the same instant of different platforms
    for (auto vec : spatialVector)
        sv2.push_back(vec[frame]);

    // Call the equivalent function that only manages on instant
    return dispatchedForce(sv2);
}

std::vector<RigidBodyDynamics::Math::SpatialVector> biorbd::rigidbody::Joints::dispatchedForce(
        std::vector<RigidBodyDynamics::Math::SpatialVector> &sv) const{ // a spatialVector per platform
    // Output table
    std::vector<RigidBodyDynamics::Math::SpatialVector> sv_out;

    // Null Spatial vector nul to fill the final table
    RigidBodyDynamics::Math::SpatialVector sv_zero(0,0,0,0,0,0);
    sv_out.push_back(sv_zero); // The first one is associated with the universe

    // Dispatch the forces
    for (auto segment : *m_segments){
        unsigned int nbDof = segment.nbDof();
        if (nbDof != 0){ // Do not add anything if the nbDoF is zero
            // For each segment
            for (unsigned int i=0; i<nbDof-1; ++i) // Put a sv_zero on each DoF except the last one
                sv_out.push_back(sv_zero);
            if (segment.platformIdx() >= 0){ // If the solid is in contact with the platform (!= -1)
                sv_out.push_back(sv[static_cast<unsigned int>(segment.platformIdx())]); // Put the force of the corresponding platform
            }
            else
                sv_out.push_back(sv_zero); // Otherwise, put zero
        }
    }

    // Return the STL vector of SpatialVector
    return sv_out;
}

int biorbd::rigidbody::Joints::GetBodyBiorbdId(const biorbd::utils::String &segmentName) const{
    for (int i=0; i<static_cast<int>(m_segments->size()); ++i)
        if (!(*m_segments)[static_cast<unsigned int>(i)].name().compare(segmentName))
            return i;
    return -1;
}

std::vector<biorbd::utils::RotoTrans> biorbd::rigidbody::Joints::allGlobalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return allGlobalJCS();
}

std::vector<biorbd::utils::RotoTrans> biorbd::rigidbody::Joints::allGlobalJCS() const
{
    std::vector<biorbd::utils::RotoTrans> out;
    for (unsigned int i=0; i<m_segments->size(); ++i)
        out.push_back(globalJCS(i));
    return out;
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::utils::String &name)
{
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return globalJCS(name);
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx)
{
    // update the Kinematics if necessary
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return globalJCS(idx);
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(const biorbd::utils::String &parentName) const
{
    return globalJCS(static_cast<unsigned int>(GetBodyBiorbdId(parentName)));
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(
        unsigned int idx) const
{
    return CalcBodyWorldTransformation((*m_segments)[idx].id());
}

std::vector<biorbd::utils::RotoTrans> biorbd::rigidbody::Joints::localJCS() const{
    std::vector<biorbd::utils::RotoTrans> out;

    for (unsigned int i=0; i<m_segments->size(); ++i)
            out.push_back(localJCS(i));

    return out;
}
biorbd::utils::RotoTrans biorbd::rigidbody::Joints::localJCS(const biorbd::utils::String &segmentName) const{
    return localJCS(static_cast<unsigned int>(GetBodyBiorbdId(segmentName.c_str())));
}
biorbd::utils::RotoTrans biorbd::rigidbody::Joints::localJCS(const unsigned int i) const{
    return (*m_segments)[i].localJCS();
}


std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Joints::projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const std::vector<biorbd::rigidbody::NodeBone> &v,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom(&Q, nullptr, nullptr);
    updateKin = false;

    // Assuming that this is also a marker type (via BiorbdModel)
    const biorbd::rigidbody::Markers &marks = dynamic_cast<biorbd::rigidbody::Markers &>(*this);

    // Safety
    biorbd::utils::Error::check(marks.nbMarkers() == v.size(), "Number of marker must be equal to number of Vector3d");

    std::vector<biorbd::rigidbody::NodeBone> out;
    for (unsigned int i=0;i<marks.nbMarkers();++i){
        biorbd::rigidbody::NodeBone tp(marks.marker(i));
        if (tp.nbAxesToRemove()!=0){
            tp = v[i].applyRT(globalJCS(tp.parent()).transpose());
            // Take the position of the new marker with the info from the model
            out.push_back(projectPoint(Q,tp,updateKin));
        }
        else
            // If we don't have to remove anything (return the same position immediately)
            out.push_back( v[i] );
    }
    return out;
}

biorbd::rigidbody::NodeBone biorbd::rigidbody::Joints::projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const utils::Vector3d &v,
        int segmentIdx,
        const biorbd::utils::String& axesToRemove,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    // Create a marker
    const biorbd::utils::String& segmentName(segment(static_cast<unsigned int>(segmentIdx)).name());
    biorbd::rigidbody::NodeBone node( v.applyRT(globalJCS(static_cast<unsigned int>(segmentIdx)).transpose()), "tp", segmentName,
                     true, true, axesToRemove, static_cast<int>(GetBodyId(segmentName.c_str())));

    // Project and then reset in global
    return projectPoint(Q, node, updateKin);
}

biorbd::rigidbody::NodeBone biorbd::rigidbody::Joints::projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::NodeBone &n,
        bool updateKin)
{
    // Assuming that this is also a Marker type (via BiorbdModel)
    return dynamic_cast<biorbd::rigidbody::Markers &>(*this).marker(Q, n, true, updateKin);
}

biorbd::utils::Matrix biorbd::rigidbody::Joints::projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        biorbd::rigidbody::NodeBone node,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    // Assuming that this is also a Marker type (via BiorbdModel)
    biorbd::rigidbody::Markers &marks = dynamic_cast<biorbd::rigidbody::Markers &>(*this);

    // If the point has not been projected, there is no effect
    if (node.nbAxesToRemove() != 0){
        // Jacobian of the marker
        node.applyRT(globalJCS(node.parent()).transpose());
        biorbd::utils::Matrix G_tp(marks.markersJacobian(Q, node.parent(), biorbd::utils::Vector3d(0,0,0), updateKin));
        biorbd::utils::Matrix JCor(biorbd::utils::Matrix::Zero(9,nbQ()));
        CalcMatRotJacobian(Q, GetBodyId(node.parent().c_str()), Eigen::Matrix3d::Identity(3,3), JCor,false);
        for (unsigned int n=0; n<3; ++n)
            if (node.isAxisKept(n))
                G_tp += JCor.block(n*3,0,3,nbQ()) * node(n);

        return G_tp;
    }
    else {
        // Return the value
        return biorbd::utils::Matrix::Zero(3,nbQ());
    }
}

biorbd::utils::Matrix biorbd::rigidbody::Joints::projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const utils::Vector3d &v,
        int segmentIdx,
        const biorbd::utils::String& axesToRemove,
        bool updateKin)
{
    // Find the point
    const biorbd::rigidbody::NodeBone& p(projectPoint(Q, v, segmentIdx, axesToRemove, updateKin));

    // Return the value
    return projectPointJacobian(Q, p, updateKin);
}

std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Joints::projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const std::vector<biorbd::rigidbody::NodeBone> &v,
        bool updateKin)
{
    // Gather the points
    const std::vector<biorbd::rigidbody::NodeBone>& tp(projectPoint(Q, v, updateKin));

    // Calculate the Jacobian if the point is not projected
    std::vector<biorbd::utils::Matrix> G;

    for (unsigned int i=0; i<tp.size(); ++i){
        // Actual marker 
        G.push_back(projectPointJacobian(Q, biorbd::rigidbody::NodeBone(v[i]), false));
    }
    return G;
}

RigidBodyDynamics::Math::SpatialTransform biorbd::rigidbody::Joints::CalcBodyWorldTransformation (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const unsigned int body_id,
        bool update_kinematics = true)
{
    // update the Kinematics if necessary
    if (update_kinematics)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return CalcBodyWorldTransformation(body_id);
}

RigidBodyDynamics::Math::SpatialTransform biorbd::rigidbody::Joints::CalcBodyWorldTransformation(
        const unsigned int body_id) const
{

    if (body_id >= this->fixed_body_discriminator) {
        unsigned int fbody_id = body_id - this->fixed_body_discriminator;
        unsigned int parent_id = this->mFixedBodies[fbody_id].mMovableParent;
        biorbd::utils::RotoTrans parentRT(this->X_base[parent_id].E.transpose(), this->X_base[parent_id].r);
        biorbd::utils::RotoTrans bodyRT(this->mFixedBodies[fbody_id].mParentTransform.E.transpose(), this->mFixedBodies[fbody_id].mParentTransform.r);
        const biorbd::utils::RotoTrans& transfo_tp = parentRT * bodyRT;
        return RigidBodyDynamics::Math::SpatialTransform (transfo_tp.rot(), transfo_tp.trans());
    }

    return RigidBodyDynamics::Math::SpatialTransform (this->X_base[body_id].E.transpose(), this->X_base[body_id].r);
}

biorbd::utils::Vector3d biorbd::rigidbody::Joints::CoM(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    // Return the position of the center of mass from the generalized coordinates

    // Ensure that the model is in the right configuration
    if (updateKin)
        UpdateKinematicsCustom(&Q, nullptr, nullptr);

    // For each segment, find the CoM (CoM = sum(segment_mass * pos_com_seg) / total mass)
    const std::vector<biorbd::rigidbody::NodeBone>& com_segment(CoMbySegment(Q,true));
    biorbd::utils::Vector3d com(0, 0, 0);
    for (unsigned int i=0; i<com_segment.size(); ++i)
        com += (*m_segments)[i].characteristics().mMass * com_segment[i];

    // Divide by total mass
    com = com/this->mass();

    // Return the CoM
    return com;
}

biorbd::utils::Vector3d biorbd::rigidbody::Joints::angularMomentum(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool updateKin)
{
    return CalcAngularMomentum(Q, Qdot, updateKin);
}


biorbd::utils::Vector3d biorbd::rigidbody::Joints::CoMdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot)
{
    // Return the velocity of the center of mass from the generalized coordinates

    // Ensure the model is in the right configuration
    UpdateKinematicsCustom(&Q, &Qdot, nullptr);

    // For each segment, find the CoM
    biorbd::utils::Vector3d com_dot(0,0,0);

    // CoMdot = sum(mass_seg * Jacobian * qdot)/mass totale
    biorbd::utils::Matrix Jac(biorbd::utils::Matrix(3,this->dof_count));
    for (auto segment : *m_segments){
        Jac.setZero();
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId(segment.name().c_str()), segment.characteristics().mCenterOfMass, Jac, false); // False for speed
        com_dot += ((Jac*Qdot) * segment.characteristics().mMass);
    }
    // Divide by total mass
    com_dot = com_dot/this->mass();

    // Return the velocity of CoM
    return com_dot;
}
biorbd::utils::Vector3d biorbd::rigidbody::Joints::CoMddot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot)
{
    double mass;
    RigidBodyDynamics::Math::Vector3d com, com_ddot;
    RigidBodyDynamics::Utils::CalcCenterOfMass(
                *this, Q, Qdot, &Qddot, mass, com, nullptr, &com_ddot,
                nullptr, nullptr, true);


    // Return the acceleration of CoM
    return com_ddot;
}

biorbd::utils::Matrix biorbd::rigidbody::Joints::CoMJacobian(const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    // Return the position of the center of mass from the generalized coordinates

    // Ensure that the model is in the right configuration
    UpdateKinematicsCustom(&Q, nullptr, nullptr);

   // Total jacobian
    biorbd::utils::Matrix JacTotal(biorbd::utils::Matrix::Zero(3,this->dof_count));

    // CoMdot = sum(mass_seg * Jacobian * qdot)/mass total
    biorbd::utils::Matrix Jac(biorbd::utils::Matrix::Zero(3,this->dof_count));
    for (auto segment : *m_segments){
        Jac.setZero();
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId(segment.name().c_str()), segment.characteristics().mCenterOfMass, Jac, false); // False for speed
        JacTotal += segment.characteristics().mMass*Jac;
    }

    // Divide by total mass
    JacTotal /= this->mass();

    // Return the Jacobian of CoM
    return JacTotal;
}


std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Joints::CoMbySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{// Position of the center of mass for each segment
    std::vector<biorbd::rigidbody::NodeBone> tp; // vector of the output vectors

    for (unsigned int i=0; i<m_segments->size(); ++i){
        tp.push_back(CoMbySegment(Q,i,updateKin));
        updateKin = false; // only do it the first time
    }
    return tp;
}


biorbd::utils::Vector3d biorbd::rigidbody::Joints::CoMbySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const unsigned int i,
        bool updateKin)
{ // Position of the center of mass of segment i
    biorbd::utils::Error::check(i < m_segments->size(), "Choosen segment doesn't exist");
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
                *this, Q, (*m_segments)[i].id(), (*m_segments)[i].characteristics().mCenterOfMass, updateKin);
}


std::vector<biorbd::utils::Vector3d> biorbd::rigidbody::Joints::CoMdotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool updateKin)
{// Position of the center of mass of each segment
    std::vector<biorbd::utils::Vector3d> tp; // vector of the output vectors

    for (unsigned int i=0; i<m_segments->size(); ++i){
        tp.push_back(CoMdotBySegment(Q,Qdot,i,updateKin));
        updateKin = false; // only do it for the first time
    }
    return tp;
}


biorbd::utils::Vector3d biorbd::rigidbody::Joints::CoMdotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const unsigned int i,
        bool updateKin)
{ // Position of the center of mass of segment i
    biorbd::utils::Error::check(i < m_segments->size(), "Choosen segment doesn't exist");
    return CalcPointVelocity(*this, Q, Qdot, (*m_segments)[i].id(),(*m_segments)[i].characteristics().mCenterOfMass,updateKin);
}


std::vector<biorbd::utils::Vector3d> biorbd::rigidbody::Joints::CoMddotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        bool updateKin)
{// Position of the center of mass of each segment
    std::vector<biorbd::utils::Vector3d> tp; // vector of output vectors

    for (unsigned int i=0; i<m_segments->size(); ++i){
        tp.push_back(CoMddotBySegment(Q,Qdot,Qddot,i,updateKin));
        updateKin = false; // only do it the first time
    }
    return tp;
}


biorbd::utils::Vector3d biorbd::rigidbody::Joints::CoMddotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        const unsigned int i,
        bool updateKin)
{ // Position of the center of mass of segment i
    biorbd::utils::Error::check(i < m_segments->size(), "Choosen segment doesn't exist");
    return RigidBodyDynamics::CalcPointAcceleration(*this, Q, Qdot, Qddot, (*m_segments)[i].id(),(*m_segments)[i].characteristics().mCenterOfMass,updateKin);
}

std::vector<std::vector<biorbd::utils::Vector3d>> biorbd::rigidbody::Joints::meshPoints(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    std::vector<std::vector<biorbd::utils::Vector3d>> v; // Vector of output vectors (mesh per segment)

    // Find the position of the segments
    const std::vector<biorbd::utils::RotoTrans>& RT(allGlobalJCS());

    // For all the segments
    for (unsigned int i=0; i<nbSegment(); ++i)
        v.push_back(meshPoints(RT,i));

    return v;
}
std::vector<biorbd::utils::Vector3d> biorbd::rigidbody::Joints::meshPoints(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int i,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    // Find the position of the segments
    const std::vector<biorbd::utils::RotoTrans>& RT(allGlobalJCS());

    return meshPoints(RT,i);
}

std::vector<biorbd::utils::Matrix>
biorbd::rigidbody::Joints::meshPointsInMatrix(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);
    const std::vector<biorbd::utils::RotoTrans>& RT(allGlobalJCS());

    std::vector<biorbd::utils::Matrix> all_points;
    for (unsigned int i=0; i<m_segments->size(); ++i) {
        biorbd::utils::Matrix mat(3, Mesh(i).size());
        for (unsigned int j=0; j<Mesh(i).size(); ++j){
            biorbd::utils::Vector3d tp (Mesh(i).point(j));
            tp.applyRT(RT[i]);
            mat.block(0, j, 3, 1) = tp;
        }
        all_points.push_back(mat);
    }
    return all_points;
}
std::vector<biorbd::utils::Vector3d> biorbd::rigidbody::Joints::meshPoints(
        const std::vector<biorbd::utils::RotoTrans> &RT,
        unsigned int i) const{

    // Gather the position of the meshings
    std::vector<biorbd::utils::Vector3d> v;
    for (unsigned int j=0; j<Mesh(i).size(); ++j){
        biorbd::utils::Vector3d tp (Mesh(i).point(j));
        tp.applyRT(RT[i]);
        v.push_back(tp);
    }

    return v;
}
std::vector<std::vector<biorbd::rigidbody::Patch>> biorbd::rigidbody::Joints::meshPatch() const{
    // Gather the position of the meshings for all the segments
    std::vector<std::vector<biorbd::rigidbody::Patch>> v_all;
    for (unsigned int j=0; j<nbSegment(); ++j)
        v_all.push_back(meshPatch(j));
    return v_all;
}
const std::vector<biorbd::rigidbody::Patch> &biorbd::rigidbody::Joints::meshPatch(unsigned int i) const{
    // Find the position of the meshings for a segment i
    return Mesh(i).patch();
}

std::vector<biorbd::rigidbody::Mesh> biorbd::rigidbody::Joints::Mesh() const
{
    std::vector<biorbd::rigidbody::Mesh> segmentOut;
    for (unsigned int i=0; i<nbSegment(); ++i)
        segmentOut.push_back(Mesh(i));
    return segmentOut;
}

const biorbd::rigidbody::Mesh &biorbd::rigidbody::Joints::Mesh(unsigned int idx) const
{
    return segment(idx).characteristics().mesh();
}

biorbd::utils::Vector3d biorbd::rigidbody::Joints::CalcAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool update_kinematics)
{
    RigidBodyDynamics::Math::Vector3d com,  angular_momentum;
    double mass;
    
    // Calculate the angular momentum with the function of the position of the center of mass
    RigidBodyDynamics::Utils::CalcCenterOfMass(
                *this, Q, Qdot, nullptr, mass, com, nullptr, nullptr,
                &angular_momentum, nullptr, update_kinematics);
    return angular_momentum;
}
biorbd::utils::Vector3d biorbd::rigidbody::Joints::CalcAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        bool update_kinematics)
{
    // Definition of the variables
    RigidBodyDynamics::Math::Vector3d com,  angular_momentum;
    double mass;

    // Calculate the angular momentum with the function of the position of the center of mass
    RigidBodyDynamics::Utils::CalcCenterOfMass(
                *this, Q, Qdot, &Qddot, mass, com, nullptr, nullptr,
                &angular_momentum, nullptr, update_kinematics);

    return angular_momentum;
}

std::vector<biorbd::utils::Vector3d> biorbd::rigidbody::Joints::CalcSegmentsAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool update_kinematics) {
    if (update_kinematics)
        UpdateKinematicsCustom (&Q, &Qdot, nullptr);
        
    double mass;
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Utils::CalcCenterOfMass (*this, Q, Qdot, nullptr, mass, com, nullptr, nullptr, nullptr, nullptr, false);
    RigidBodyDynamics::Math::SpatialTransform X_to_COM (RigidBodyDynamics::Math::Xtrans(com));

    std::vector<biorbd::utils::Vector3d> h_segment;
    for (unsigned int i = 1; i < this->mBodies.size(); i++) {
        this->Ic[i] = this->I[i];
        this->hc[i] = this->Ic[i].toMatrix() * this->v[i];

        RigidBodyDynamics::Math::SpatialVector h = this->X_lambda[i].applyTranspose (this->hc[i]);
        if (this->lambda[i] != 0) {
            unsigned int j(i);
            do {
                j = this->lambda[j];
                h = this->X_lambda[j].applyTranspose (h);
            } while (this->lambda[j]!=0);
        }
        h = X_to_COM.applyAdjoint (h);
        h_segment.push_back(biorbd::utils::Vector3d(h[0],h[1],h[2]));
    }


    return h_segment;
}

std::vector<biorbd::utils::Vector3d> biorbd::rigidbody::Joints::CalcSegmentsAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        bool update_kinematics) {
    if (update_kinematics)
        UpdateKinematicsCustom (&Q, &Qdot, &Qddot);
        
    double mass;
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Utils::CalcCenterOfMass (*this, Q, Qdot, &Qddot, mass, com, nullptr, nullptr, nullptr, nullptr, false);
    RigidBodyDynamics::Math::SpatialTransform X_to_COM (RigidBodyDynamics::Math::Xtrans(com));

    std::vector<biorbd::utils::Vector3d> h_segment;
    for (unsigned int i = 1; i < this->mBodies.size(); i++) {
        this->Ic[i] = this->I[i];
        this->hc[i] = this->Ic[i].toMatrix() * this->v[i];

        RigidBodyDynamics::Math::SpatialVector h = this->X_lambda[i].applyTranspose (this->hc[i]);
        if (this->lambda[i] != 0) {
            unsigned int j(i);
            do {
                j = this->lambda[j];
                h = this->X_lambda[j].applyTranspose (h);
            } while (this->lambda[j]!=0);
        }
        h = X_to_COM.applyAdjoint (h);
        h_segment.push_back(biorbd::utils::Vector3d(h[0],h[1],h[2]));
    }


    return h_segment;
}

void biorbd::rigidbody::Joints::ForwardDynamicsContactsLagrangian (
        const RigidBodyDynamics::Math::VectorNd &Q,
        const RigidBodyDynamics::Math::VectorNd &QDot,
        const RigidBodyDynamics::Math::VectorNd &GeneralizedTorque,
        RigidBodyDynamics::ConstraintSet &CS,
        RigidBodyDynamics::Math::VectorNd &QDDot
     ) {

   // Compute C
   CS.QDDot_0.setZero();
   RigidBodyDynamics::InverseDynamics (*this, Q, QDot, CS.QDDot_0, CS.C);

   // Compute H
   CS.H = RigidBodyDynamics::Math::MatrixNd::Zero(this->dof_count, this->dof_count);
   RigidBodyDynamics::CompositeRigidBodyAlgorithm (*this, Q, CS.H, false);

   // Compute G
   unsigned int i,j;

   // variables to check whether we need to recompute G
   unsigned int prev_body_id = 0;
   RigidBodyDynamics::Math::Vector3d prev_body_point = RigidBodyDynamics::Math::Vector3d::Zero();
   RigidBodyDynamics::Math::MatrixNd Gi (RigidBodyDynamics::Math::MatrixNd::Zero(3, this->dof_count));

   for (i = 0; i < CS.size(); i++) {
     // Only alow contact normals along the coordinate axes
//     unsigned int axis_index = 0;

     // only compute the matrix Gi if actually needed
     if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
        RigidBodyDynamics::CalcPointJacobian (*this, Q, CS.body[i], CS.point[i], Gi, false);
       prev_body_id = CS.body[i];
       prev_body_point = CS.point[i];
     }

     for (j = 0; j < this->dof_count; j++) {
       RigidBodyDynamics::Math::Vector3d gaxis (Gi(0,j), Gi(1,j), Gi(2,j));
       CS.G(i,j) = gaxis.transpose() * CS.normal[i];
 //      CS.G(i,j) = Gi(axis_index, j);
     }
  }

   // Compute gamma
   prev_body_id = 0;
   prev_body_point = RigidBodyDynamics::Math::Vector3d::Zero();
   RigidBodyDynamics::Math::Vector3d gamma_i = RigidBodyDynamics::Math::Vector3d::Zero();

   // update Kinematics just once
   UpdateKinematics (*this, Q, QDot, CS.QDDot_0);

   for (i = 0; i < CS.size(); i++) {
     // Only alow contact normals along the coordinate axes
     unsigned int axis_index = 0;

     if (CS.normal[i] == RigidBodyDynamics::Math::Vector3d(1., 0., 0.))
       axis_index = 0;
    else if (CS.normal[i] == RigidBodyDynamics::Math::Vector3d(0., 1., 0.))
       axis_index = 1;
    else if (CS.normal[i] == RigidBodyDynamics::Math::Vector3d(0., 0., 1.))
       axis_index = 2;
    else
       biorbd::utils::Error::raise("Invalid contact normal axis!");

        // only compute point accelerations when necessary
     if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
       gamma_i = CalcPointAcceleration (*this, Q, QDot, CS.QDDot_0, CS.body[i], CS.point[i], false);
       prev_body_id = CS.body[i];
       prev_body_point = CS.point[i];
     }

    // we also substract ContactData[i].acceleration such that the contact
    // point will have the desired acceleration
     CS.gamma[i] = gamma_i[axis_index] - CS.acceleration[i];
   }

   // Build the system
   CS.A.setZero();
  CS.b.setZero();
  CS.x.setZero();

   // Build the system: Copy H
  for (i = 0; i < this->dof_count; i++) {
     for (j = 0; j < this->dof_count; j++) {
       CS.A(i,j) = CS.H(i,j);
     }
  }

   // Build the system: Copy G, and G^T
   for (i = 0; i < CS.size(); i++) {
     for (j = 0; j < this->dof_count; j++) {
       CS.A(i + this->dof_count, j) = CS.G (i,j);
       CS.A(j, i + this->dof_count) = CS.G (i,j);
     }
   }

   // Build the system: Copy -C + \GeneralizedTorque
   for (i = 0; i < this->dof_count; i++) {
    CS.b[i] = -CS.C[i] + GeneralizedTorque[i];
   }

   // Build the system: Copy -gamma
   for (i = 0; i < CS.size(); i++) {
     CS.b[i + this->dof_count] = - CS.gamma[i];
   }

//   std::cout << "A = " << std::endl << CS.A << std::endl;
   //std::cout << "b = " << std::endl << CS.b << std::endl;

  switch (CS.linear_solver) {
    case (RigidBodyDynamics::Math::LinearSolverPartialPivLU) :
     CS.x = CS.A.partialPivLu().solve(CS.b);
      break;
  case (RigidBodyDynamics::Math::LinearSolverColPivHouseholderQR) :
       CS.x = CS.A.colPivHouseholderQr().solve(CS.b);
       break;
     default:
#ifdef RBDL_ENABLE_LOGGING
       LOG << "Error: Invalid linear solver: " << CS.linear_solver << std::endl;
#endif
       biorbd::utils::Error::raise("Error: Invalid linear solver");
     break;
   }

   //std::cout << "x = " << std::endl << CS.x << std::endl;

   // Copy back QDDot
for (i = 0; i < this->dof_count; i++)
     QDDot[i] = CS.x[i];

  // Copy back contact forces
  for (i = 0; i < CS.size(); i++) {
     CS.force[i] = -CS.x[this->dof_count + i];
   }
 }

unsigned int biorbd::rigidbody::Joints::nbQuat() const{
    return *m_nRotAQuat;
}

void biorbd::rigidbody::Joints::computeQdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &QDot,
        biorbd::rigidbody::GeneralizedCoordinates &QDotOut)
{
    // Verify if there are quaternions, if not the derivate is directly QDot
    if (!m_nRotAQuat){
        QDotOut = QDot;
        return;
    }

    QDotOut.resize(Q.size()); // Create an empty vector of the final dimension
    unsigned int cmpQuat(0);
    unsigned int cmpDof(0);
    for (unsigned int i=0; i<nbSegment(); ++i){
        biorbd::rigidbody::Segment segment_i= segment(i);
        if (segment_i.isRotationAQuaternion()){
            // Extract the quaternion
            biorbd::utils::Quaternion quat_tp(Q.block(cmpDof+ segment_i.nbDofTrans(), 0, 3, 1), Q(Q.size()-*m_nRotAQuat+cmpQuat));

            // Place the output vector
            QDotOut.block(cmpDof, 0, segment_i.nbDofTrans(), 1) = QDot.block(cmpDof, 0, segment_i.nbDofTrans(), 1); // La dérivée des translations est celle directement de qdot

            // Derive
            quat_tp.derivate(QDot.block(cmpDof+ segment_i.nbDofTrans(), 0, 3, 1));
            QDotOut.block(cmpDof+segment_i.nbDofTrans(), 0, 3, 1) = quat_tp.block(0,0,3,1);
            QDotOut(Q.size()-*m_nRotAQuat+cmpQuat) = quat_tp(3);// Placer dans le vecteur de sortie

           // Increment the number of done quaternions
            ++cmpQuat;
        }
        else{
            // If it's a normal, do what it usually does
            QDotOut.block(cmpDof, 0, segment_i.nbDof(), 1) = QDot.block(cmpDof, 0, segment_i.nbDof(), 1);
        }
        cmpDof += segment_i.nbDof();
    }

}



unsigned int biorbd::rigidbody::Joints::getDofIndex(
        const biorbd::utils::String& segmentName,
        const biorbd::utils::String& dofName)
{
    unsigned int idx = 0;

    unsigned int iB = 0;
    bool found = false;
    while (1){
        biorbd::utils::Error::check(iB!=m_segments->size(), "Segment not found");

        if (segmentName.compare(  (*m_segments)[iB].name() )   )
            idx +=  (*m_segments)[iB].nbDof();
        else{
            idx += (*m_segments)[iB].getDofIdx(dofName);
            found = true;
            break;
        }

        ++iB;
    }

    biorbd::utils::Error::check(found, "Dof not found");
    return idx;
}

void biorbd::rigidbody::Joints::UpdateKinematicsCustom(
        const biorbd::rigidbody::GeneralizedCoordinates *Q,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates *Qddot)
{
    RigidBodyDynamics::UpdateKinematicsCustom(*this, Q, Qdot, Qddot);
}

void biorbd::rigidbody::Joints::CalcMatRotJacobian(
        const RigidBodyDynamics::Math::VectorNd &Q,
        unsigned int body_id,
        const RigidBodyDynamics::Math::Matrix3d &rotation,
        RigidBodyDynamics::Math::MatrixNd &G,
        bool update_kinematics)
{
#ifdef RBDL_ENABLE_LOGGING
    LOG << "-------- " << __func__ << " --------" << std::endl;
#endif

    // update the Kinematics if necessary
    if (update_kinematics) {
        RigidBodyDynamics::UpdateKinematicsCustom (*this, &Q, nullptr, nullptr);
    }

    assert (G.rows() == 9 && G.cols() == this->qdot_size );

    std::vector<biorbd::utils::Vector3d> axes;
    axes.push_back(biorbd::utils::Vector3d(1,0,0));
    axes.push_back(biorbd::utils::Vector3d(0,1,0));
    axes.push_back(biorbd::utils::Vector3d(0,0,1));
    for (unsigned int iAxes=0; iAxes<3; ++iAxes){
        RigidBodyDynamics::Math::Matrix3d bodyMatRot (
                    RigidBodyDynamics::CalcBodyWorldOrientation (*this, Q, body_id, false).transpose());
        RigidBodyDynamics::Math::SpatialTransform point_trans(
                    RigidBodyDynamics::Math::SpatialTransform (
                        RigidBodyDynamics::Math::Matrix3d::Identity(), bodyMatRot * rotation * *(axes.begin()+iAxes)));


        unsigned int reference_body_id = body_id;

        if (this->IsFixedBodyId(body_id)) {
            unsigned int fbody_id = body_id - this->fixed_body_discriminator;
            reference_body_id = this->mFixedBodies[fbody_id].mMovableParent;
        }

        unsigned int j = reference_body_id;

        // e[j] is set to 1 if joint j contributes to the jacobian that we are
        // computing. For all other joints the column will be zero.
        while (j != 0) {
            unsigned int q_index = this->mJoints[j].q_index;
            // If it's not a DoF in translation (3 4 5 in this->S)
            if (this->S[j](3)!=1.0 && this->S[j](4)!=1.0 && this->S[j](5)!=1.0)
            {
                RigidBodyDynamics::Math::SpatialTransform X_base = this->X_base[j];
                X_base.r << 0,0,0; // Remove all concept of translation (only keep the rotation matrix)

                if (this->mJoints[j].mDoFCount == 3) {
                    G.block(iAxes*3, q_index, 3, 3) = ((point_trans * X_base.inverse()).toMatrix() * this->multdof3_S[j]).block(3,0,3,3);
                } else {
                    G.block(iAxes*3,q_index, 3, 1) = point_trans.apply(X_base.inverse().apply(this->S[j])).block(3,0,3,1);
                }
            }
            j = this->lambda[j]; // Pass to parent segment
        }
    }
}










