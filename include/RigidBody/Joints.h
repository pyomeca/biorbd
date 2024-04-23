#ifndef BIORBD_RIGIDBODY_JOINTS_H
#define BIORBD_RIGIDBODY_JOINTS_H

#include <memory>
#include <rbdl/Model.h>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
class RotoTrans;
class Matrix;
class Matrix3d;
class Vector;
class Vector3d;
class Range;
class SpatialVector;
class SpatialTransform;
}

namespace rigidbody
{
class ExternalForceSet;
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedTorque;
class GeneralizedAcceleration;
class NodeSegment;
class MeshFace;
class Segment;
class SegmentCharacteristics;
class Mesh;
class Contacts;

///
/// \brief This is the core of the musculoskeletal model in biorbd
///
#ifdef SWIG
class BIORBD_API Joints
#else
class BIORBD_API Joints : public RigidBodyDynamics::Model
#endif
{
public:

    ///
    /// \brief Construct a joint model
    ///
    Joints();

    ///
    /// \brief Construct a joint model from another model
    /// \param other The other joint model
    ///
    Joints(
        const Joints& other);

    ///
    /// \brief Properly destroy class
    ///
    virtual ~Joints();

    ///
    /// \brief Deep copy of the joints
    /// \return Copy of the joints
    ///
    Joints DeepCopy() const;

    ///
    /// \brief Deep copy of the joints
    /// \param other The joints to copy
    ///
    void DeepCopy(
        const Joints& other);

    ///
    /// \brief Add a segment to the model
    /// \param segmentName Name of the segment
    /// \param parentName Name of the segment parent
    /// \param translationSequence The translation sequence
    /// \param rotationSequence Euler sequence of rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QdotRanges Ranges of the translations and rotations dof velocity. The length of QdotRanges must be equal to length of translations and rotations
    /// \param QddotRanges Ranges of the translations and rotations dof acceleration. The length of QddotRanges must be equal to length of translations and rotations
    /// \param jointDampings The damping of the joints to apply to the dynamics
    /// \param characteristics The characteristics of the semgent (mass, center of mass, inertia of the segment, etc)
    /// \param referenceFrame Transformation of the parent to child
    ///
    size_t AddSegment(
        const utils::String &segmentName,
        const utils::String &parentName,
        const utils::String &translationSequence,
        const utils::String &rotationSequence,
        const std::vector<utils::Range>& QRanges,
        const std::vector<utils::Range>& QdotRanges,
        const std::vector<utils::Range>& QddotRanges,
        const std::vector<utils::Scalar>& jointDampings,
        const SegmentCharacteristics& characteristics,
        const utils::RotoTrans& referenceFrame);

    ///
    /// \brief Add a segment to the model
    /// \param segmentName Name of the segment
    /// \param parentName Name of the segment parent
    /// \param translationSequence The translation sequence
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QdotRanges Ranges of the translations and rotations dof velocity. The length of QdotRanges must be equal to length of translations and rotations
    /// \param QddotRanges Ranges of the translations and rotations dof acceleration. The length of QddotRanges must be equal to length of translations and rotations
    /// \param jointDampings The damping of the joints to apply to the dynamics
    /// \param characteristics The characteristics of the semgent (mass, center of mass, inertia of the segment, etc)
    /// \param referenceFrame Transformation of the parent to child
    ///
    size_t AddSegment(
        const utils::String &segmentName,
        const utils::String &parentName,
        const utils::String &translationSequence,
        const std::vector<utils::Range>& QRanges,
        const std::vector<utils::Range>& QdotRanges,
        const std::vector<utils::Range>& QddotRanges,
        const std::vector<utils::Scalar>& jointDampings,
        const SegmentCharacteristics& characteristics,
        const utils::RotoTrans& referenceFrame);


    // -- GENERAL MODELLING -- //
    ///
    /// \brief Get the current gravity
    /// \return The current gravity
    ///
    utils::Vector3d getGravity() const;

    ///
    /// \brief Set the gravity
    /// \param newGravity The new gravity vector
    ///
    void setGravity(
        const utils::Vector3d& newGravity);

    // -- INFORMATION ON THE MODEL -- //
    ///
    /// \brief Return the biorbd body identification
    /// \param segmentName The name of the segment
    /// \return The biorbd body identification
    ///
    int getBodyBiorbdId(
        const utils::String &segmentName) const;

    ///
    /// \brief Return the rbdl body identification
    /// \param segmentName The name of the segment
    /// \return The rbdl body identification
    ///
    int getBodyRbdlId(
        const utils::String &segmentName) const;

    ///
    /// \brief Return the Biorbd body identification from rbdl
    /// \param idx The Rbdl body Id
    /// \return The Biorbd body identification
    ///
    int getBodyRbdlIdToBiorbdId(
        const int idx) const;

    ///
    /// \brief Return the Rbdl body identification from Biorbd
    /// \param idx The Biorbd segment Id
    /// \return The Rbdl body identification
    ///
    size_t getBodyBiorbdIdToRbdlId(
        const int idx) const;

    ///
    /// \brief Return the rbdl idx of subtrees of each segments
    /// \return the rbdl idx of subtrees of each segments
    ///
    std::vector<std::vector<size_t> > getDofSubTrees();

protected:
    ///
    /// \brief Return the rbdl idx of subtrees of each segments
    /// \param subTrees the rbdl idx of subtrees of each segments to be filled
    /// \param idx starting index to explore the subtrees
    /// \return the rbdl idx of subtrees of each segments starting from the specified index
    ///
    std::vector<std::vector<size_t> > recursiveDofSubTrees(
        std::vector<std::vector<size_t> >subTrees,
        size_t idx);

public:
    ///
    /// \brief Return the number of generalized torque
    /// \return The number of generalized torque
    ///
    size_t nbGeneralizedTorque() const;

    ///
    /// \brief Return the actual number of segment
    /// \return The actual number of segment
    ///
    size_t nbSegment() const;

    ///
    /// \brief Return the number of degrees of freedom (DoF)
    /// \return The number of DoF
    ///
    size_t nbDof() const;

    ///
    /// \brief Return the index of a DoF in a segment
    /// \param SegmentName The name of the Segment
    /// \param dofName The name of the degree of freedom (DoF)
    /// \return The index of a DoF in a segment
    ///
    size_t getDofIndex(
        const utils::String& SegmentName,
        const utils::String& dofName);

    ///
    /// \brief Return the names of the degree of freedom (DoF)
    /// \return The names of the DoF
    ///
    std::vector<utils::String> nameDof() const;

    ///
    /// \brief Return the number of generalized coordinates (Q)
    /// \return The number of Q
    ///
    size_t nbQ() const;

    ///
    /// \brief Return the number of generalized velocities (Qdot)
    /// \return The number of Qdot
    ///
    size_t nbQdot() const;

    ///
    /// \brief Return the number of generalized acceleration (Qddot)
    /// \return The number of Qddot
    ///
    size_t nbQddot() const;

    ///
    /// \brief Return the dof on the root
    /// \return The dof on the root
    ///
    size_t nbRoot() const;

    ///
    /// \brief Return the number of segments that are described using quaternions
    /// \return The number number of segments
    ///
    size_t nbQuat() const;


    ///
    /// \brief updateSegmentCharacteristics Change the inertia characteristics of the segment idx
    /// \param idx The index of the segment to change
    /// \param characteristics The new characteristics
    ///
    /// Warning: This function may behave surpringly due to the core of RBDL. The
    /// new characteristic values will replace everything which is attach in a fixed manner
    /// (that is no degrees-of-freedom). So if your model has 3 segments, but only the first
    /// one has dof (and the rest is rigidly attached to the first), then it doesn't matter
    /// if idx is 0, 1 or 2, because RBDL considers that all these segment are 1 segment.
    /// It is therefore expected that characteristics is the combination of mass and
    /// inertia for these 3 segments as well.
    ///
    void updateSegmentCharacteristics(
        size_t idx,
        const SegmentCharacteristics& characteristics);


    ///
    /// \brief Get a segment of index idx
    /// \param idx Index of the segment
    /// \return The segment
    ///
    Segment& segment(
        size_t idx) const;

    ///
    /// \brief Get a segment of a specific name
    /// \param name The name of the segment to return
    /// \return The segment
    ///
    Segment& segment(
        const utils::String& name) const;

    ///
    /// \brief Get all the segments
    /// \return  All the segments
    ///
    std::vector<Segment>& segments() const;

    ///
    /// \brief Get the joint dampings to apply to the dynamics
    /// \param Qdot The generalized velocities
    /// \return The joint dampings to apply to the dynamics
    ///
    GeneralizedTorque computeDampedTau(
        const GeneralizedVelocity &Qdot) const;

public:

    ///
    /// \brief Update the kinematic variables such as body velocities and accelerations in the model to reflect the variables passed to this function
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    ///
#ifdef BIORBD_USE_CASADI_MATH
Joints
#else
Joints&
#endif
UpdateKinematicsCustom(
        const GeneralizedCoordinates *Q = nullptr,
        const GeneralizedVelocity *Qdot = nullptr,
        const rigidbody::GeneralizedAcceleration *Qddot = nullptr);


    // -- POSITION INTERFACE OF THE MODEL -- //

    ///
    /// \brief Return the joint coordinate system (JCS) in global reference frame at a given Q
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be updated
    /// \return The JCS in global reference frame at a given Q
    ///
    std::vector<utils::RotoTrans> allGlobalJCS(
        const GeneralizedCoordinates &Q,
        bool updateKin = true    
    );

    ///
    /// \brief Return the joint coordinate system (JCS) for the segment in global reference frame at a given Q
    /// \param Q The generalized coordinates
    /// \param name The name of the segment
    /// \param updateKin If the kinematics should be updated
    /// \return The JCS of the segment in global reference frame at a given Q
    ///
    utils::RotoTrans globalJCS(
        const GeneralizedCoordinates &Q,
        const utils::String &name, 
        bool updateKin = true);

    ///
    /// \brief Return the joint coordinate system (JCS) for the segment idx in global reference frame at a given Q
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param updateKin If the kinematics should be updated
    /// \return The JCS of the segment idx in global reference frame at a given Q
    ///
    utils::RotoTrans globalJCS(
        const GeneralizedCoordinates &Q,
        size_t idx, 
        bool updateKin = true);

    ///
    /// \brief Return all the joint coordinate system (JCS) in its parent reference frame
    /// \return All the JCS in parent reference frame
    ///
    std::vector<utils::RotoTrans> localJCS() const;

    ///
    /// \brief Return the joint coordinate system (JCS) of the segment its parent reference frame
    /// \param name The name of the segment
    /// \return The JCS of the segment in parent reference frame
    ///
    utils::RotoTrans localJCS(
        const utils::String &name) const;

    ///
    /// \brief Return the joint coordinate system (JCS) of the segment idx its parent reference frame
    /// \param idx The index of the segment
    /// \return The JCS of the segment idx in parent reference frame
    ///
    utils::RotoTrans localJCS(
        const size_t idx) const;

    ///
    /// \brief Interface to call CalcBodyToBaseCoordinates. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param segmentName The name of the segment
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Vector3d CalcBodyToBaseCoordinates(
        const rigidbody::GeneralizedCoordinates& Q,
        utils::String segmentName,
        const utils::Vector3d &pointInLocal,
        bool updateKin = true);

    ///
    /// \brief Interface to call CalcBodyToBaseCoordinates. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param bodyId The index of the segment (obtainable with GetBodyId)
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Vector3d CalcBodyToBaseCoordinates(
        const rigidbody::GeneralizedCoordinates& Q,
        unsigned int bodyId,
        const utils::Vector3d &pointInLocal,
        bool updateKin = true);

    /// 
    /// \brief Interface to call CalcPointVelocity. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param segmentName The name of the segment
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Vector3d CalcPointVelocity(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        utils::String segmentName,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    /// 
    /// \brief Interface to call CalcPointVelocity. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param bodyId The index of the segment (obtainable with GetBodyId)
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Vector3d CalcPointVelocity(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        unsigned int bodyId,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    /// 
    /// \brief Interface to call CalcPointVelocity6D. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param segmentName The name of the segment
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::SpatialVector CalcPointVelocity6D(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        utils::String segmentName,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    /// 
    /// \brief Interface to call CalcPointVelocity6D. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param bodyId The index of the segment (obtainable with GetBodyId)
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::SpatialVector CalcPointVelocity6D(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        unsigned int bodyId,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    /// 
    /// \brief Interface to call CalcPointAcceleration. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param segmentName The name of the segment
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Vector3d CalcPointAcceleration(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        const rigidbody::GeneralizedAcceleration& Qddot,
        utils::String segmentName,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    /// 
    /// \brief Interface to call CalcPointAcceleration. It is pretty much useless in Eigen, but 
    /// fixes an issue in Casadi where RBDL changes internal variables which creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param bodyId The index of the segment (obtainable with GetBodyId)
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Vector3d CalcPointAcceleration(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        const rigidbody::GeneralizedAcceleration& Qddot,
        unsigned int bodyId,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    ///
    /// \brief Interface to call CalcPointJacobian. Instead of sending the jacobian as an parameter, it is returned.
    /// It is pretty much useless in Eigen, but fixes an issue in Casadi where RBDL changes internal variables which 
    /// creates a free variables issue
    /// \param Q The generalized coordinates/// 
    /// \param segmentName The name of the segment
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Matrix CalcPointJacobian(
        const rigidbody::GeneralizedCoordinates& Q,
        utils::String segmentName,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    ///
    /// \brief Interface to call CalcPointJacobian. Instead of sending the jacobian as an parameter, it is returned.
    /// It is pretty much useless in Eigen, but fixes an issue in Casadi where RBDL changes internal variables which 
    /// creates a free variables issue
    /// \param Q The generalized coordinates
    /// \param bodyId The index of the segment (obtainable with GetBodyId)
    /// \param pointInLocal The point in the body
    /// \param updateKin If the kinematics of the model should be computed (always true for casadi)
    utils::Matrix CalcPointJacobian(
        const rigidbody::GeneralizedCoordinates& Q,
        unsigned int bodyId,
        const utils::Vector3d& pointInLocal,
        bool updateKin = true
    );

    ///
    /// \brief Project a point on specific axis of a segment
    /// \param Q The generalized coordinates
    /// \param v The point to project
    /// \param segmentIdx The segment index to project the marker on
    /// \param axesToRemove The axis to remove
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the projected marker
    ///
    NodeSegment projectPoint(
        const GeneralizedCoordinates &Q,
        const utils::Vector3d &v,
        int segmentIdx,
        const utils::String& axesToRemove,
        bool updateKin=true);

    ///
    /// \brief Project multiples points on their respective segment
    /// \param Q The generalized coordinates
    /// \param v All the points to project. The number of points must match the number of marker in the joint model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The projected markers from points corresponding to markers from the model
    ///
    /// Return projected markers from points corresponding to markers from the model.
    /// The vector needs to be equal to the number of markers and in the order given
    /// by Markers and in global coordinates
    ///
    std::vector<NodeSegment>  projectPoint(
        const GeneralizedCoordinates &Q,
        const std::vector<NodeSegment> &v,
        bool updateKin=true);

    ///
    /// \brief Return the projected markers from a point corresponding to a marker from the model
    /// \param Q The generalized coordinates
    /// \param n A reference to a marker from the model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The projected markers from a point corresponding to a marker from the model
    ///
    NodeSegment projectPoint(
        const GeneralizedCoordinates& Q,
        const NodeSegment& n,
        bool updateKin);

    ///
    /// \brief Return the jacobian matrix of the projected markers for a marker from the model
    /// \param Q The generalized coordinates
    /// \param p A reference to a marker from the model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The jacobian matrix of the projected marker
    ///
    utils::Matrix projectPointJacobian(
        const GeneralizedCoordinates &Q,
        NodeSegment p,
        bool updateKin);

    ///
    /// \brief Return the Jacobian matrix of a projected marker on the segment segmentIdx
    /// \param Q The generalized coordinates
    /// \param v The marker to project
    /// \param segmentIdx The index of the segment to project the marker on
    /// \param axesToRemove The axes to remove
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The Jacobian matrix of a projected marker on the segment segmentIdx
    ///
    utils::Matrix projectPointJacobian(
        const GeneralizedCoordinates &Q,
        const utils::Vector3d &v,
        int segmentIdx,
        const utils::String& axesToRemove,
        bool updateKin);

    ///
    /// \brief Return the jacobian matrix of the projected markers
    /// \param Q The generalized coordinates
    /// \param v All the markers. The size of which must match the one in the joint model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The jacobian matrix of the projected markers
    ///
    /// Return the jacobian matrix of projected markers from points corresponding to markers from the model.
    /// The vector needs to be equal to the number of markers and in the order given
    /// by Markers and in global coordinates
    ///
    std::vector<utils::Matrix> projectPointJacobian(
        const GeneralizedCoordinates &Q,
        const std::vector<NodeSegment> &v,
        bool updateKin);
    // ------------------------------------- //


    // -- MASS RELATED STUFF -- //
    ///
    /// \brief Return the total mass of the model
    /// \return The toal mass of the model
    ///
    utils::Scalar mass() const;

    ///
    /// \brief Return the position of the center of mass
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass
    ///
    utils::Vector3d CoM(
        const GeneralizedCoordinates &Q,
        bool updateKin=true);

    /// 
    /// \brief Interface to CalcCenterOfMass. It is pretty much useless in Eigen, but fixes an issue in Casadi 
    /// where RBDL changes internal variables which creates a free variables issue
    /// \param Q The current joint positions
    /// \param Qdot The current joint velocities
    /// \param Qddot (optional input) The current joint accelerations
    /// \param mass (output) total mass of the model
    /// \param com (output) location of the Center of Mass of the model in base coordinates
    /// \param comVelocity (optional output) linear velocity of the COM in base coordinates
    /// \param comAcceleration (optional output) linear acceleration of the COM in base coordinates
    /// \param angularMomentum (optional output) angular momentum of the model at the COM in base coordinates
    /// \param changeOfAngularMomentum (optional output) change of angular momentum of the model at the COM in base coordinates
    /// \param updateKin (optional input) whether the kinematics should be updated (defaults to true)
    /// 
    /// \note When wanting to compute com_acceleration or change_of_angular
    /// momentum one has to provide Qddot. In all other cases one can use NULL
    /// as argument for Qddot.
    /// 
    void CalcCenterOfMass(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        const rigidbody::GeneralizedAcceleration* Qddot,
        utils::Scalar& mass,
        utils::Vector3d& com,
        utils::Vector3d* comVelocity = NULL,
        utils::Vector3d* comAcceleration = NULL,
        utils::Vector3d* angularMomentum = NULL,
        utils::Vector3d* changeOfAngularMomentum = NULL,
        bool updateKin = true
    );

    ///
    /// \brief Return the position of the center of mass of each segment
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass of each segment
    ///
    std::vector<NodeSegment> CoMbySegment(
        const GeneralizedCoordinates &Q,
        bool updateKin=true);

    ///
    /// \brief Return the position of the center of mass of each segment in a matrix
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass of each segment
    ///
    utils::Matrix CoMbySegmentInMatrix(
        const GeneralizedCoordinates &Q,
        bool updateKin=true);

    ///
    /// \brief Return the position of the center of mass of segment idx
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass of segment idx
    ///
    utils::Vector3d CoMbySegment(
        const GeneralizedCoordinates &Q,
        const size_t idx,
        bool updateKin=true);

    ///
    /// \brief Return the velocity of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The velocity of the center of mass
    ///
    utils::Vector3d CoMdot(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The acceleration variables
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The acceleration of the center of mass
    ///
    utils::Vector3d CoMddot(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin=true);

    ///
    /// \brief Return the velocity of the center of mass of each segment
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The velocity of the center of mass of each segment
    ///
    std::vector<utils::Vector3d> CoMdotBySegment(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin=true);

    ///
    /// \brief Return the velocity of the center of mass of segment idx
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The velocity of the center of mass of segment idx
    ///
    utils::Vector3d CoMdotBySegment(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const size_t idx,
        bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass of each segment
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The acceleration variables
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The acceleration of the center of mass of each segment
    ///
    std::vector<utils::Vector3d> CoMddotBySegment(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass of segment idx
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The acceleration variables
    /// \param idx The segment identification
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The acceleration of the center of mass of segment idx
    ///
    utils::Vector3d CoMddotBySegment(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        const size_t idx,
        bool updateKin = true);

    ///
    /// \brief Return the jacobian matrix of the center of mass
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The jacobian matrix of the center of mass
    ///
    utils::Matrix CoMJacobian(
        const GeneralizedCoordinates &Q,
        bool updateKin = true);
    // ------------------------ //


    // -- MESH OF THE MODEL -- //
    ///
    /// \brief Return the vertices of the mesh for all segments in global reference frame
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The vertices of the for all segments
    ///
    std::vector<std::vector<utils::Vector3d>> meshPoints(
                const GeneralizedCoordinates &Q,
                bool updateKin = true);

    ///
    /// \brief Return the vertices of the mesh for the segment idx
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The vertices of the of the segment idx
    ///
    std::vector<utils::Vector3d> meshPoints(
        const GeneralizedCoordinates &Q,
        size_t idx,
        bool updateKin = true);

    ///
    /// \brief Return all the vertices of the mesh points in a matrix
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return All the vertices
    ///
    std::vector<utils::Matrix> meshPointsInMatrix(
        const GeneralizedCoordinates &Q,
        bool updateKin = true
    );

    ///
    /// \brief Return the mesh faces for all the segments
    /// \return The mesh faces for all the segments
    ///
    std::vector<std::vector<MeshFace> > meshFaces() const;

    ///
    /// \brief Return the mesh faces for segment idx
    /// \param idx The index of the segment
    /// \return The mesh face for segment idx
    ///
    const std::vector<MeshFace> &meshFaces(
        size_t idx) const;

    ///
    /// \brief Return the segment mesh
    /// \return The segment mesh
    ///
    std::vector<Mesh> mesh() const;

    ///
    /// \brief Return the segment mesh for segment idx
    /// \param idx The index of the segment
    /// \return The Segment mesh for segment idx
    ///
    const Mesh& mesh(
        size_t  idx) const;
    // ----------------------- //


    // -- ANGULAR MOMENTUM FUNCTIONS -- //
    ///
    /// \brief Calculate the angular momentum of the center of mass. This method
    /// is an interface to CalcAngularMomemtum
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The angular momentum of the center of mass
    ///
    utils::Vector3d angularMomentum(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin = true); // Wrapper pour le moment angulaire

    ///
    /// \brief Get the mass matrix at a given position Q
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be updated
    /// \return The mass matrix
    ///
    utils::Matrix massMatrix(
        const GeneralizedCoordinates &Q,
        bool updateKin = true);

    ///
    /// \brief Get the inverse mass matrix at a given position Q
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be updated
    /// \return The inverse mass matrix
    ///
    utils::Matrix massMatrixInverse(
        const rigidbody::GeneralizedCoordinates &Q,
        bool updateKin = true);

    ///
    /// \brief Calculate the angular momentum of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The angular momentum of the center of mass
    ///
    utils::Vector3d CalcAngularMomentum (
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin);

    ///
    /// \brief Calculate the angular momentum of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The angular momentum of the center of mass
    ///
    utils::Vector3d CalcAngularMomentum (
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin);

    ///
    /// \brief Calculate the segment center of mass angular momentum with respect to the global center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The segments center of mass angular momentum
    ///
    std::vector<utils::Vector3d> CalcSegmentsAngularMomentum (
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin);

    ///
    /// \brief Calculate the segment center of mass angular momentum with respect to the global center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The segments center of mass angular momentum
    ///
    std::vector<utils::Vector3d> CalcSegmentsAngularMomentum (
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin);
        
    ///
    /// \brief Calculate the angular velocity of the model around its center of mass.
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The the angular velocity around the center of mass in the global reference frame
    ///
    utils::Vector3d bodyAngularVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin = true);
    // -------------------------------- //

    ///
    /// \brief Calculate the jacobian matrix of a rotation matrix
    /// \param Q The generalized coordinates
    /// \param segmentIdx The index of the segment
    /// \param rotation The rotation matrix
    /// \param G The jacobian matrix (output)
    /// \param updateKin If the kinematics of the model should be computed
    ///
    void CalcMatRotJacobian (
        const GeneralizedCoordinates &Q,
        size_t segmentIdx,
        const utils::Matrix3d &rotation,
        utils::Matrix &G,
        bool updateKin);

    ///
    /// \brief Calculate the jacobian matrix of a rotation matrix for a given segment idx
    /// \param Q The generalized coordinates
    /// \param segmentIdx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    ///
    utils::Matrix JacobianSegmentRotMat (
            const rigidbody::GeneralizedCoordinates &Q,
            size_t segmentIdx,
            bool updateKin);

    ///
    /// \brief Return the derivate of Q in function of Qdot (if not Quaternion, Qdot is directly returned)
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param k_stab
    /// \return The derivate of Q in function of Qdot
    ///
    GeneralizedVelocity computeQdot(const GeneralizedCoordinates &Q,
        const GeneralizedCoordinates &Qdot,
        const utils::Scalar &k_stab = 1);

    ///
    /// \brief Return the angular velocity of the segment
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the model should be updated
    /// \return The angular velocity of the segment
    ///
    utils::Vector3d segmentAngularVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        size_t idx,
        bool updateKin = true);

    ///
    /// \brief Interface for the Kinetic Energy of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \return The Kinetic Energy
    ///
    ///
    utils::Scalar KineticEnergy(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin = true);

    ///
    /// \brief Interface for the Potential Energy of RBDL
    /// \param Q The Generalized Coordinates
    /// \return The Potential Energy
    ///
    ///
    utils::Scalar PotentialEnergy(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin = true);

    ///
    /// \brief Compute the Lagrangian of the system
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \return The Lagrangian:  kinetic - potential energy
    ///
    ///
    utils::Scalar Lagrangian(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin = true);

    ///
    /// \brief Compute the Total Energy of System
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \return The total energy: kinetic + potential energy
    ///
    ///
    utils::Scalar TotalEnergy(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin = true);


    // ---- DYNAMIC INTERFACE ---- //

    ///
    /// \brief Interface for the inverse dynamics of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Qddot The Generalzed Acceleration
    /// \return The Generalized Torques
    ///
    GeneralizedTorque InverseDynamics(const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const rigidbody::GeneralizedAcceleration& Qddot
    );
    ///
    /// \brief Interface for the inverse dynamics of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Qddot The Generalzed Acceleration
    /// \param externalForces External force acting on the system if there are any
    /// \return The Generalized Torques
    ///
    GeneralizedTorque InverseDynamics(const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const rigidbody::GeneralizedAcceleration& Qddot,
        rigidbody::ExternalForceSet& externalForces
    );

    ///
    /// \brief Interface to NonLinearEffect
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \return The Generalized Torques of the bias effects
    ///
    GeneralizedTorque NonLinearEffect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot
    );
    ///
    /// \brief Interface to NonLinearEffect
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param externalForces External force acting on the system if there are any
    /// \return The Generalized Torques of the bias effects
    ///
    GeneralizedTorque NonLinearEffect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        rigidbody::ExternalForceSet& externalForces
    );

    ///
    /// \brief Interface for the forward dynamics of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \return The Generalized Accelerations
    ///
    rigidbody::GeneralizedAcceleration ForwardDynamics(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau
    );
    ///
    /// \brief Interface for the forward dynamics of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param externalForces External force acting on the system if there are any
    /// \return The Generalized Accelerations
    ///
    rigidbody::GeneralizedAcceleration ForwardDynamics(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau,
        rigidbody::ExternalForceSet& externalForces
    );

    ///
    /// \brief Biorbd's implementation of forward dynamics with a free floating base
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param QddotJoints The Generalized Accelerations of the joints (no root)
    /// \return The Generalized Accelerations of the root
    ///
    rigidbody::GeneralizedAcceleration ForwardDynamicsFreeFloatingBase(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedAcceleration& QddotJoints
    );

    ///
    /// \brief Interface for the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The Generalized Accelerations
    ///
    rigidbody::GeneralizedAcceleration ForwardDynamicsConstraintsDirect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau,
        bool updateKin = true
    );
    ///
    /// \brief Interface for the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param externalForces External force acting on the system if there are any
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The Generalized Accelerations
    ///
    rigidbody::GeneralizedAcceleration ForwardDynamicsConstraintsDirect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau,
        rigidbody::ExternalForceSet& externalForces,
        bool updateKin = true
    );
    ///
    /// \brief Interface for the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param CS The Constraint set that will be filled
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The Generalized Accelerations
    ///
    rigidbody::GeneralizedAcceleration ForwardDynamicsConstraintsDirect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau,
        Contacts& CS,
        bool updateKin = true
    );
    ///
    /// \brief Interface for the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param CS The Constraint set that will be filled
    /// \param externalForces External force acting on the system if there are any
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The Generalized Accelerations
    ///
    rigidbody::GeneralizedAcceleration ForwardDynamicsConstraintsDirect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau,
        Contacts& CS,
        rigidbody::ExternalForceSet& externalForces,
        bool updateKin = true
    );

    ///
    /// \brief Interface for contacts of the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \return The Contraint set
    ///
    utils::Vector ContactForcesFromForwardDynamicsConstraintsDirect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau
    );
    ///
    /// \brief Interface for contacts of the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param Qdot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param externalForces External force acting on the system if there are any
    /// \return The Contraint set
    ///
    utils::Vector ContactForcesFromForwardDynamicsConstraintsDirect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& Qdot,
        const GeneralizedTorque& Tau,
        rigidbody::ExternalForceSet& externalForces
    );

    ///
    /// \brief bodyInertia Return the matrix of inertia of the body expressed
    /// in the global reference frame computed at the center of mass
    /// \param Q The Generalized Coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The total inertia of the body
    ///
    utils::Matrix3d bodyInertia (
        const rigidbody::GeneralizedCoordinates &Q,
        bool updateKin = true);

    ///
    /// \brief Compute the Qdot post from an impact
    /// \param Q The Generalized Coordinates
    /// \param QdotPre The Generalized Velocities before impact
    /// \return The Generalized Velocities post acceleration
    ///
    GeneralizedVelocity ComputeConstraintImpulsesDirect(
        const GeneralizedCoordinates& Q,
        const GeneralizedVelocity& QdotPre);

protected:
    std::shared_ptr<std::vector<Segment>>
            m_segments; ///< All the articulations

    std::shared_ptr<size_t>
    m_nbRoot; ///< The number of DoF on the root segment
    std::shared_ptr<size_t>
    m_nbDof; ///< The total number of degrees of freedom
    std::shared_ptr<size_t> m_nbQ; ///< The total number of Q
    std::shared_ptr<size_t> m_nbQdot; ///< The total number of Qdot
    std::shared_ptr<size_t> m_nbQddot; ///< The total number of Qddot
    std::shared_ptr<size_t>
    m_nRotAQuat; ///< The number of segments per quaternion
    std::shared_ptr<bool>
    m_isKinematicsComputed; ///< If the kinematics are computed
    std::shared_ptr<utils::Scalar>
    m_totalMass; ///< Mass of all the bodies combined

    ///
    /// \brief Calculate the joint coordinate system (JCS) in global reference frame of a specified segment
    /// \param Q The generalized coordinates
    /// \param segmentIdx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The JCS of the segment in global reference frame
    ///
    utils::SpatialTransform CalcBodyWorldTransformation(
        const GeneralizedCoordinates &Q,
        const size_t segmentIdx,
        bool updateKin = true);

    ///
    /// \brief Return the mesh vertices of segment idx
    /// \param RT The RotoTrans of the segment
    /// \param idx The index of the segment
    /// \return The mesh vertices attached to a segment idx
    ///
    std::vector<utils::Vector3d> meshPoints(
        const std::vector<utils::RotoTrans> &RT,
        size_t idx) const;

public:
    ///
    /// \brief Check for the Generalized coordinates, velocities, acceleration and torque dimensions
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param torque The generalized torques
    ///
    void checkGeneralizedDimensions(
        const GeneralizedCoordinates *Q = nullptr,
        const GeneralizedVelocity *Qdot = nullptr,
        const rigidbody::GeneralizedAcceleration *Qddot = nullptr,
        const GeneralizedTorque *torque = nullptr);
};

} // namespace rigidbody
} // namespace BIORBD_NAMESPACE

#endif // BIORBD_RIGIDBODY_JOINTS_H
