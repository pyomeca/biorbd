#ifndef BIORBD_RIGIDBODY_JOINTS_H
#define BIORBD_RIGIDBODY_JOINTS_H

#include <memory>
#include <rbdl/Model.h>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class RotoTrans;
class Matrix;
class Vector3d;
}

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
class NodeSegment;
class Patch;
class Segment;
class SegmentCharacteristics;
class Mesh;
class Integrator;

///
/// \brief Class Joints
///
class BIORBD_API Joints : public RigidBodyDynamics::Model
{
public:

    ///
    /// \brief Construct joints
    ///
    Joints();

    ///
    /// \brief Construct joints
    /// \param other Joint to copy (TODO:?)
    ///
    Joints(const biorbd::rigidbody::Joints& other);

    ///
    /// \brief Properly destroy class
    ///
    virtual ~Joints();

    /// 
    /// \brief Deep copy of the joints
    /// \return Copy of the joints
    ///
    biorbd::rigidbody::Joints DeepCopy() const;

    ///
    /// \brief Deep copy of the joints
    /// \param other The joints to copy
    ///
    void DeepCopy(const biorbd::rigidbody::Joints& other);

    // Set and Get

    /// 
    /// \brief Add a segment
    /// \param segmentName Name of the segment
    /// \param parentName Name of the segment parent
    /// \param translationSequence The translation sequence
    /// \param rotationSequence Cardan sequence of rotation to classify the DoF in rotation
    /// \param characteristics The mass, segment's center of mass, inertia of the segment, etc.
    /// \param centreOfRotation Transformation of the parent to child
    /// \param forcePlates The number of the force platform attached to the Segment (default: -1)
    ///
    unsigned int AddSegment(
            const biorbd::utils::String &segmentName, // Nom du segment
            const biorbd::utils::String &parentName, // Nom du segment
            const biorbd::utils::String &translationSequence,
            const biorbd::utils::String &rotationSequence, // Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::SegmentCharacteristics& characteristics, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& centreOfRotation, // Transformation du parent vers l'enfant
            int forcePlates=-1); // Numéro de la plateforme de force attaché à cet os

    ///
    /// \brief Add a segment
    /// \param segmentName Name of the segment
    /// \param parentName Name of the parent segment
    /// \param translationSequence The translation sequence
    /// \param rotationSequence Cardan sequence of rotation to classify the DoF in rotation
    /// \param centreOfRotation Transformation of the parent to child
    /// \param forcePlates The number of the force platform attached to the segment (default: -1)
    ///
    unsigned int AddSegment(
            const biorbd::utils::String &segmentName, 
            const biorbd::utils::String &parentName, 
            const biorbd::utils::String &translationSequence, 
            const biorbd::rigidbody::SegmentCharacteristics& rotationSequence, 
            const RigidBodyDynamics::Math::SpatialTransform& centreOfRotation, 
            int forcePlates=-1); 


    // -- INFORMATION ON THE MODEL -- //

    ///
    /// \brief Return the biorbd body identification
    /// \param segmentName The name of the segment
    /// \return The biorbd body identification
    ///
    int GetBodyBiorbdId(const biorbd::utils::String &segmentName) const;

    ///
    /// \brief Return the number of generalized torque
    /// \return The number of generalized torque
    ///
    unsigned int nbGeneralizedTorque() const;

    ///
    /// \brief Return the actual number of segment 
    /// \return The actual number of segment
    ///
    unsigned int nbSegment() const;

    ///
    /// \brief Return the number of degrees of freedom (DoF)
    /// \return The number of DoF
    ///
    unsigned int nbDof() const;

    ///
    /// \brief Return the identification of the Segment
    /// \param SegmentName The name of the Segment
    /// \param dofName The name of the degree of freedom (DoF)
    /// \return The identification of the Segment
    ///
    unsigned int getDofIndex(
            const biorbd::utils::String& SegmentName,
            const biorbd::utils::String& dofName);

    ///
    /// \brief Return the names of the degree of freedom (DoF)
    /// \return The names of the DoF
    ///
    std::vector<biorbd::utils::String> nameDof() const;

    ///
    /// \brief Return the number of Q
    /// \return The number of Q
    ///
    unsigned int nbQ() const;

    ///
    /// \brief Return the number of Qdot
    /// \return The number of Qdot
    ///
    unsigned int nbQdot() const;

    ///
    /// \brief Return the number of Qddot
    /// \return The number of Qddot
    ///
    unsigned int nbQddot() const;

    ///
    /// \brief Return the number of elements that are not actuated
    /// \return The number of elements that are not actuated
    ///
    unsigned int nbRoot() const;

    ///
    /// \brief Return the number of quaternions
    /// \return The number of quaternions
    ///
    unsigned int nbQuat() const;

    ///
    /// \brief Set root segment to be actuated or not
    /// \param a True or False 
    ///
    void setIsRootActuated(bool a);

    ///
    /// \brief Determine if root segment is actuated or not
    /// \return True(1) or False(0)
    ///
    bool isRootActuated() const;

    ///
    /// \brief Set model to include external forces or note
    /// \param f True (1) or False (0)
    ///
    void setHasExternalForces(bool f);

    ///
    /// \brief Determine if model has external forces
    /// \return True (1) or False (0)
    ///
    bool hasExternalForces() const;

    ///
    /// \brief Get Segment 
    /// \param idxSegment Identification of the segment
    ///
    const biorbd::rigidbody::Segment& Segment(unsigned int idxSegment) const;

    ///
    /// \brief Get Segment
    /// \param nameSegment Name of the segment
    ///
    const biorbd::rigidbody::Segment& Segment(const biorbd::utils::String& nameSegment) const;
    // ------------------------------ //


    // -- FORCE PLATE DISPATCHER -- //
    ///
    /// \brief TODO: Dispatch the forces from the force plate in a vector 
    /// \param spatialVector 
    /// \param frame
    /// \return A spatial vector with the forces
    ///
   
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
            std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>> &spatialVector,
            unsigned int frame) const;

    ///
    /// \brief TODO: Dispatch the forces from the force plate in a spatial vector
    /// \param sv One spatial vector per force platform
    /// \return A spatial vector with the forces
    ///
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
            std::vector<RigidBodyDynamics::Math::SpatialVector> &sv) const; // un SpatialVector par PF
    // ---------------------------- //


    // -- INTEGRATOR INTERFACE -- //

    ///
    /// \brief Update the kinematic variables such as body velocities and accelerations in the model to reflect the variables passed to this function
    /// \param Q The positional variables of the model
    /// \param Qdot The generalized velocities of the joints
    /// \param Qddot The generalized accelerations of the joints
    /// 
    void UpdateKinematicsCustom(
            const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr);
    ///
    /// \brief Process integration of the kinematics
    /// \param Q The positional variables of the model
    /// \param QDot The generalized velocities of the joints
    /// \param GeneralizedTorque The effectors
    /// \param t0 Start time
    /// \param tend End time
    /// \param timeStep The time step (dt)
    ///
    void integrateKinematics(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& QDot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorque,
            double t0,
            double tend,
            double timeStep); // Process integration (Q, Qdot, effecteurs)
    ///
    /// \brief Get integrated kinematics
    /// \param step The step
    /// \param Q The positional variables of the model
    /// \param QDot The generalized velocities of the joints
    ///  
    void getIntegratedKinematics(
            unsigned int  step,
            biorbd::rigidbody::GeneralizedCoordinates& Q,
            biorbd::rigidbody::GeneralizedCoordinates& QDot);  // Put in a VectorNd the Qs a time t

    ///
    /// \brief Return number of iteration steps
    /// \return Iteration steps
    ///
    unsigned int nbInterationStep() const;
    // -------------------------- //


    // -- POSITION INTERFACE OF THE MODEL -- //

    ///
    /// \brief Return the JCSs in global coordinate for the given Q
    /// \param Q The positional variables of the model
    /// \return The JCSs in global coordinate for the given Q
    ///
    std::vector<biorbd::utils::RotoTrans> allGlobalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q); 

    /// 
    /// \brief Return the JCSs in global coordinate
    /// \return The JCSs in global coordinate 
    ///
    std::vector<biorbd::utils::RotoTrans> allGlobalJCS() const;

    ///
    /// \brief Return the JCSs for segment i in global coordinate system for the given Q
    /// \param Q The positional variables of the model
    /// \param name The name of the segment (TODO?)
    /// \return The JCSs for segment i in global coordinate system for the given Q
    ///
    biorbd::utils::RotoTrans globalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::String &name); 

    /// 
    /// \brief Return the JCSs for segment i in global coordinate system for the given Q
    /// \param Q The positional variables of the model
    /// \param idx The segment identification
    /// \return The JCSs for segment i in global coordinate system for the given Q
    ///
    biorbd::utils::RotoTrans globalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            unsigned int idx); 

    /// 
    /// \brief Return the JCSs for segment i 
    /// \param parentName The name of the segment's parent
    /// \return The JCSs for segment i
    /// 
    biorbd::utils::RotoTrans globalJCS(
            const biorbd::utils::String &parentName) const;

    ///
    /// \brief Return the JCSs for segment i
    /// \param idx The segment identification
    /// \return The JCSs for segment i
    ///
    biorbd::utils::RotoTrans globalJCS(
            unsigned int idx) const;

    ///
    /// \brief Return the JCSs in parent coordinate system
    /// \return The JCSs in local coordinate system
    ///
    std::vector<biorbd::utils::RotoTrans> localJCS() const;

    ///
    /// \brief Return the JCSs for segment i in parent coordinate system
    /// \param segmentName The name of the segment
    /// \return The JCSs in local coordinate system
    ///
    biorbd::utils::RotoTrans localJCS(const biorbd::utils::String &segmentName) const;  

    ///
    /// \brief Return the JCS for segment i in parent coordinate system
    /// \param i The segment identification
    /// \return The JCS for segment i in parent coordinate system
    ///
    biorbd::utils::RotoTrans localJCS(const unsigned int i) const; 



    ///
    /// \brief Project a point in the global coordinate system
    /// \param Q The positional variables of the model
    /// \param v The nodes
    /// \param SegmentIdx The Segment identification
    /// \param axesToRemove
    /// \param updateKin (default: True)
    /// \return 
    ///
    biorbd::rigidbody::NodeSegment projectPoint(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::Vector3d &v,
            int SegmentIdx,
            const biorbd::utils::String& axesToRemove,
            bool updateKin=true);

    ///
    /// \brief Return projected markers from points corresponding to markers from the model (The vector needs to be equal to the number of markers and in the order given by Markers and in global coordinates)
    /// \param Q The positional variables of the model
    /// \param v The nodes
    /// \param updateKin (default: True)
    /// \return Projected markers from points corresponding to markers from the model
    ///
    std::vector<biorbd::rigidbody::NodeSegment>  projectPoint(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const std::vector<biorbd::rigidbody::NodeSegment> &v,
            bool updateKin=true); 

    ///
    /// \brief Return the Jacobian matrix of the projected markers from points corresponding to markers from the model
    /// \param Q
    /// \param p 
    /// \param updateKin
    /// \return 
    ///
    biorbd::utils::Matrix projectPointJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            biorbd::rigidbody::NodeSegment p,
            bool updateKin);


    biorbd::rigidbody::NodeSegment projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::NodeSegment& n,
        bool updateKin);
    ///
    /// \brief Return the Jacobian matrix of the projected markers from points corresponding to markers from the model
    /// \param Q
    /// \param v
    /// \param SegmentIdx The identification of the Segment
    /// \param axesToRemove
    /// \param updateKin
    /// \return
    ///
    biorbd::utils::Matrix projectPointJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::Vector3d &v,
            int SegmentIdx,
            const biorbd::utils::String& axesToRemove,
            bool updateKin);

    ///
    /// \brief Return the Jacobian matrix of the projected markers from points corresponding to markers from the model (The vector must be equal to the number of markers and the order given by Markers and in global coordinates)
    /// \param Q
    /// \param v
    /// \param updateKin
    /// \return 
    ///
    std::vector<biorbd::utils::Matrix> projectPointJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const std::vector<biorbd::rigidbody::NodeSegment> &v,
            bool updateKin); 
    // ------------------------------------- //


    // -- MASS RELATED STUFF -- //
    ///
    /// \brief Return the mass of all segments
    /// \return The mass of all segments
    ///
    double mass() const; 

    ///
    /// \brief Return the position of the center of mass
    /// \return The position of the center of mass
    ///
    biorbd::utils::Vector3d CoM(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin=true); 

    ///
    /// \brief Return the position of the center of mass of each segment
    /// \param Q The positional variables of the model
    /// \param updateKin (default: True)
    /// \return The position of the center of mass of each segment
    ///
    std::vector<biorbd::rigidbody::NodeSegment> CoMbySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin=true);

    ///
    /// \brief Return the position of the center of mass of segment i
    /// \param Q The position variables of the model
    /// \param i The segment identification
    /// \param updateKin (default: True)
    /// \return The position of the center of mass of segment i
    ///
    biorbd::utils::Vector3d CoMbySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int i,
            bool updateKin=true);

    ///
    /// \brief Return the velocity of the center of mass 
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \return The velocity of the center of mass
    ///
    biorbd::utils::Vector3d CoMdot(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot); 

    ///
    /// \brief Return the acceleration of the center of mass 
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \return The acceleration of the center of mass
    ///
    biorbd::utils::Vector3d CoMddot(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot); 

    /// 
    /// \brief Return the velocity of the center of mass of each segment
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param updateKin (default: True)
    /// \return The velocity of the center of mass of each segment
    ///
    std::vector<biorbd::utils::Vector3d> CoMdotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool updateKin=true); 

    ///
    /// \brief Return the velocity of the center of mass of segment i
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param i The segment identification
    /// \param updateKin (default: True)
    /// \return The velocity of the center of mass of segment i
    ///
    biorbd::utils::Vector3d CoMdotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const unsigned int i,
            bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass of each segment
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \param updateKin (default: True)
    /// \return The acceleration of the center of mass of each segment
    ///
    std::vector<biorbd::utils::Vector3d> CoMddotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass of segment i
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \param i The segment identification
    /// \param updateKin (default: True)
    /// \return The acceleration of the center of mass of segment i
    ///
    biorbd::utils::Vector3d CoMddotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            const unsigned int i,
            bool updateKin=true);

    /// 
    /// \brief Return the Jacobian of the center of mass
    /// \param Q The position variables of the model
    /// \return The Jacobian of the center of mass
    ///
    biorbd::utils::Matrix CoMJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q); 
    // ------------------------ //


    // -- MESH OF THE MODEL -- //
    ///
    /// \brief Return the position of the meshing for all segments
    /// \param Q The position variables of the model
    /// \param updateKin (default: True)
    /// \return The position of the meshing for all segments
    ///
    std::vector<std::vector<biorbd::utils::Vector3d>> meshPoints(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true);

    ///
    /// \brief Return the position of the meshing for segment i
    /// \param Q The position variables of the model
    /// \param idx The segment identification
    /// \param updateKin (default: True)
    /// \return The position of the meshing for segment i
    ///
    std::vector<biorbd::utils::Vector3d> meshPoints(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            unsigned int  idx,
            bool updateKin = true);

    ///
    /// \brief Return the mesh points in matrix
    /// \param Q The position variables of the model
    /// \param updateKin (default: True)
    /// \return All the mesh points
    ///
    std::vector<biorbd::utils::Matrix> meshPointsInMatrix(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true
            );

    ///
    /// \brief Return the mesh patch for all segments
    /// \return The mesh patch for all segments
    ///
    std::vector<std::vector<Patch> > meshPatch() const;

    ///
    /// \brief Return the mesh patch for segment i
    /// \param i The segment identification
    /// \return The mesh patch for segment i
    ///
    const std::vector<biorbd::rigidbody::Patch> &meshPatch(unsigned int i) const;

    ///
    /// \brief Return the Segment mesh
    /// \return The Segment mesh
    ///
    std::vector<biorbd::rigidbody::Mesh> Mesh() const;
    
    ///
    /// \brief Return the Segment mesh for segment i
    /// \param idx The segment identification
    /// \return The Segment mesh for segment i
    ///
    const biorbd::rigidbody::Mesh& Mesh(unsigned int  idx) const;
    // ----------------------- //


    // -- ANGULAR MOMENTUM FUNCTIONS -- //

    ///
    /// \brief Calculate the angular momentum (wrapper for the angular momentum)
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param updateKin (default: True)
    /// \return The angular momentum
    ///
    biorbd::utils::Vector3d angularMomentum(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool updateKin = true); // Wrapper pour le moment angulaire
    // Réimplémentation de la fonction CalcAngularMomentum car elle a une erreur (inversion du calcul du com)
   
    ///
    /// \brief Calculate the angular momentum 
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param update_kinematics
    /// \return The angular momentum
    ///
    biorbd::utils::Vector3d CalcAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool update_kinematics);

    ///
    /// \brief Calculate the angular momentum 
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \param update_kinematics
    /// \return The angular momentum
    ///
    biorbd::utils::Vector3d CalcAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool update_kinematics);

    ///
    /// \brief Calculate the segment angular momentum 
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param update_kinematics Update kinematics if necessary
    /// \return The segment angular momentum
    ///
    std::vector<biorbd::utils::Vector3d> CalcSegmentsAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool update_kinematics);

    ///
    /// \brief Calculate the segment angular momentum 
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \param update_kinematics Update kinematics if necessary
    /// \return The segment angular momentum
    ///
    std::vector<biorbd::utils::Vector3d> CalcSegmentsAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool update_kinematics);
    // -------------------------------- //

    ///
    /// \brief Calculate the Jacobian matrix of a rotation matrix
    /// \param Q The position variables of the model
    /// \param body_id The body identification
    /// \param rotation The rotation matrix
    /// \param G TODO?
    /// \param update_kinematics Update kinematics if necessary
    ///
    void CalcMatRotJacobian (
            const RigidBodyDynamics::Math::VectorNd &Q,
            unsigned int body_id,
            const RigidBodyDynamics::Math::Matrix3d &rotation,
            RigidBodyDynamics::Math::MatrixNd &G,
            bool update_kinematics);

    ///
    /// \brief Forward Dynamics TODO
    /// \param Q The position variables of the model
    /// \param QDot The velocity variables of the model
    /// \param GeneralizedTorque The generalized torque of the model
    /// \param CS The constraint set
    /// \param QDDot The acceleration variables of the model
    /// 
    void ForwardDynamicsContactsLagrangian (
            const RigidBodyDynamics::Math::VectorNd &Q,
            const RigidBodyDynamics::Math::VectorNd &QDot,
            const RigidBodyDynamics::Math::VectorNd &GeneralizedTorque,
            RigidBodyDynamics::ConstraintSet &CS,
            RigidBodyDynamics::Math::VectorNd &QDDot);
    ///
    /// \brief Return the derivate of Q in function of Qdot (if not Quaternion, Qdot is directly returned)
    /// \param Q The position variables of the model
    /// \param QDot The velocity variables of the model
    /// \param QDotOut The output vector
    /// \return The derivate of Q in function of Qdot
    ///
	
    biorbd::rigidbody::GeneralizedCoordinates computeQdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &QDot,
        const double k_stab = 1); // Cette fonction retourne la dérivée de Q en fonction de Qdot (Si pas de Quaternion, QDot est directement retourné)

protected:
    std::shared_ptr<std::vector<biorbd::rigidbody::Segment>> m_segments; ///< All the articulations

    std::shared_ptr<biorbd::rigidbody::Integrator> m_integrator; ///< The integrator
    std::shared_ptr<unsigned int> m_nbRoot; ///< The number of DoF on the root segment
    std::shared_ptr<unsigned int> m_nbDof; ///< The total number of degrees of freedom  
    std::shared_ptr<unsigned int> m_nbQ; ///< The total number of Q
    std::shared_ptr<unsigned int> m_nbQdot; ///< The total number of qdot
    std::shared_ptr<unsigned int> m_nbQddot; ///< The total number of qddot
    std::shared_ptr<unsigned int> m_nRotAQuat; ///< The number of segments per quaternion
    std::shared_ptr<bool> m_isRootActuated; ///< If the root segment is controled or not
    std::shared_ptr<bool> m_hasExternalForces; ///< If the model includes external force
    std::shared_ptr<bool> m_isKinematicsComputed; ///< If the kinematics are computed
    std::shared_ptr<double> m_totalMass; ///< Mass of all the bodies

    ///
    /// \brief Calculate the JCS in global
    /// \param Q The position variables of the model
    /// \param body_id The body identification
    /// \param update_kinematics Update the kinematics if necessary
    /// \return The JCS in global
    ///
    RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int body_id,
            bool update_kinematics); 

    ///
    /// \brief Calculate the JCS in global
    /// \param body_id The body identification
    /// \return The JCS in global
    ///
    RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(
            const unsigned int body_id) const; 

    ///
    /// \brief Return the mesh points of segment i
    /// \param RT The rotation and translation vector
    /// \param idx The index of the segment
    /// \return The mesh points of segment i
    ///
    std::vector<biorbd::utils::Vector3d> meshPoints(
            const std::vector<biorbd::utils::RotoTrans> &RT,
            unsigned int  idx) const;

};

}}

#endif // BIORBD_RIGIDBODY_JOINTS_H
