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
class Node3d;
}

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
class NodeBone;
class Patch;
class Bone;
class BoneCharacteristics;
class BoneMesh;
class Integrator;

class BIORBD_API Joints : public RigidBodyDynamics::Model
{
public:
    Joints();
    Joints(const biorbd::rigidbody::Joints& other);
    virtual ~Joints();
    biorbd::rigidbody::Joints DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::Joints& other);

    // Set and Get
    unsigned int AddBone(
            const biorbd::utils::String &segmentName, // Nom du segment
            const biorbd::utils::String &parentName, // Nom du segment
            const biorbd::utils::String &translationSequence,
            const biorbd::utils::String &rotationSequence, // Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::BoneCharacteristics& characteristics, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& centreOfRotation, // Transformation du parent vers l'enfant
            int forcePlates=-1); // Numéro de la plateforme de force attaché à cet os
    unsigned int AddBone(
            const biorbd::utils::String &segmentName, // Nom du segment
            const biorbd::utils::String &parentName, // Nom du segment
            const biorbd::utils::String &translationSequence, // Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::BoneCharacteristics& rotationSequence, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& centreOfRotation, // Transformation du parent vers l'enfant
            int forcePlates=-1); // Numéro de la plateforme de force attaché à cet os


    // -- INFORMATION ON THE MODEL -- //
    int GetBodyBiorbdId(const biorbd::utils::String &segmentName) const;
    unsigned int nbGeneralizedTorque() const;
    unsigned int nbBone() const; // Return the actual number of segments
    unsigned int nbDof() const;
    unsigned int getDofIndex(
            const biorbd::utils::String& boneName,
            const biorbd::utils::String& dofName);
    std::vector<std::string> nameDof() const;
    unsigned int nbQ() const;
    unsigned int nbQdot() const;
    unsigned int nbQddot() const;
    unsigned int nbRoot() const; // retourne le nombre d'élément qui ne sont pas actionnés
    unsigned int nbQuat() const;
    void setIsRootActuated(bool a); // Determine if root segment is actuated or not
    bool isRootActuated() const;
    void setHasExternalForces(bool f); // If the model includes external force
    bool hasExternalForces() const;
    const biorbd::rigidbody::Bone& bone(unsigned int idxSegment) const;
    const biorbd::rigidbody::Bone& bone(const biorbd::utils::String& nameSegment) const;
    // ------------------------------ //


    // -- FORCE PLATE DISPATCHER -- //
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
            std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>> &spatialVector,
            unsigned int frame) const;
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
            std::vector<RigidBodyDynamics::Math::SpatialVector> &) const; // un SpatialVector par PF
    // ---------------------------- //


    // -- INTEGRATOR INTERFACE -- //
    void UpdateKinematicsCustom(
            const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr);
    void integrateKinematics(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& QDot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorque); // Process integration (Q, Qdot, effecteurs)
    void getIntegratedKinematics(
            unsigned int  step,
            biorbd::rigidbody::GeneralizedCoordinates& Q,
            biorbd::rigidbody::GeneralizedCoordinates& Qdot);  // Put in a VectorNd the Qs a time t
    unsigned int nbInterationStep() const;
    // -------------------------- //


    // -- POSITION INTERFACE OF THE MODEL -- //
    std::vector<biorbd::utils::RotoTrans> allGlobalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q); // Return the JCSs in global coordinate system for the given q
    std::vector<biorbd::utils::RotoTrans> allGlobalJCS() const; // Return the JCSs in global coordinate system for the given q
    biorbd::utils::RotoTrans globalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::String &parentName);  // Return the JCS for segment i in global coordinate system for the given q
    biorbd::utils::RotoTrans globalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            unsigned int idx);  // Return the JCS for segment i in global coordinate system for the given q
    biorbd::utils::RotoTrans globalJCS(
            const biorbd::utils::String &parentName) const;
    biorbd::utils::RotoTrans globalJCS(
            unsigned int idx) const;
    std::vector<biorbd::utils::RotoTrans> localJCS() const; // Return the JCSs in global coordinate system for the given q
    biorbd::utils::RotoTrans localJCS(const biorbd::utils::String &segmentName) const;  // Return the JCS for segment named String in parent coordinate system
    biorbd::utils::RotoTrans localJCS(const unsigned int i) const;  // Return the JCS for segment i in parent coordinate system
    biorbd::rigidbody::NodeBone projectPoint(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::NodeBone&, bool updateKin=true); // Projeter selon les axes/plan déterminé déjà dans nodeBone
    biorbd::rigidbody::NodeBone projectPoint(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::Node3d &v,
            int boneIdx,
            const biorbd::utils::String& axesToRemove,
            bool updateKin=true); // Projeter un point dans le repère global
    std::vector<biorbd::rigidbody::NodeBone>  projectPoint(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const std::vector<biorbd::rigidbody::NodeBone> &v,
            bool updateKin=true); //Marqueurs projetés de points correspondant aux marqueurs du modèle (le vector doit être égal au nombre de marqueur et dans l'ordre donné par Markers)
    biorbd::utils::Matrix projectPointJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            biorbd::rigidbody::NodeBone p,
            bool updateKin);
    biorbd::utils::Matrix projectPointJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::Node3d &v,
            int boneIdx,
            const biorbd::utils::String& axesToRemove,
            bool updateKin);
    std::vector<biorbd::utils::Matrix> projectPointJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const std::vector<biorbd::rigidbody::NodeBone> &v,
            bool updateKin); // Matrice jacobienne des marqueurs projetés de points correspondant aux marqueurs du modèle (le vector doit être égal au nombre de marqueur et dans l'ordre donné par Markers et dans le repère global)
    // ------------------------------------- //


    // -- MASS RELATED STUFF -- //
    double mass() const; // retourne la masse de tous les segments
    biorbd::utils::Node3d CoM(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin=true); // Position du centre de masse
    std::vector<biorbd::rigidbody::NodeBone> CoMbySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin=true); // Position du centre de masse de chaque segment
    biorbd::utils::Node3d CoMbySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int i,
            bool updateKin=true); // Position du centre de masse du segment i
    biorbd::utils::Node3d CoMdot(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot); // Vitesse du CoM
    biorbd::utils::Node3d CoMddot(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot); // Acceleration du CoM
    std::vector<biorbd::utils::Node3d> CoMdotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool updateKin=true); // vitesse du centre de masse de chaque segment
    biorbd::utils::Node3d CoMdotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const unsigned int i,
            bool updateKin=true); // vitesse du centre de masse du segment i
    std::vector<biorbd::utils::Node3d> CoMddotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool updateKin=true); // accélération du centre de masse de chaque segment
    biorbd::utils::Node3d CoMddotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            const unsigned int i,
            bool updateKin=true); // accélération du centre de masse du segment i
    biorbd::utils::Matrix CoMJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q); // Jacobienne
    // ------------------------ //


    // -- MESH OF THE MODEL -- //
    std::vector<std::vector<biorbd::utils::Node3d>> meshPoints(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true);
    std::vector<biorbd::utils::Node3d> meshPoints(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            unsigned int  idx,
            bool updateKin = true);
    std::vector<biorbd::utils::Matrix> meshPointsInMatrix(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true
            );
    std::vector<std::vector<Patch> > meshPatch() const;
    const std::vector<biorbd::rigidbody::Patch> &meshPatch(unsigned int i) const;
    std::vector<biorbd::rigidbody::BoneMesh> boneMesh() const;
    const biorbd::rigidbody::BoneMesh& boneMesh(unsigned int  idx) const;
    // ----------------------- //


    // -- ANGULAR MOMENTUM FUNCTIONS -- //
    biorbd::utils::Node3d angularMomentum(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool updateKin = true); // Wrapper pour le moment angulaire
    // Réimplémentation de la fonction CalcAngularMomentum car elle a une erreur (inversion du calcul du com)
    biorbd::utils::Node3d CalcAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool update_kinematics);
    biorbd::utils::Node3d CalcAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool update_kinematics);
    std::vector<biorbd::utils::Node3d> CalcSegmentsAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool update_kinematics);
    std::vector<biorbd::utils::Node3d> CalcSegmentsAngularMomentum (
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool update_kinematics);
    // -------------------------------- //

    void CalcMatRotJacobian (
            const RigidBodyDynamics::Math::VectorNd &Q,
            unsigned int body_id,
            const RigidBodyDynamics::Math::Matrix3d &rotation,
            RigidBodyDynamics::Math::MatrixNd &G,
            bool update_kinematics); // Calcule la matrice jacobienne d'une matrice de rotation

    void ForwardDynamicsContactsLagrangian (
            const RigidBodyDynamics::Math::VectorNd &Q,
            const RigidBodyDynamics::Math::VectorNd &QDot,
            const RigidBodyDynamics::Math::VectorNd &GeneralizedTorque,
            RigidBodyDynamics::ConstraintSet &CS,
            RigidBodyDynamics::Math::VectorNd &QDDot);
	
    biorbd::rigidbody::GeneralizedCoordinates computeQdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &QDot); // Cette fonction retourne la dérivée de Q en fonction de Qdot (Si pas de Quaternion, QDot est directement retourné)

protected:
    std::shared_ptr<std::vector<biorbd::rigidbody::Bone>> m_bones; // Toutes les articulations

    std::shared_ptr<biorbd::rigidbody::Integrator> m_integrator;
    std::shared_ptr<unsigned int> m_nbRoot; // Nombre de dof sur le segment racine
    std::shared_ptr<unsigned int> m_nDof; // Nombre de degré de liberté total
    std::shared_ptr<unsigned int> m_nbQ; // Nombre de q au total
    std::shared_ptr<unsigned int> m_nbQdot; // Nombre de qdot au total
    std::shared_ptr<unsigned int> m_nbQddot; // Nombre de qddot au total
    std::shared_ptr<unsigned int> m_nRotAQuat; // Nombre de segments par quaternion
    std::shared_ptr<bool> m_isRootActuated; // If the root segment is controled or not
    std::shared_ptr<bool> m_hasExternalForces; // If the model includes external force
    std::shared_ptr<bool> m_isKinematicsComputed;
    std::shared_ptr<double> m_totalMass; // Masse de tous les corps
    RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int body_id,
            bool update_kinematics); // Calculate the JCS in global
    RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(
            const unsigned int body_id) const; // Calculate the JCS in global
    std::vector<biorbd::utils::Node3d> meshPoints(
            const std::vector<biorbd::utils::RotoTrans>&,
            unsigned int  idx) const;

};

}}

#endif // BIORBD_RIGIDBODY_JOINTS_H
