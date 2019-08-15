#ifndef BIORBD_RIGIDBODY_JOINTS_H
#define BIORBD_RIGIDBODY_JOINTS_H

#include <rbdl/Model.h>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace utils {
class Attitude;
class Integrator;
class Matrix;
class Node;
}

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
class Markers;
class NodeBone;
class Patch;
class Bone;
class Caracteristics;
class Mesh;

class BIORBD_API Joints : public RigidBodyDynamics::Model
{
public:
    Joints();
    Joints(const Joints&);
    virtual ~Joints();

    // Set and Get
    unsigned int AddBone(
            const unsigned int &parent_id, // Numéro du parent
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::Caracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            const biorbd::utils::String &name="", // Nom du segment
            const int &PF=-1); // Numéro de la plateforme de force attaché à cet os
    unsigned int AddBone(
            const unsigned int &parent_id, // Numéro du parent
            const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::Caracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            const biorbd::utils::String &name="", // Nom du segment
            const int &PF=-1); // Numéro de la plateforme de force attaché à cet os


    // -- INFORMATION ON THE MODEL -- //
    int GetBodyBiorbdId(const biorbd::utils::String &) const;
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
    void setIsRootActuated(const bool &a); // Determine if root segment is actuated or not
    bool isRootActuated() const;
    void setHasExternalForces(const bool &f); // If the model includes external force
    bool hasExternalForces() const;
    const biorbd::rigidbody::Bone& bone(unsigned int i) const;
    const biorbd::rigidbody::Bone& bone(const biorbd::utils::String&) const;
    // ------------------------------ //


    // -- FORCE PLATE DISPATCHER -- //
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
            std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>> &,
            const unsigned int &frame) const;
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
            std::vector<RigidBodyDynamics::Math::SpatialVector> &) const; // un SpatialVector par PF
    // ---------------------------- //


    // -- INTEGRATOR INTERFACE -- //
    void UpdateKinematicsCustom(
            Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr);
    void integrateKinematics(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& QDot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorque); // Process integration (Q, Qdot, effecteurs)
    void getIntegratedKinematics(
            const unsigned int& step,
            biorbd::rigidbody::GeneralizedCoordinates& Q,
            biorbd::rigidbody::GeneralizedCoordinates& Qdot);  // Put in a VectorNd the Qs a time t
    unsigned int nbInterationStep() const;
    // -------------------------- //


    // -- POSITION INTERFACE OF THE MODEL -- //
    std::vector<biorbd::utils::Attitude> globalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &,
            const bool updateKin = true); // Return the JCSs in global coordinate system for the given q
    biorbd::utils::Attitude globalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::String &parentName,
            const bool updateKin = true);  // Return the JCS for segment i in global coordinate system for the given q
    biorbd::utils::Attitude globalJCS(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int i,
            const bool updateKin = true);  // Return the JCS for segment i in global coordinate system for the given q
    std::vector<biorbd::utils::Attitude> localJCS() const; // Return the JCSs in global coordinate system for the given q
    biorbd::utils::Attitude localJCS(const biorbd::utils::String &) const;  // Return the JCS for segment named String in parent coordinate system
    biorbd::utils::Attitude localJCS(const unsigned int i) const;  // Return the JCS for segment i in parent coordinate system
    biorbd::rigidbody::NodeBone projectPoint(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::NodeBone&, bool updateKin=true); // Projeter selon les axes/plan déterminé déjà dans nodeBone
    biorbd::rigidbody::NodeBone projectPoint(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const Eigen::Vector3d &v,
            int boneIdx,
            const biorbd::utils::String& axesToRemove,
            bool updateKin=true); // Projeter un point dans le repère global
    std::vector<biorbd::rigidbody::NodeBone>  projectPoint(
            const biorbd::rigidbody::Markers &marks,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const std::vector<biorbd::rigidbody::NodeBone> &v,
            bool updateKin=true); //Marqueurs projetés de points correspondant aux marqueurs du modèle (le vector doit être égal au nombre de marqueur et dans l'ordre donné par Tags)
    biorbd::utils::Matrix projectPointJacobian(
            Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            biorbd::rigidbody::NodeBone p,
            bool updateKin);
    biorbd::utils::Matrix projectPointJacobian(
            Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const Eigen::Vector3d &v,
            int boneIdx,
            const biorbd::utils::String& axesToRemove,
            bool updateKin);
    std::vector<biorbd::utils::Matrix> projectPointJacobian(
            Joints& model,
            const biorbd::rigidbody::Markers &marks,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const std::vector<biorbd::rigidbody::NodeBone> &v,
            bool updateKin); // Matrice jacobienne des marqueurs projetés de points correspondant aux marqueurs du modèle (le vector doit être égal au nombre de marqueur et dans l'ordre donné par Tags et dans le repère global)
    // ------------------------------------- //


    // -- MASS RELATED STUFF -- //
    double mass() const; // retourne la masse de tous les segments
    biorbd::utils::Node CoM(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin=true); // Position du centre de masse
    std::vector<biorbd::rigidbody::NodeBone> CoMbySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin=true); // Position du centre de masse de chaque segment
    biorbd::rigidbody::NodeBone CoMbySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int i,
            bool updateKin=true); // Position du centre de masse du segment i
    RigidBodyDynamics::Math::Vector3d CoMdot(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot); // Vitesse du CoM
    RigidBodyDynamics::Math::Vector3d CoMddot(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot); // Acceleration du CoM
    std::vector<RigidBodyDynamics::Math::Vector3d> CoMdotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool updateKin=true); // vitesse du centre de masse de chaque segment
    RigidBodyDynamics::Math::Vector3d CoMdotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const unsigned int i,
            bool updateKin=true); // vitesse du centre de masse du segment i
    std::vector<RigidBodyDynamics::Math::Vector3d> CoMddotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool updateKin=true); // accélération du centre de masse de chaque segment
    RigidBodyDynamics::Math::Vector3d CoMddotBySegment(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            const unsigned int i,
            bool updateKin=true); // accélération du centre de masse du segment i
    biorbd::utils::Matrix CoMJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q); // Jacobienne
    // ------------------------ //


    // -- MESH OF THE MODEL -- //
    std::vector<std::vector<biorbd::rigidbody::NodeBone>> meshPoints(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const bool updateKin = true);
    std::vector<biorbd::rigidbody::NodeBone> meshPoints(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int& idx,
            const bool updateKin = true);
    std::vector<std::vector<biorbd::rigidbody::Patch>> meshPatch() const;
    const std::vector<biorbd::rigidbody::Patch>& meshPatch(const unsigned int &i) const;
    std::vector<biorbd::rigidbody::Mesh> boneMesh() const;
    const biorbd::rigidbody::Mesh& boneMesh(const unsigned int& idx) const;
    // ----------------------- //


    // -- ANGULAR MOMENTUM FUNCTIONS -- //
    RigidBodyDynamics::Math::Vector3d angularMomentum(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const bool updateKin = true); // Wrapper pour le moment angulaire
    // Réimplémentation de la fonction CalcAngularMomentum car elle a une erreur (inversion du calcul du com)
    RigidBodyDynamics::Math::Vector3d CalcAngularMomentum (
            Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool update_kinematics);
    RigidBodyDynamics::Math::Vector3d CalcAngularMomentum (
            Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool update_kinematics);
    std::vector<RigidBodyDynamics::Math::Vector3d> CalcSegmentsAngularMomentum (
            Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool update_kinematics);
    std::vector<RigidBodyDynamics::Math::Vector3d> CalcSegmentsAngularMomentum (
            Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
            bool update_kinematics);
    // -------------------------------- //

    void CalcMatRotJacobian (
            Joints &model,
            const RigidBodyDynamics::Math::VectorNd &Q,
            unsigned int body_id,
            const RigidBodyDynamics::Math::Matrix3d &rotation,
            RigidBodyDynamics::Math::MatrixNd &G,
            bool update_kinematics); // Calcule la matrice jacobienne d'une matrice de rotation

    void ForwardDynamicsContactsLagrangian (
         Joints &model,
         const RigidBodyDynamics::Math::VectorNd &Q,
         const RigidBodyDynamics::Math::VectorNd &QDot,
         const RigidBodyDynamics::Math::VectorNd &GeneralizedTorque,
         RigidBodyDynamics::ConstraintSet &CS,
         RigidBodyDynamics::Math::VectorNd &QDDot
         );
    void computeQdot(
            Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &QDot,
            biorbd::rigidbody::GeneralizedCoordinates &QDotOut); // Cette fonction retourne la dérivée de Q en fonction de Qdot (Si pas de Quaternion, QDot est directement retourné)

protected:
    std::vector<biorbd::rigidbody::Bone> m_bones; // Toutes les articulations

    biorbd::utils::Integrator * integrator;
    unsigned int m_nbRoot; // Nombre de dof sur le segment racine
    unsigned int m_nDof; // Nombre de degré de liberté total
    unsigned int m_nbQ; // Nombre de q au total
    unsigned int m_nbQdot; // Nombre de qdot au total
    unsigned int m_nbQddot; // Nombre de qddot au total
    unsigned int m_nRotAQuat; // Nombre de segments par quaternion
    bool m_isRootActuated; // If the root segment is controled or not
    bool m_hasExternalForces; // If the model includes external force
    bool m_isKinematicsComputed;
    double m_totalMass; // Masse de tous les corps
    RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(
            Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const unsigned int body_id,
            bool update_kinematics); // Calculate the JCS in global
    std::vector<biorbd::rigidbody::NodeBone> meshPoints(
            const std::vector<biorbd::utils::Attitude>&,
            const unsigned int& idx) const;

};

}}

#endif // BIORBD_RIGIDBODY_JOINTS_H
