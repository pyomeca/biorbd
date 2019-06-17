#ifndef S2MJOINTS_H
#define S2MJOINTS_H

    #include "s2mMatrix.h"
    #include "biorbdConfig.h"
    #include "s2mError.h"
    #include <rbdl/rbdl.h>
    #include "s2mBone.h"
    #include "s2mBoneCaracteristics.h"
    #include "s2mIntegrator.h"
    #include "s2mTau.h"
    #include "s2mNodeBone.h"
    #include "s2mMarkers.h"


class s2mAttitude;
class s2mBone;
class s2mBoneCaracteristics;
class s2mTau;
class s2mGenCoord;
class s2mIntegrator;
class s2mMarkers;
class BIORBD_API s2mJoints : public RigidBodyDynamics::Model
{
    public:
        s2mJoints();
        s2mJoints(const s2mJoints&);
        ~s2mJoints();

        // Set and Get
        unsigned int AddBone(const unsigned int &parent_id, // Numéro du parent
                             const s2mString &seqT, const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
                             const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
                             const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
                             const s2mString &name="", // Nom du segment
                             const int &PF=-1); // Numéro de la plateforme de force attaché à cet os
        unsigned int AddBone(const unsigned int &parent_id, // Numéro du parent
                             const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
                             const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
                             const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
                             const s2mString &name="", // Nom du segment
                             const int &PF=-1); // Numéro de la plateforme de force attaché à cet os
        std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector> > &, const unsigned int &frame);
        std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(std::vector<RigidBodyDynamics::Math::SpatialVector> &); // un SpatialVector par PF

        int GetBodyS2MId(const s2mString &) const;
        void computeKinematics(const s2mGenCoord&, const s2mGenCoord&,const s2mTau&); // Process integration (Q, Qdot, effecteurs)
        void kinematics(const unsigned int&, s2mGenCoord&, s2mGenCoord&);  // Put in a VectorNd the Qs a time t
        unsigned int nbInterationStep() const;
        unsigned int nbTau() const;
        unsigned int nbDof() const;
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
        std::vector<s2mAttitude> globalJCS(const s2mGenCoord &, const bool updateKin = true); // Return the JCSs in global coordinate system for the given q
        s2mAttitude globalJCS(const s2mGenCoord &Q, const s2mString &parentName, const bool updateKin = true);  // Return the JCS for segment i in global coordinate system for the given q
        s2mAttitude globalJCS(const s2mGenCoord &Q, const unsigned int i, const bool updateKin = true);  // Return the JCS for segment i in global coordinate system for the given q
        std::vector<s2mAttitude> localJCS() const; // Return the JCSs in global coordinate system for the given q
        s2mAttitude localJCS(const s2mString &) const;  // Return the JCS for segment named s2mString in parent coordinate system
        s2mAttitude localJCS(const unsigned int i) const;  // Return the JCS for segment i in parent coordinate system
        s2mNodeBone projectPoint(const s2mGenCoord &Q, const s2mNodeBone&, bool updateKin=true); // Projeter selon les axes/plan déterminé déjà dans nodeBone
        s2mNodeBone projectPoint(const s2mGenCoord &Q, const Eigen::Vector3d &v, int boneIdx, const s2mString& axesToRemove, bool updateKin=true); // Projeter un point dans le repère global
        std::vector<s2mNodeBone>  projectPoint(const s2mMarkers &marks, const s2mGenCoord &Q, const std::vector<Eigen::Vector3d> &v, bool updateKin=true); //Marqueurs projetés de points correspondant aux marqueurs du modèle (le vector doit être égal au nombre de marqueur et dans l'ordre donné par Tags)
        s2mMatrix projectPointJacobian(s2mJoints& model, const s2mGenCoord &Q, s2mNodeBone p, bool updateKin);
        s2mMatrix projectPointJacobian(s2mJoints& model, const s2mGenCoord &Q, const Eigen::Vector3d &v, int boneIdx, const s2mString& axesToRemove, bool updateKin);
        std::vector<s2mMatrix> projectPointJacobian(s2mJoints& model, const s2mMarkers &marks, const s2mGenCoord &Q, const std::vector<Eigen::Vector3d> &v, bool updateKin); // Matrice jacobienne des marqueurs projetés de points correspondant aux marqueurs du modèle (le vector doit être égal au nombre de marqueur et dans l'ordre donné par Tags et dans le repère global)


        unsigned int nbBone() const; // Return the actual number of segments
        double mass() const; // retourne la masse de tous les segments
        s2mNode CoM(const s2mGenCoord &Q); // Position du centre de masse
        std::vector<s2mNodeBone> CoMbySegment(const s2mGenCoord &Q, bool updateKin=true); // Position du centre de masse de chaque segment
        s2mNodeBone CoMbySegment(const s2mGenCoord &Q, const unsigned int i, bool updateKin=true); // Position du centre de masse du segment i
        RigidBodyDynamics::Math::Vector3d CoMdot(const s2mGenCoord &Q, const s2mGenCoord &Qdot); // Vitesse du CoM
        RigidBodyDynamics::Math::Vector3d CoMddot(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot); // Acceleration du CoM
        std::vector<RigidBodyDynamics::Math::Vector3d> CoMdotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, bool updateKin=true); // vitesse du centre de masse de chaque segment
        RigidBodyDynamics::Math::Vector3d CoMdotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const unsigned int i, bool updateKin=true); // vitesse du centre de masse du segment i
        std::vector<RigidBodyDynamics::Math::Vector3d> CoMddotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot, bool updateKin=true); // accélération du centre de masse de chaque segment
        RigidBodyDynamics::Math::Vector3d CoMddotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot, const unsigned int i, bool updateKin=true); // accélération du centre de masse du segment i
        s2mMatrix CoMJacobian(const s2mGenCoord &Q); // Jacobienne
        RigidBodyDynamics::Math::Vector3d angularMomentum(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const bool updateKin = true); // Wrapper pour le moment angulaire
        std::vector<std::vector<s2mNodeBone> > meshPoints(const s2mGenCoord &Q, const bool updateKin = true);
        std::vector<s2mNodeBone> meshPoints(const s2mGenCoord &Q, const unsigned int& idx, const bool updateKin = true);
        std::vector<std::vector<s2mPatch> > meshPatch();
        std::vector<s2mPatch> meshPatch(const unsigned int &i);
        std::vector<s2mBoneMesh> boneMesh();
        s2mBoneMesh boneMesh(const unsigned int& idx);
        // Réimplémentation de la fonction CalcAngularMomentum car elle a une erreur (inversion du calcul du com)
        RigidBodyDynamics::Math::Vector3d CalcAngularMomentum (s2mJoints &model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, bool update_kinematics);
        RigidBodyDynamics::Math::Vector3d CalcAngularMomentum (s2mJoints &model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot, bool update_kinematics);
        std::vector<RigidBodyDynamics::Math::Vector3d> CalcSegmentsAngularMomentum (s2mJoints &model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, bool update_kinematics);
        std::vector<RigidBodyDynamics::Math::Vector3d> CalcSegmentsAngularMomentum (s2mJoints &model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot, bool update_kinematics);
        unsigned int getDofIndex(const s2mString& boneName, const s2mString& dofName);

        void CalcMatRotJacobian (s2mJoints &model, const RigidBodyDynamics::Math::VectorNd &Q, unsigned int body_id, const RigidBodyDynamics::Math::Matrix3d &rotation, RigidBodyDynamics::Math::MatrixNd &G, bool update_kinematics); // Calcule la matrice jacobienne d'une matrice de rotation

        const s2mBone& bone(unsigned int i) const;
        const s2mBone& bone(const s2mString&) const;
        void ForwardDynamicsContactsLagrangian (
             s2mJoints &model,
             const RigidBodyDynamics::Math::VectorNd &Q,
             const RigidBodyDynamics::Math::VectorNd &QDot,
             const RigidBodyDynamics::Math::VectorNd &Tau,
             RigidBodyDynamics::ConstraintSet &CS,
             RigidBodyDynamics::Math::VectorNd &QDDot
             );
        void computeQdot(s2mJoints &model, const s2mGenCoord &Q, const s2mGenCoord &QDot, s2mGenCoord &QDotOut); // Cette fonction retourne la dérivée de Q en fonction de Qdot (Si pas de Quaternion, QDot est directement retourné)

    protected:
        std::vector<s2mBone> m_bones; // Toutes les articulations

        s2mIntegrator * integrator;
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
        RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(s2mJoints &model,
                                                           const s2mGenCoord &Q,
                                                           const unsigned int body_id,
                                                           bool update_kinematics); // Calculate the JCS in global
        std::vector<s2mNodeBone> meshPoints(const std::vector<s2mAttitude>&, const unsigned int& idx);
    private:

};

#endif // S2MJOINTS_H


