#ifndef S2MBONE_H
#define S2MBONE_H

#include "biorbdConfig.h"
#include "s2mBoneCaracteristics.h"
#include "s2mJoint.h"

class s2mJoints;
class s2mString;
class s2mAttitude;
class BIORBD_API s2mBone
{
    public:
        // Constructeurs
        s2mBone(
            s2mJoints *model, const unsigned int &parent_id,
            const s2mString &seqT, const s2mString &seqR,// Séquence de Cardan pour classer les dof en rotation
            const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            const s2mString &name = "", // nom du segment
            const int &PF = -1);  // Index de la plateforme
        s2mBone(s2mJoints *model, const unsigned int &parent_id, // Assume no translation
            const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
            const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            const s2mString &name = "", // nom du segment
            const int &PF = -1);  // Index de la plateforme
        s2mBone(const s2mBone&);
        virtual ~s2mBone();

        unsigned int id() const;
        unsigned int parent_rbdl_id() const;
        s2mString parentName(const s2mJoints &model) const;
        int plateformeIdx() const;
        const s2mString& name() const; // Retourne le nom du segment
        const s2mString& seqT() const; // Retourne la séquence de translation en texte
        const s2mString& seqR() const; // Retourne la séquence d'angle en texte
        unsigned int nDof() const; // Retourne le nombre de Dof de ce segment
        unsigned int nDofTrans() const; // Retourne le nombre de Dof de ce segment
        unsigned int nDofRot() const; // Retourne le nombre de Dof de ce segment
        unsigned int nQ() const; // Retourne le nombre de Dof de ce segment
        unsigned int nQdot() const; // Retourne le nombre de Dof de ce segment
        unsigned int nQddot() const; // Retourne le nombre de Dof de ce segment
        unsigned int nTau() const;
        unsigned int getDofIdx(const s2mString &dofName) const; // Retourne l'index d'un dof spéficique pour ce segment
        const s2mString& nameDof(const unsigned int i) const;// Retourne le nom des Dof de ce segment
        s2mAttitude localJCS() const; // retourne exactement ce qui est écrit dans le fichier
        const s2mBoneCaracteristics& caract() const; // Retourne
        bool isRotationAQuaternion() const; // Retourne si la rotation de ce segment est un quaternion

    protected:
        s2mString m_name; // Nom du segment
        unsigned int m_parent_id; // Numéro du segment parent
        int m_idxPF; // Index de la plateforme sur lequel il est -1 est pas de plateforme
        void setPF(const int &); // Setter l'index de la plateforme

        // Info sur la relation parent enfant
        void setParentToChildTransformation(const RigidBodyDynamics::Math::SpatialTransform&);
        RigidBodyDynamics::Math::SpatialTransform m_cor; // Transformation decrivant la position du segment par rapport à son parent en position neutre

        // DOF
        virtual void setDofs(
            s2mJoints *model, const unsigned int &parent_id,
            const s2mString &seqT, const s2mString &seqR,// Séquence de Cardan pour classer les dof en rotation
            const double &mass, // Masse du segment
            const RigidBodyDynamics::Math::Vector3d &com,   // Centre de masse du segment
            const RigidBodyDynamics::Math::Matrix3d &inertia);  // Inertie du segment
        virtual void setDofs(
            s2mJoints *model, const unsigned int &parent_id,
            const s2mString &seqT, const s2mString &seqR,// Séquence de Cardan pour classer les dof en rotation
            const s2mBoneCaracteristics&);  // Inertie du segment
        virtual void setNumberOfDof(const unsigned int&, const unsigned int&); // Détermine le nombre de DoF Total
        s2mString m_seqT;   // Séquence en translation telle qu'écrite dans le fichier
        s2mString m_seqR;   // Séquence de rotation telle qu'écrite dans le fichier
        unsigned int m_nDof;    // Nombre de degrés de liberté
        unsigned int m_nQdot;   // Nombre de Qdot
        unsigned int m_nQddot;   // Nombre de Qdot
        unsigned int m_nDofTrue;    // Nombre de degrés de liberté
        unsigned int m_nDofTrueOutside; // Nombre de degré de liberté lu de l'extérieur (Idem à nDof sauf si Quaternion)
        unsigned int m_nDofTrans; // Nombre de degrés de liberté en translation
        unsigned int m_nDofRot; // Nombre de degrés de liberté en rotation
        unsigned int m_nDofQuat; // Nombre de degrés de liberté en rotation
        bool m_isQuaternion; // conserver si les dof en rotation est un quaternion
        void determineIfRotIsQuaternion(const s2mString &seqR);
        s2mJoint * m_dof; // Articulation des dof : t1, t2, t3, r1, r2, r3; selon l'ordre réel des coordonnées généralisées
        unsigned int * m_idxDof;  // Index de l'articulation parent à mettre dans la variable model,
                                    // lorsque l'utilisateur demande le parent_id de ce segment, le dernier indice est envoyé

        // Sequence d'angle et de translation
        void setSequence(const s2mString &seqT, const s2mString &seqR); // Ajuster la séquence d'angle et redéclarer tout ce qui est nécessaire
        void fillSequence();
        void str2numSequence(unsigned int*, const s2mString&); // Passage de séquence vers le chiffre correspondant
        void str2numSequence(const s2mString&, const s2mString&); // Stockage dans m_sequence des strings en integer
        unsigned int * m_sequenceTrans; // Séquence de translation
        unsigned int * m_sequenceRot; // Séquence de rotation de Cardan ou d'Euler
        s2mString * m_nomDof;

        // Définition de l'articulation intra segment
        virtual void setJoints(s2mJoints *model); // Déclare tous les joints intrasegments
        virtual void setJointAxis();    // Choisir les axes de rotation en fonction de la séquence demandée
        unsigned int * m_dofPosition; // position dans la séquence de x, y et z

        // Définition formelle du segment
        void setDofCaracteristicsOnLastSegment(); // Mettre m_caract sur le dernier segment
        s2mBoneCaracteristics m_caract;// Segment virtuel non utilisé, il permet de "sauvegarder" les données et donc d'éviter l'usage de multiples variables intermédiaires
        s2mBoneCaracteristics * m_dofCaract; // Variable contenant les données Inertielles et autre de chaque sous segment (0 à 4 devraient être vide et 5 rempli)

};

#endif // S2MBONE_H
