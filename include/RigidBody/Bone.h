#ifndef BIORBD_RIGIDBODY_BONE_H
#define BIORBD_RIGIDBODY_BONE_H

#include "biorbdConfig.h"
#include "RigidBody/BoneCaracteristics.h"
#include "RigidBody/Joint.h"

namespace biorbd {
namespace utils {
class Attitude;
}

namespace rigidbody {
class Joints;

class BIORBD_API Bone
{
public:
    // Constructeurs
    Bone(
            biorbd::rigidbody::Joints *model,
            const unsigned int &parent_id,
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR,// Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::Caracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            const biorbd::utils::String &name = "", // nom du segment
            const int &PF = -1);  // Index de la plateforme
    Bone(
            biorbd::rigidbody::Joints *model,
            const unsigned int &parent_id, // Assume no translation
            const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::Caracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            const biorbd::utils::String &name = "", // nom du segment
            const int &PF = -1);  // Index de la plateforme
    Bone(const Bone&);
    virtual ~Bone();

    unsigned int id() const;
    unsigned int parent_rbdl_id() const;
    biorbd::utils::String parentName(const biorbd::rigidbody::Joints &model) const;
    int plateformeIdx() const;
    const biorbd::utils::String& name() const; // Retourne le nom du segment
    const biorbd::utils::String& seqT() const; // Retourne la séquence de translation en texte
    const biorbd::utils::String& seqR() const; // Retourne la séquence d'angle en texte
    unsigned int nDof() const; // Retourne le nombre de Dof de ce segment
    unsigned int nDofTrans() const; // Retourne le nombre de Dof de ce segment
    unsigned int nDofRot() const; // Retourne le nombre de Dof de ce segment
    unsigned int nQ() const; // Retourne le nombre de Dof de ce segment
    unsigned int nQdot() const; // Retourne le nombre de Dof de ce segment
    unsigned int nQddot() const; // Retourne le nombre de Dof de ce segment
    unsigned int nGeneralizedTorque() const;
    unsigned int getDofIdx(const biorbd::utils::String &dofName) const; // Retourne l'index d'un dof spéficique pour ce segment
    const biorbd::utils::String& nameDof(const unsigned int i) const;// Retourne le nom des Dof de ce segment
    biorbd::utils::Attitude localJCS() const; // retourne exactement ce qui est écrit dans le fichier
    const biorbd::rigidbody::Caracteristics& caract() const; // Retourne
    bool isRotationAQuaternion() const; // Retourne si la rotation de ce segment est un quaternion

protected:
    biorbd::utils::String m_name; // Nom du segment
    unsigned int m_parent_id; // Numéro du segment parent
    int m_idxPF; // Index de la plateforme sur lequel il est -1 est pas de plateforme
    void setPF(const int &); // Setter l'index de la plateforme

    // Info sur la relation parent enfant
    void setParentToChildTransformation(const RigidBodyDynamics::Math::SpatialTransform&);
    RigidBodyDynamics::Math::SpatialTransform m_cor; // Transformation decrivant la position du segment par rapport à son parent en position neutre

    // DOF
    virtual void setDofs(
            biorbd::rigidbody::Joints *model,
            const unsigned int &parent_id,
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR,// Séquence de Cardan pour classer les dof en rotation
            const double &mass, // Masse du segment
            const RigidBodyDynamics::Math::Vector3d &com,   // Centre de masse du segment
            const RigidBodyDynamics::Math::Matrix3d &inertia);  // Inertie du segment
    virtual void setDofs(
            biorbd::rigidbody::Joints *model,
            const unsigned int &parent_id,
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR,// Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::Caracteristics&);  // Inertie du segment
    virtual void setNumberOfDof(
            const unsigned int&,
            const unsigned int&); // Détermine le nombre de DoF Total
    biorbd::utils::String m_seqT;   // Séquence en translation telle qu'écrite dans le fichier
    biorbd::utils::String m_seqR;   // Séquence de rotation telle qu'écrite dans le fichier
    unsigned int m_nDof;    // Nombre de degrés de liberté
    unsigned int m_nQdot;   // Nombre de Qdot
    unsigned int m_nQddot;   // Nombre de Qdot
    unsigned int m_nDofTrue;    // Nombre de degrés de liberté
    unsigned int m_nDofTrueOutside; // Nombre de degré de liberté lu de l'extérieur (Idem à nDof sauf si Quaternion)
    unsigned int m_nDofTrans; // Nombre de degrés de liberté en translation
    unsigned int m_nDofRot; // Nombre de degrés de liberté en rotation
    unsigned int m_nDofQuat; // Nombre de degrés de liberté en rotation
    bool m_isQuaternion; // conserver si les dof en rotation est un quaternion
    void determineIfRotIsQuaternion(const biorbd::utils::String &seqR);
    biorbd::rigidbody::Joint * m_dof; // Articulation des dof : t1, t2, t3, r1, r2, r3; selon l'ordre réel des coordonnées généralisées
    unsigned int * m_idxDof;  // Index de l'articulation parent à mettre dans la variable model,
                                // lorsque l'utilisateur demande le parent_id de ce segment, le dernier indice est envoyé

    // Sequence d'angle et de translation
    void setSequence(
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR); // Ajuster la séquence d'angle et redéclarer tout ce qui est nécessaire
    void fillSequence();
    void str2numSequence(
            unsigned int*,
            const biorbd::utils::String&); // Passage de séquence vers le chiffre correspondant
    void str2numSequence(
            const biorbd::utils::String&,
            const biorbd::utils::String&); // Stockage dans m_sequence des strings en integer
    unsigned int * m_sequenceTrans; // Séquence de translation
    unsigned int * m_sequenceRot; // Séquence de rotation de Cardan ou d'Euler
    biorbd::utils::String * m_nomDof;

    // Définition de l'articulation intra segment
    virtual void setJoints(biorbd::rigidbody::Joints *model); // Déclare tous les joints intrasegments
    virtual void setJointAxis();    // Choisir les axes de rotation en fonction de la séquence demandée
    unsigned int * m_dofPosition; // position dans la séquence de x, y et z

    // Définition formelle du segment
    void setDofCaracteristicsOnLastSegment(); // Mettre m_caract sur le dernier segment
    biorbd::rigidbody::Caracteristics m_caract;// Segment virtuel non utilisé, il permet de "sauvegarder" les données et donc d'éviter l'usage de multiples variables intermédiaires
    biorbd::rigidbody::Caracteristics * m_dofCaract; // Variable contenant les données Inertielles et autre de chaque sous segment (0 à 4 devraient être vide et 5 rempli)

};

}}

#endif // BIORBD_RIGIDBODY_BONE_H
