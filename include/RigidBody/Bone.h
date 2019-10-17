#ifndef BIORBD_RIGIDBODY_BONE_H
#define BIORBD_RIGIDBODY_BONE_H

#include <vector>
#include <rbdl/Model.h>
#include <rbdl/Joint.h>
#include <rbdl/rbdl_math.h>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class RotoTrans;
}

namespace rigidbody {
class Joints;
class BoneCharacteristics;

class BIORBD_API Bone : public biorbd::utils::Node
{
public:
    // Constructeurs
    Bone();
    Bone(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &name, // nom du segment
            const biorbd::utils::String &parentName, // nom du segment parent
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR,// Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::BoneCharacteristics& characteristics, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            int PF = -1);  // Index de la plateforme
    Bone(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &name, // nom du segment
            const biorbd::utils::String &parentName, // nom du segment parent
            const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
            const biorbd::rigidbody::BoneCharacteristics& characteristics, // Mase, Centre de masse du segment, Inertie du segment, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
            int PF = -1);  // Index de la plateforme
    biorbd::rigidbody::Bone DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::Bone& other);
    virtual ~Bone();

    unsigned int id() const;

    int plateformeIdx() const;
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

    biorbd::utils::RotoTrans localJCS() const; // retourne exactement ce qui est écrit dans le fichier

    const biorbd::rigidbody::BoneCharacteristics& characteristics() const; // Retourne
    bool isRotationAQuaternion() const; // Retourne si la rotation de ce segment est un quaternion

protected:
    void setType();

    std::shared_ptr<int> m_idxPF; // Index de la plateforme sur lequel il est -1 est pas de plateforme
    void setPF(int ); // Setter l'index de la plateforme

    // Info sur la relation parent enfant
    std::shared_ptr<RigidBodyDynamics::Math::SpatialTransform> m_cor; // Transformation decrivant la position du segment par rapport à son parent en position neutre

    // DOF
    void setDofs(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR); // Séquence de Cardan pour classer les dof en rotation
    void setNumberOfDof(
            unsigned int ,
            unsigned int ); // Détermine le nombre de DoF Total

    std::shared_ptr<biorbd::utils::String> m_seqT;   // Séquence en translation telle qu'écrite dans le fichier
    std::shared_ptr<biorbd::utils::String> m_seqR;   // Séquence de rotation telle qu'écrite dans le fichier
    std::shared_ptr<unsigned int> m_nDof;    // Nombre de degrés de liberté
    std::shared_ptr<unsigned int> m_nQdot;   // Nombre de Qdot
    std::shared_ptr<unsigned int> m_nQddot;   // Nombre de Qdot
    std::shared_ptr<unsigned int> m_nDofTrue;    // Nombre de degrés de liberté
    std::shared_ptr<unsigned int> m_nDofTrueOutside; // Nombre de degré de liberté lu de l'extérieur (Idem à nDof sauf si Quaternion)
    std::shared_ptr<unsigned int> m_nDofTrans; // Nombre de degrés de liberté en translation
    std::shared_ptr<unsigned int> m_nDofRot; // Nombre de degrés de liberté en rotation
    std::shared_ptr<unsigned int> m_nDofQuat; // Nombre de degrés de liberté en rotation

    std::shared_ptr<bool> m_isQuaternion; // conserver si les dof en rotation est un quaternion
    void determineIfRotIsQuaternion(const biorbd::utils::String &seqR);

    std::shared_ptr<std::vector<RigidBodyDynamics::Joint>> m_dof; // Articulation des dof : t1, t2, t3, r1, r2, r3; selon l'ordre réel des coordonnées généralisées
    std::shared_ptr<std::vector<unsigned int>> m_idxDof;  // Index de l'articulation parent à mettre dans la variable model,
                                // lorsque l'utilisateur demande le parent_id de ce segment, le dernier indice est envoyé

    // Sequence d'angle et de translation
    void setSequence(
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR); // Ajuster la séquence d'angle et redéclarer tout ce qui est nécessaire
    void fillSequence();
    void str2numSequence(
            std::vector<unsigned int> &sequenceInteger,
            const biorbd::utils::String&); // Passage de séquence vers le chiffre correspondant
    void str2numSequence(
            const biorbd::utils::String&,
            const biorbd::utils::String&); // Stockage dans m_sequence des strings en integer
    std::shared_ptr<std::vector<unsigned int>> m_sequenceTrans; // Séquence de translation
    std::shared_ptr<std::vector<unsigned int>> m_sequenceRot; // Séquence de rotation de Cardan ou d'Euler
    std::shared_ptr<std::vector<biorbd::utils::String>> m_nameDof;

    // Définition de l'articulation intra segment
    virtual void setJoints(biorbd::rigidbody::Joints& model); // Déclare tous les joints intrasegments
    virtual void setJointAxis();    // Choisir les axes de rotation en fonction de la séquence demandée
    std::shared_ptr<std::vector<unsigned int>> m_dofPosition; // position dans la séquence de x, y et z

    // Définition formelle du segment
    void setDofCharacteristicsOnLastSegment(); // Mettre m_characteristics sur le dernier segment
    std::shared_ptr<biorbd::rigidbody::BoneCharacteristics> m_characteristics;// Segment virtuel non utilisé, il permet de "sauvegarder" les données et donc d'éviter l'usage de multiples variables intermédiaires
    std::shared_ptr<std::vector<biorbd::rigidbody::BoneCharacteristics>> m_dofCharacteristics; // Variable contenant les données Inertielles et autre de chaque sous segment (0 à 4 devraient être vide et 5 rempli)

};

}}

#endif // BIORBD_RIGIDBODY_BONE_H
