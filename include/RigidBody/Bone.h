#ifndef BIORBD_RIGIDBODY_BONE_H
#define BIORBD_RIGIDBODY_BONE_H

#include <vector>
#include <rbdl/Model.h>
#include <rbdl/Joint.h>
#include <rbdl/rbdl_math.h>
#include "biorbdConfig.h"
#include "Utils/Node.h"

/// 
/// \brief Namespace biorbd
///

namespace biorbd {


///
/// \brief Namespace that holds the RotoTrans class that corresponds to the 3d position given in a 4d matrix
///
namespace utils {
class RotoTrans;
}
/// 
/// \brief Namespace rigidbody
///
namespace rigidbody {
class Joints;
class BoneCaracteristics;

///
/// \brief Class for each segment
///
class BIORBD_API Bone : public biorbd::utils::Node
{
public:
    // Constructors
    /// 
    /// \brief Construct a bone
    ///
    Bone();
    
    ///
    /// \brief Construct a bone
    /// \param model The model
    /// \param name The name of the segment
    /// \param parentName The name of the parent segment
    /// \param seqT
    /// \param seqR Cardan sequence to classify the dof in rotation
    /// \param caract The mass, the center of mass of the segment, the segment inertia, etc.
    /// \param cor Transformation from parent to child
    /// \param PF Platform index
    ///
    Bone(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &name, // name of the segment
            const biorbd::utils::String &parentName, // name of the parent segment
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR,// Cardan sequence to classify the dof in rotation
            const biorbd::rigidbody::BoneCaracteristics& caract, // The mass, the center of mass of the segment, the segment inertia, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation from parent to child
            int PF = -1);  // Platform index
    ///
    /// \brief Construct a bone
    /// \param model The model
    /// \param name The name of the segment
    /// \param parentName The name of the parent segment
    /// \param seqR Cardan sequence to classify the dof in rotation
    /// \param caract The mass, the center of mass of the segment, the segment inertia, etc.
    /// \param cor Transformation from parent to child
    /// \param PF Platform index
    ///

    Bone(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &name, // name of the segment
            const biorbd::utils::String &parentName, // noname of the parent segment
            const biorbd::utils::String &seqR, // Cardan sequence to classify the dof in rotation
            const biorbd::rigidbody::BoneCaracteristics& caract, // The mass, the center of mass of the segment, the segment inertia, etc.
            const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation from parent to child
            int PF = -1);  // Platform index

    ///
    /// \brief Deep copy of bone
    /// \return Copy of bone
    ///
    biorbd::rigidbody::Bone DeepCopy() const;

    ///
    /// \brief Deep copy of bone
    /// \param other 
    ///
    void DeepCopy(const biorbd::rigidbody::Bone& other);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Bone();

    ///
    /// \brief Returns the ??
    /// \return
    ///
    unsigned int id() const;

    ///
    /// \brief The platform index
    /// \return The platform index
    ///
    int platformIdx() const;

    /// 
    /// \brief The translation sequence in text
    /// \return The translation sequence in text
    ///
    const biorbd::utils::String& seqT() const;

    ///
    /// \brief The angle sequence in text
    /// \return The angle sequence in text
    ///
    const biorbd::utils::String& seqR() const; 

    ///
    /// \brief The number of Dof of the segment
    /// \return The number of Dof of the segment
    ///
    unsigned int nDof() const;

    /// 
    /// \brief The number of Dof in translation of the segment
    /// \return The number of Dof in translation of the segment
    ///
    unsigned int nDofTrans() const;

    ///
    /// \brief The number of Dof in rotation of the segment
    /// \return The number of Dof in rotation of the segment
    ///
    unsigned int nDofRot() const;

    /// 
    /// \brief The bone position
    /// \return The bone position
    ///
    unsigned int nQ() const; // Retourne le nombre de Dof de ce segment
    
    ///
    /// \brief The bone velocity
    /// \return The bone velocity
    ///
    unsigned int nQdot() const; // Retourne le nombre de Dof de ce segment

    ///
    /// \brief The bone acceleration
    /// \return The bone acceleration
    ///
    unsigned int nQddot() const; // Retourne le nombre de Dof de ce segment
    
    ///
    /// \brief The generalized torque
    /// \return The generalized torque
    ///
    unsigned int nGeneralizedTorque() const;

    ///
    /// \brief The index of a specific Dof for this segment
    /// \return The index of a specific Dof for this segment
    ///
    unsigned int getDofIdx(const biorbd::utils::String &dofName) const; // Retourne l'index d'un dof spéficique pour ce segment

    ///
    /// \brief The name of the Dof of this segment
    /// \return The name of the Dof of this segment
    ///
    const biorbd::utils::String& nameDof(const unsigned int i) const;// Retourne le nom des Dof de ce segment

    ///
    /// \brief Returns exactly what is written in the file
    /// \return Exactly what is written in the file
    ///
    biorbd::utils::RotoTrans localJCS() const; // retourne exactement ce qui est écrit dans le fichier

    ///
    /// \brief The bone characteristics
    /// \return The bone characteristics
    ///
    const biorbd::rigidbody::BoneCaracteristics& caract() const; // Retourne

    ///
    /// \brief Returns if the rotation of this segment is a quaternion
    /// \return True or false
    ///
    bool isRotationAQuaternion() const; // Retourne si la rotation de ce segment est un quaternion

protected:
    ///
    /// \brief Set the type
    ///
    void setType();

    std::shared_ptr<int> m_idxPF; ///< Index de la plateforme sur lequel il est -1 est pas de plateforme

    ///
    /// \brief Set the platform index
    ///
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
    void setDofCaracteristicsOnLastSegment(); // Mettre m_caract sur le dernier segment
    std::shared_ptr<biorbd::rigidbody::BoneCaracteristics> m_caract;// Segment virtuel non utilisé, il permet de "sauvegarder" les données et donc d'éviter l'usage de multiples variables intermédiaires
    std::shared_ptr<std::vector<biorbd::rigidbody::BoneCaracteristics>> m_dofCaract; // Variable contenant les données Inertielles et autre de chaque sous segment (0 à 4 devraient être vide et 5 rempli)

};

}}

#endif // BIORBD_RIGIDBODY_BONE_H
