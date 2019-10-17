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
    /// \param seqT Sequence to classify the dof in translation
    /// \param seqR Cardan sequence to classify the dof in rotation
    /// \param caract The mass, the center of mass of the segment, the segment inertia, etc.
    /// \param cor Transformation from parent to child
    /// \param PF Platform index
    ///
    Bone(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName, 
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR,
            const biorbd::rigidbody::BoneCaracteristics& caract, 
            const RigidBodyDynamics::Math::SpatialTransform& cor, 
            int PF = -1); 
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
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName,
            const biorbd::utils::String &seqR, 
            const biorbd::rigidbody::BoneCaracteristics& caract, 
            const RigidBodyDynamics::Math::SpatialTransform& cor, 
            int PF = -1); 

    ///
    /// \brief TODO: Deep copy of bone
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
    /// \brief TODO: Returns the ??
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

    std::shared_ptr<int> m_idxPF; ///< Platform index on which -1 is the platform step

    ///
    /// \brief Set the platform index
    ///
    void setPF(int ); 

    // Information on the parent child relationship
    std::shared_ptr<RigidBodyDynamics::Math::SpatialTransform> m_cor; ///< Transformation representing the segment position in relation to its parent in neutral position


    ///
    /// \brief Set the DoF
    /// \brief model The model
    /// \param seqT Sequence to classify the dof in translation
    /// \param seqR Cardan sequence to classify the dof in rotation
    ///
    void setDofs(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR); 

    ///
    /// \brief Set the total number of DoF
    /// \param nTrans Number of DoF in translation
    /// \param nRot Number of DoF in rotation
    ///
    void setNumberOfDof(
            unsigned int nTrans,
            unsigned int nRot);

    std::shared_ptr<biorbd::utils::String> m_seqT;  ///< Translation sequency as written in the file
    std::shared_ptr<biorbd::utils::String> m_seqR;  ///< Rotation sequence as written in the file0
    std::shared_ptr<unsigned int> m_nDof;   ///< Number of degrees of freedom 
    std::shared_ptr<unsigned int> m_nQdot;  ///< Number of Qdot
    std::shared_ptr<unsigned int> m_nQddot;  ///< Number of Qddot
    std::shared_ptr<unsigned int> m_nDofTrue;    ///< TODO: Number of degrees of freedom TRUE?
    std::shared_ptr<unsigned int> m_nDofTrueOutside; // Number of degree of freedom read from the outside (Same as nDof except if Quaternion)
    std::shared_ptr<unsigned int> m_nDofTrans; ///< Number of degrees of freedom in translation
    std::shared_ptr<unsigned int> m_nDofRot; ///< Number of degrees of freedom in rotation
    std::shared_ptr<unsigned int> m_nDofQuat; ///< Number of degrees of freedom in rotation? TODO: Nombre de degrés de liberté en rotation

    std::shared_ptr<bool> m_isQuaternion; ///< Keep if DoF in rotation is a Quaternion

    ///
    /// \brief Determines if DoF in rotation is a Quaternion
    /// \param seqR Cardan sequence to classify the DoF in rotation
    ///
    void determineIfRotIsQuaternion(const biorbd::utils::String &seqR);

    std::shared_ptr<std::vector<RigidBodyDynamics::Joint>> m_dof; ///< Articulation of the DoF: t1, t2, t3, r1, r2, r3; depending on the real order of the generalized coordinates
    std::shared_ptr<std::vector<unsigned int>> m_idxDof;  ///< Parent articulation index to be included in the model variable; when the user asks for the parent_id of the segment, the last index is returned
                         

 
    ///
    /// \brief Set angle and translation sequence, adjust angle sequence and redeclare what is necessary
    /// \param seqT Sequence to classify the dof in translation
    /// \param seqR Cardan sequence to classify the dof in rotation
    ///
    void setSequence(
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR);

    ///
    /// \brief Fill sequence
    ///
    /// Places the translations first, followed by the rotations in the asked order.
    ///
    void fillSequence();

    ///
    /// \brief Switch the sequence from string to the associated number
    /// \param sequenceInteger TODO: Sequence integer
    /// \param sequenceText 
    ///
    void str2numSequence(
            std::vector<unsigned int> &sequenceInteger,
            const biorbd::utils::String &sequenceText); 

    ///
    /// \brief Store the integer strings in m_sequence
    /// \param seqT Sequence to classify the dof in translation
    /// \param seqR Cardan sequence to classify the dof in rotation
    ///
    void str2numSequence(
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR);

    std::shared_ptr<std::vector<unsigned int>> m_sequenceTrans; ///< Translation sequence
    std::shared_ptr<std::vector<unsigned int>> m_sequenceRot; ///< Cardan or Euler rotation sequence
    std::shared_ptr<std::vector<biorbd::utils::String>> m_nameDof; ///< To store the DoF names

    // Definition of the intra segment articulation
    
    ///
    /// \brief Declare all the intrasegment joints
    /// \param model TODO: The model
    ///
    /// Part of the definition of the intra segment articulation
    ///
    virtual void setJoints(biorbd::rigidbody::Joints& model);

    ///
    /// \brief Determine the rotation axis in relation to the requested sequence
    ///
    /// Part of the definition of the intra segment articulation
    ///
    virtual void setJointAxis();  

    std::shared_ptr<std::vector<unsigned int>> m_dofPosition; ///< Position in the x, y, and z sequence

    // Formal definition of the segment

    ///
    /// \brief Set the DoF caracteristics (m_caract) on the last segment
    ///
    /// Part of the formal definition of the segment
    ///
    void setDofCaracteristicsOnLastSegment(); 

    std::shared_ptr<biorbd::rigidbody::BoneCaracteristics> m_caract;///< Non-used virtual segment; it allows to "save" the data and to avoid the use of multiple intermediate variables

    std::shared_ptr<std::vector<biorbd::rigidbody::BoneCaracteristics>> m_dofCaract; ///< Variable containing the inertial data and other from each segment (0 to 4 should be empty and 5 filled)

};

}}

#endif // BIORBD_RIGIDBODY_BONE_H
