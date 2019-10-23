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
class BoneCharacteristics;

///
/// \brief Class for each segment
///
class BIORBD_API Bone : public biorbd::utils::Node
{
public:

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
    /// \param characteristics The mass, the center of mass of the segment, the segment inertia, etc.
    /// \param cor Transformation from parent to child
    /// \param PF Platform index
    ///
    Bone(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName, 
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR,
            const biorbd::rigidbody::BoneCharacteristics& characteristics,
            const RigidBodyDynamics::Math::SpatialTransform& cor,
            int PF = -1);  

    ///
    /// \brief Construct a bone
    /// \param model The model
    /// \param name The name of the segment
    /// \param parentName The name of the parent segment
    /// \param seqR Cardan sequence to classify the dof in rotation
    /// \param characteristics The mass, the center of mass of the segment, the segment inertia, etc.
    /// \param cor Transformation from parent to child
    /// \param PF Platform index
    ///
    Bone(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName, 
            const biorbd::utils::String &seqR, 
            const biorbd::rigidbody::BoneCharacteristics& characteristics, 
            const RigidBodyDynamics::Math::SpatialTransform& cor, 
            int PF = -1); 

    ///
    /// \brief Create a deep copy of Bone
    /// \return Copy of Bone
    ///
    biorbd::rigidbody::Bone DeepCopy() const;

    ///
    /// \brief Deep copy of Bone
    /// \param other The Bone to copy
    ///
    void DeepCopy(const biorbd::rigidbody::Bone& other);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Bone();

    ///
    /// \brief TODO: Returns the bone ID??
    /// \return The bone ID?
    ///
    unsigned int id() const;

    ///
    /// \brief Return the platform index
    /// \return The platform index
    ///
    int platformIdx() const;

    /// 
    /// \brief Return the translation sequence in text
    /// \return The translation sequence in text
    ///
    const biorbd::utils::String& seqT() const;

    ///
    /// \brief Return the angle sequence in text
    /// \return The angle sequence in text
    ///
    const biorbd::utils::String& seqR() const; 

    ///
    /// \brief Return the number of Dof of the segment
    /// \return The number of Dof of the segment
    ///
    unsigned int nbDof() const;

    /// 
    /// \brief Return the number of Dof in translation of the segment
    /// \return The number of Dof in translation of the segment
    ///
    unsigned int nbDofTrans() const;

    ///
    /// \brief Return the number of Dof in rotation of the segment
    /// \return The number of Dof in rotation of the segment
    ///
    unsigned int nbDofRot() const;

    /// 
    /// \brief Return the number of bone position
    /// \return The number bone position
    ///
    unsigned int nbQ() const;
    
    ///
    /// \brief Return the number of bone velocity
    /// \return The number bone velocity
    ///
    unsigned int nbQdot() const; 
    ///
    /// \brief Return the number of bone acceleration
    /// \return The number bone acceleration
    ///
    unsigned int nbQddot() const; 
    ///
    /// \brief Return the  number of generalized torque
    /// \return The number of generalized torque
    ///
    unsigned int nbGeneralizedTorque() const;

    ///
    /// \brief Return the index of a specific Dof for this segment
    /// \return The index of a specific Dof for this segment
    ///
    unsigned int getDofIdx(const biorbd::utils::String &dofName) const; 
    ///
    /// \brief Return the name of the Dof of this segment
    /// \return The name of the Dof of this segment
    ///
    const biorbd::utils::String& nameDof(const unsigned int i) const;
    ///
    /// \brief Return exactly what is written in the file
    /// \return Exactly what is written in the file
    ///
    biorbd::utils::RotoTrans localJCS() const; 


    ///
    /// \brief Return the bone characteristics
    /// \return The bone characteristics
    ///
    const biorbd::rigidbody::BoneCharacteristics& characteristics() const; 

    ///
    /// \brief Return if the rotation of this segment is a quaternion
    /// \return True or false
    ///
    bool isRotationAQuaternion() const;

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
    /// \param model The model
    /// \param seqT Sequence to classify the dof in translation
    /// \param seqR Cardan sequence to classify the dof in rotation
    ///
    void setDofs(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::String &seqT,
            const biorbd::utils::String &seqR); 

    ///
    /// \brief Set the total number of DoF
    /// \param nbTrans Number of DoF in translation
    /// \param nbRot Number of DoF in rotation
    ///
    void setNumberOfDof(
            unsigned int nbTrans,
            unsigned int nbRot);

    std::shared_ptr<biorbd::utils::String> m_seqT;  ///< Translation sequence as written in the file
    std::shared_ptr<biorbd::utils::String> m_seqR;  ///< Rotation sequence as written in the file0
    std::shared_ptr<unsigned int> m_nbDof;   ///< Number of degrees of freedom 
    std::shared_ptr<unsigned int> m_nbQdot;  ///< Number of Qdot
    std::shared_ptr<unsigned int> m_nbQddot;  ///< Number of Qddot
    std::shared_ptr<unsigned int> m_nbDofTrue;    ///< Number of degrees of freedom
    std::shared_ptr<unsigned int> m_nbDofTrueOutside; ///< Number of degree of freedom read from the outside (Same as nDof except if Quaternion)
    std::shared_ptr<unsigned int> m_nbDofTrans; ///< Number of degrees of freedom in translation
    std::shared_ptr<unsigned int> m_nbDofRot; ///< Number of degrees of freedom in rotation
    std::shared_ptr<unsigned int> m_nbDofQuat; ///< Number of degrees of freedom in rotation? TODO: Nombre de degrés de liberté en rotation

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


    std::shared_ptr<std::vector<unsigned int>> m_dofPosition;  ///< Position in the x, y, and z sequence

    ///
    /// \brief Set the DoF characteristics (m_characteristics) on the last segment
    ///
    /// Part of the formal definition of the segment
    ///
    void setDofCharacteristicsOnLastSegment();

    std::shared_ptr<biorbd::rigidbody::BoneCharacteristics> m_characteristics;///< Non-used virtual segment; it allows to "save" the data and to avoid the use of multiple intermediate variables
    std::shared_ptr<std::vector<biorbd::rigidbody::BoneCharacteristics>> m_dofCharacteristics;  ///< Variable containing the inertial data and other from each segment (0 to 4 should be empty and 5 filled)


};

}}

#endif // BIORBD_RIGIDBODY_BONE_H
