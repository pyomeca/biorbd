#ifndef BIORBD_RIGIDBODY_SEGMENT_H
#define BIORBD_RIGIDBODY_SEGMENT_H

#include <vector>
#include <rbdl/Model.h>
#include <rbdl/Joint.h>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class RotoTrans;
class Range;
}

namespace rigidbody
{
class Joints;
class SegmentCharacteristics;

///
/// \brief Description of a segment
///
class BIORBD_API Segment : public utils::Node
{
public:
    ///
    /// \brief Construct a Segment
    ///
    Segment();

    ///
    /// \brief Construct a Segment
    /// \param model The joint model
    /// \param name The name of the segment
    /// \param parentName The name of the parent segment
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    /// \param characteristics of the segment (mass, center of mass, inertia, etc.)
    /// \param cor Transformation in parent reference frame
    ///
    Segment(
        rigidbody::Joints& model,
        const utils::String &name,
        const utils::String &parentName,
        const utils::String &seqT,
        const utils::String &seqR,
        const std::vector<utils::Range>& QRanges,
        const std::vector<utils::Range>& QDotRanges,
        const std::vector<utils::Range>& QDDotRanges,
        const SegmentCharacteristics& characteristics,
        const RigidBodyDynamics::Math::SpatialTransform& cor);

    ///
    /// \brief Construct a Segment
    /// \param model The joint model
    /// \param name The name of the segment
    /// \param parentName The name of the parent segment
    /// \param seqR Angle sequence of the Euler rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    /// \param characteristics of the segment (mass, center of mass, inertia, etc.)
    /// \param cor Transformation in parent reference frame
    ///
    Segment(
        rigidbody::Joints& model,
        const utils::String &name,
        const utils::String &parentName,
        const utils::String &seqR,
        const std::vector<utils::Range>& QRanges,
        const std::vector<utils::Range>& QDotRanges,
        const std::vector<utils::Range>& QDDotRanges,
        const SegmentCharacteristics& characteristics,
        const RigidBodyDynamics::Math::SpatialTransform& cor);

    ///
    /// \brief Create a deep copy of Segment
    /// \return Copy of Segment
    ///
    Segment DeepCopy() const;

    ///
    /// \brief Deep copy of Segment
    /// \param other The Segment to copy
    ///
    void DeepCopy(
        const Segment& other);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Segment();

    ///
    /// \brief Return the Segment index
    /// \return The Segment index
    ///
    size_t id() const;

    ///
    /// \brief Return the translation sequence in text
    /// \return The translation sequence in text
    ///
    const utils::String& seqT() const;

    ///
    /// \brief Return the angle sequence in text
    /// \return The angle sequence in text
    ///
    const utils::String& seqR() const;

    ///
    /// \brief Return the ranges for all the dof, translations and rotations respectively
    /// \return The ranges for all the dof, translations and rotations respectively
    ///
    const std::vector<utils::Range>&
    QRanges() const;

    ///
    /// \brief Return the ranges for all the dof velocity, translations and rotations respectively
    /// \return The ranges for all the dof velocity, translations and rotations respectively
    ///
    const std::vector<utils::Range>&
    QDotRanges() const;

    ///
    /// \brief Return the ranges for all the dof acceleration, translations and rotations respectively
    /// \return The ranges for all the dofa acceleration, translations and rotations respectively
    ///
    const std::vector<utils::Range>&
    QDDotRanges() const;

    ///
    /// \brief Return the number of DoF of the segment
    /// \return The number of Dof of the segment
    ///
    size_t nbDof() const;

    ///
    /// \brief Return the number of translation DoF of the segment
    /// \return The number of translation DoF of the segment
    ///
    size_t nbDofTrans() const;

    ///
    /// \brief Return the number of rotation DoF of the segment
    /// \return The number of rotation DoF of the segment
    ///
    size_t nbDofRot() const;

    ///
    /// \brief Return the number of generalized coordinates
    /// \return The number of generalized coordinates
    ///
    size_t nbQ() const;

    ///
    /// \brief Return the number of generalized velocities
    /// \return The number of generalized velocities
    ///
    size_t nbQdot() const;

    ///
    /// \brief Return the number of generalized accelerations
    /// \return The number of generalized accelerations
    ///
    size_t nbQddot() const;

    ///
    /// \brief Return the number of generalized torque
    /// \return The number of generalized torque
    ///
    /// This value is equal to nbQddot
    ///
    size_t nbGeneralizedTorque() const;

    ///
    /// \brief Return the index of a specified DoF
    /// \return The index of a specified DoF
    ///
    size_t getDofIdx(
        const utils::String &dofName) const;

    /// 
    /// \brief Traverse the model hierarchy until it gets to a segment with a dof. If the 
    /// current segment has at least one dof, the current segment is returned
    /// \return The first the first segment with a dof in the hierarchy
    const rigidbody::Segment& findFirstSegmentWithDof(
        const rigidbody::Joints& model) const;

    ///
    /// \brief Returns the first index of the segment in the Generalized Coordinates vector
    /// \return The index of the first dof in the Generalized Coordinates vector
    /// 
    /// Warning: if the current segment does not have any dof, it returns the first index of the
    /// parent segment
    size_t getFirstDofIndexInGeneralizedCoordinates(
        const rigidbody::Joints& model) const;

    ///
    /// \brief Returns the last index of the segment in the Generalized Coordinates vector
    /// \return The index of the last dof in the Generalized Coordinates vector
    /// 
    /// Warning: if the current segment does not have any dof, it returns the last index of the
    /// parent segment
    size_t getLastDofIndexInGeneralizedCoordinates(
        const rigidbody::Joints& model) const;

    ///
    /// \brief Return the name of the specified DoF
    /// \return The name of the specified DoF
    ///
    const utils::String& nameDof(
        const size_t i) const;

    ///
    /// \brief Return the joint coordinate system (JCS) in the parent reference frame
    /// \return The joint coordinate system in the parent reference frame
    ///
    utils::RotoTrans localJCS() const;

    ///
    /// \brief set the joint coordinate system (JCS) in the parent reference frame
    /// \param The model, it's need to update the underlying rbdl model
    /// \param rototrans The rototranslation object
    /// 
    /// Warning: This function doesn't work for segments that have no degree of freedom
    /// w.r.t their parent segment.
    ///
    void setLocalJCS(
        rigidbody::Joints& model,
        utils::RotoTrans& rototrans);

    ///
    /// \brief updateCharacteristics Change the inertia characteristics of the segment
    /// \param model The underlying model to update
    /// \param characteristics The new characteristics
    ///
    /// Warning: This function may behave surpringly due to the core of RBDL. The
    /// new characteristic values will replace everything which is attach in a fixed manner
    /// (that is no degrees-of-freedom). So if your model has 3 segments, but only the first
    /// one has dof (and the rest is rigidly attached to the first), then it doesn't matter
    /// if idx is 0, 1 or 2, because RBDL considers that all these segment are 1 segment.
    /// It is therefore expected that characteristics is the combination of mass and
    /// inertia for these 3 segments as well.
    ///
    void updateCharacteristics(
        rigidbody::Joints& model,
        const SegmentCharacteristics& characteristics);

    ///
    /// \brief Return the segment characteristics
    /// \return The segment characteristics
    ///
    const SegmentCharacteristics& characteristics() const;

    ///
    /// \brief Return if the rotation DoF of this segment is a quaternion
    /// \return If the rotation DoF of this segment is a quaternion
    ///
    bool isRotationAQuaternion() const;

protected:
    std::shared_ptr<int> m_idxInModel; ///< Index in RBDL model

    ///
    /// \brief Set the type of the segment
    ///
    void setType();

    std::shared_ptr<RigidBodyDynamics::Math::SpatialTransform>
    m_cor; ///< Attitude of the segment in parent reference frame

    ///
    /// \brief Set the DoF
    /// \param model The joint model
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    ///
    void setDofs(
        rigidbody::Joints& model,
        const utils::String &seqT,
        const utils::String &seqR,
        const std::vector<utils::Range>& QRanges,
        const std::vector<utils::Range>& QDotRanges,
        const std::vector<utils::Range>& QDDotRanges);

    ///
    /// \brief Set the total number of DoF
    /// \param nbTrans Number of translation DoF
    /// \param nbRot Number of rotation DoF
    ///
    void setNumberOfDof(
        size_t nbTrans,
        size_t nbRot);

    std::shared_ptr<utils::String> m_seqT;  ///< Translation sequence
    std::shared_ptr<utils::String> m_seqR;  ///< Euler rotation sequence
    std::shared_ptr<std::vector<utils::Range>> m_QRanges;  ///< Minimum and maximum coordinate values that each dof should hold. This is only prescriptive and can be ignored when setting the GeneralizedCoordinates
    std::shared_ptr<std::vector<utils::Range>> m_QDotRanges;  ///< Minimum and maximum velocity values that each dof should hold. This is only prescriptive and can be ignored when setting the GeneralizedVelocities
    std::shared_ptr<std::vector<utils::Range>> m_QDDotRanges;  ///< Minimum and maximum acceleration values that each dof should hold. This is only prescriptive and can be ignored when setting the GeneralizedAccelerations
    std::shared_ptr<size_t> m_nbDof;   ///< Number of degrees of freedom
    std::shared_ptr<size_t> m_nbQdot;  ///< Number of generalized velocities
    std::shared_ptr<size_t> m_nbQddot;  ///< Number of generalized accelerations
    std::shared_ptr<size_t> m_nbDofTrue;    ///< Number of degrees of freedom including the extra DoF when there is a quaternion
    std::shared_ptr<size_t> m_nbDofTrueOutside; ///< Number of degree of freedom read from the outside (Same as nDof except if Quaternion)
    std::shared_ptr<size_t> m_nbDofTrans; ///< Number of degrees of freedom in translation
    std::shared_ptr<size_t> m_nbDofRot; ///< Number of degrees of freedom in rotation
    std::shared_ptr<size_t> m_nbDofQuat; ///< Number of degrees of freedom in rotation if expressed in quaternion

    std::shared_ptr<bool> m_isQuaternion; ///< If DoF in rotation is a Quaternion

    ///
    /// \brief Determines if DoF in rotation is a Quaternion
    /// \param seqR Cardan sequence to classify the DoF in rotation
    ///
    /// If seqR is equal to "q" then it is a quaternion
    ///
    void determineIfRotIsQuaternion(const utils::String &seqR);

    std::shared_ptr<std::vector<RigidBodyDynamics::Joint>>
            m_dof; ///< Actual DoF: t1, t2, t3, r1, r2, r3; where the order depends on seqT and seqR
    std::shared_ptr<std::vector<size_t>> m_idxDof;  ///< Index of the parent segment

    ///
    /// \brief Set angle and translation sequences, adjust angle sequence and redeclare if is necessary
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    ///
    void setSequence(
        const utils::String &seqT,
        const utils::String &seqR);

    ///
    /// \brief Fill the transation and rotation sequences
    ///
    /// Places the translations first, followed by the rotations in the asked order.
    ///
    void fillSequence();

    ///
    /// \brief Convert a text sequence to its number counterpart (x = 0, y = 1, z = 2, q = 3)
    /// \param sequenceInteger Initialized vector into which the results should be placed in
    /// \param sequenceText The sequence to convert
    ///
    void str2numSequence(
        std::vector<size_t> &sequenceInteger,
        const utils::String &sequenceText);

    ///
    /// \brief Store the sequences
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    ///
    void str2numSequence(
        const utils::String &seqT,
        const utils::String &seqR);

    std::shared_ptr<std::vector<size_t>> m_sequenceTrans; ///< Translation sequence
    std::shared_ptr<std::vector<size_t>> m_sequenceRot; ///< Euler rotation sequence
    std::shared_ptr<std::vector<utils::String>> m_nameDof; ///< To store the DoF names

    ///
    /// \brief Function that adds the segment to the RBDL body set
    /// \param model The joint model
    ///
    virtual void setJoints(
            rigidbody::Joints& model);

    ///
    /// \brief Determine the rotation axis in relation to the requested sequence
    ///
    virtual void setJointAxis();

    std::shared_ptr<std::vector<size_t>> m_dofPosition;  ///< Position in the x, y, and z sequence

    ///
    /// \brief Set the DoF segment characteristics on the last body
    ///
    /// The idea is that since a segment is described by all of its DoF, the inertia
    /// and masses must be put on the last body
    ///
    void setDofCharacteristicsOnLastBody();

    std::shared_ptr<SegmentCharacteristics>
            m_characteristics;///< Non-used virtual segment; it allows to "save" the data and to avoid the use of multiple intermediate variables
    std::shared_ptr<std::vector<SegmentCharacteristics>>
            m_dofCharacteristics;  ///< Variable containing the inertial data and other from each segment (on a 6DoF segment, 0 to 4 should be empty and 5 filled)


};

}
}

#endif // BIORBD_RIGIDBODY_SEGMENT_H
