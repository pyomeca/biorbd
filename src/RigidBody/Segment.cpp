#define BIORBD_API_EXPORTS
#include "RigidBody/Segment.h"

#include <limits.h>
#include "Utils/String.h"
#include "Utils/Error.h"
#include "Utils/Matrix3d.h"
#include "Utils/Vector3d.h"
#include "Utils/RotoTrans.h"
#include "Utils/Rotation.h"
#include "Utils/SpatialTransform.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "Utils/Range.h"

using namespace BIORBD_NAMESPACE;

rigidbody::Segment::Segment() :
    utils::Node(),
    m_idxInModel(std::make_shared<int>(-1)),
    m_cor(std::make_shared<utils::SpatialTransform>()),
    m_seqT(std::make_shared<utils::String>()),
    m_seqR(std::make_shared<utils::String>()),
    m_nbDof(std::make_shared<size_t>(0)),
    m_nbQdot(std::make_shared<size_t>(0)),
    m_nbQddot(std::make_shared<size_t>(0)),
    m_nbDofTrue(std::make_shared<size_t>(0)),
    m_nbDofTrueOutside(std::make_shared<size_t>(0)),
    m_nbDofTrans(std::make_shared<size_t>(0)),
    m_nbDofRot(std::make_shared<size_t>(0)),
    m_nbDofQuat(std::make_shared<size_t>(0)),
    m_QRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QdotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QddotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_jointDampings(std::make_shared<std::vector<utils::Scalar>>()),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<size_t>>()),
    m_sequenceTrans(std::make_shared<std::vector<size_t>>()),
    m_sequenceRot(std::make_shared<std::vector<size_t>>()),
    m_nameDof(std::make_shared<std::vector<utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<size_t>>()),
    m_characteristics(
        std::make_shared<rigidbody::SegmentCharacteristics>()),
    m_dofCharacteristics(
        std::make_shared<std::vector<rigidbody::SegmentCharacteristics>>())
{
    setType();
}

rigidbody::Segment::Segment(
    rigidbody::Joints& model,
    const utils::String &name,
    const utils::String &parentName,
    const utils::String &seqT,
    const utils::String &seqR,
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QdotRanges,
    const std::vector<utils::Range>& QddotRanges,
    const std::vector<utils::Scalar>& jointDampings,
    const rigidbody::SegmentCharacteristics& characteristics,
    const utils::SpatialTransform& cor
) :
    utils::Node(name, parentName),
    m_idxInModel(std::make_shared<int>(-1)),
    m_cor(std::make_shared<utils::SpatialTransform>(cor)),
    m_seqT(std::make_shared<utils::String>(seqT)),
    m_seqR(std::make_shared<utils::String>(seqR)),
    m_nbDof(std::make_shared<size_t>(0)),
    m_nbQdot(std::make_shared<size_t>(0)),
    m_nbQddot(std::make_shared<size_t>(0)),
    m_nbDofTrue(std::make_shared<size_t>(0)),
    m_nbDofTrueOutside(std::make_shared<size_t>(0)),
    m_nbDofTrans(std::make_shared<size_t>(0)),
    m_nbDofRot(std::make_shared<size_t>(0)),
    m_nbDofQuat(std::make_shared<size_t>(0)),
    m_QRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QdotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QddotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_jointDampings(std::make_shared<std::vector<utils::Scalar>>()),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<size_t>>()),
    m_sequenceTrans(std::make_shared<std::vector<size_t>>()),
    m_sequenceRot(std::make_shared<std::vector<size_t>>()),
    m_nameDof(std::make_shared<std::vector<utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<size_t>>()),
    m_characteristics(std::make_shared<rigidbody::SegmentCharacteristics>(characteristics)),
    m_dofCharacteristics(std::make_shared<std::vector<rigidbody::SegmentCharacteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, seqT, seqR, QRanges, QdotRanges, QddotRanges, jointDampings);
}
rigidbody::Segment::Segment(
    rigidbody::Joints& model,
    const utils::String &name, // Name of segment
    const utils::String &parentName, // Name of segment
    const utils::String
    &seqR, // Cardan sequence to classify the rotation DoF
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QdotRanges,
    const std::vector<utils::Range>& QddotRanges,
    const std::vector<utils::Scalar>& jointDampings,
    const rigidbody::SegmentCharacteristics&
    characteristics, // Mass, Center of mass of segment, Inertia of segment, etc.
    const utils::SpatialTransform&
    cor //  Transformation from parent to child
 ): utils::Node(name, parentName),
    m_idxInModel(std::make_shared<int>(-1)),
    m_cor(std::make_shared<utils::SpatialTransform>(cor)),
    m_seqT(std::make_shared<utils::String>()),
    m_seqR(std::make_shared<utils::String>(seqR)),
    m_nbDof(std::make_shared<size_t>(0)),
    m_nbQdot(std::make_shared<size_t>(0)),
    m_nbQddot(std::make_shared<size_t>(0)),
    m_nbDofTrue(std::make_shared<size_t>(0)),
    m_nbDofTrueOutside(std::make_shared<size_t>(0)),
    m_nbDofTrans(std::make_shared<size_t>(0)),
    m_nbDofRot(std::make_shared<size_t>(0)),
    m_nbDofQuat(std::make_shared<size_t>(0)),
    m_QRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QdotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QddotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_jointDampings(std::make_shared<std::vector<utils::Scalar>>()),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<size_t>>()),
    m_sequenceTrans(std::make_shared<std::vector<size_t>>()),
    m_sequenceRot(std::make_shared<std::vector<size_t>>()),
    m_nameDof(std::make_shared<std::vector<utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<size_t>>()),
    m_characteristics(std::make_shared<rigidbody::SegmentCharacteristics>(characteristics)),
    m_dofCharacteristics(std::make_shared<std::vector<rigidbody::SegmentCharacteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, "", seqR, QRanges, QdotRanges, QddotRanges, jointDampings);
}

rigidbody::Segment rigidbody::Segment::DeepCopy() const
{
    rigidbody::Segment copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::Segment::DeepCopy(const
        rigidbody::Segment&other)
{
    utils::Node::DeepCopy(other);
    *m_idxInModel = *other.m_idxInModel;
    *m_cor = *other.m_cor;
    *m_seqT = *other.m_seqT;
    *m_seqR = *other.m_seqR;
    *m_nbDof = *other.m_nbDof;
    *m_nbQdot = *other.m_nbQdot;
    *m_nbQddot = *other.m_nbQddot;
    *m_nbDofTrue = *other.m_nbDofTrue;
    *m_nbDofTrueOutside = *other.m_nbDofTrueOutside;
    *m_nbDofTrans = *other.m_nbDofTrans;
    *m_nbDofRot = *other.m_nbDofRot;
    *m_nbDofQuat = *other.m_nbDofQuat;
    *m_QRanges = *other.m_QRanges;
    *m_QdotRanges = *other.m_QdotRanges;
    *m_QddotRanges = *other.m_QddotRanges;
    *m_jointDampings = *other.m_jointDampings;
    *m_isQuaternion = *other.m_isQuaternion;
    m_dof->resize(other.m_dof->size());
    for (size_t i=0; i<other.m_dof->size(); ++i) {
        (*m_dof)[i] = (*other.m_dof)[i];
    }
    *m_idxDof = *other.m_idxDof;
    *m_sequenceTrans = *other.m_sequenceTrans;
    *m_sequenceRot = *other.m_sequenceRot;
    m_nameDof->resize(other.m_nameDof->size());
    for (size_t i=0; i<other.m_nameDof->size(); ++i) {
        (*m_nameDof)[i] = (*other.m_nameDof)[i];
    }
    *m_dofPosition = *other.m_dofPosition;
    *m_characteristics = other.m_characteristics->DeepCopy();
    m_dofCharacteristics->resize(other.m_dofCharacteristics->size());
    for (size_t i=0; i<other.m_dofCharacteristics->size(); ++i) {
        (*m_dofCharacteristics)[i] = (*other.m_dofCharacteristics)[i].DeepCopy();
    }
}

rigidbody::Segment::~Segment()
{

}

bool rigidbody::Segment::isRotationAQuaternion() const
{
    return *m_isQuaternion;
}

void rigidbody::Segment::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::SEGMENT;
}

size_t rigidbody::Segment::id() const
{
    if (*m_nbDof!=0) {
        return (*m_idxDof)[*m_nbDof-1];
    } else {
        return (*m_idxDof)[*m_nbDof];
    }
}

size_t rigidbody::Segment::nbGeneralizedTorque() const
{
    return nbQddot();
}
size_t rigidbody::Segment::nbDof() const
{
    return *m_nbDofTrueOutside;
}
size_t rigidbody::Segment::nbDofTrans() const
{
    return *m_nbDofTrans;
}
size_t rigidbody::Segment::nbDofRot() const
{
    return *m_nbDofRot;
}
size_t rigidbody::Segment::nbQ() const
{
    return *m_nbDofTrue;
}
size_t rigidbody::Segment::nbQdot() const
{
    return *m_nbQdot;
}
size_t rigidbody::Segment::nbQddot() const
{
    return *m_nbQddot;
}

const utils::String &rigidbody::Segment::nameDof(
    const size_t i) const
{
    // Return the number of DoF of the segment
    utils::Error::check(i<*m_nbDofTrue, "Dof ouside N dof for this segment");
    return (*m_nameDof)[i];
}

const utils::String& rigidbody::Segment::seqT() const
{
    return *m_seqT;
}

const utils::String& rigidbody::Segment::seqR() const
{
    return *m_seqR;
}

const std::vector<utils::Range>& rigidbody::Segment::QRanges() const
{
    return *m_QRanges;
}

void rigidbody::Segment::setQRanges(
    const std::vector<utils::Range>& QRanges)
{
    utils::Error::check(isVectorHasDofDimension(QRanges, true), "QRanges must be empty or of the same size as the number of dof");
    *m_QRanges = QRanges;
}

const std::vector<utils::Range>& rigidbody::Segment::QdotRanges() const
{
    return *m_QdotRanges;
}

void rigidbody::Segment::setQdotRanges(
    const std::vector<utils::Range>& QdotRanges)
{
    utils::Error::check(isVectorHasDofDimension(QdotRanges, false), "QdotRanges must be empty or of the same size as the number of dof");
    *m_QdotRanges = QdotRanges;
}

const std::vector<utils::Range>& rigidbody::Segment::QddotRanges() const
{
    return *m_QddotRanges;
}

void rigidbody::Segment::setQddotRanges(
    const std::vector<utils::Range>& QddotRanges)
{
    utils::Error::check(isVectorHasDofDimension(QddotRanges, false), "QddotRanges must be empty or of the same size as the number of dof");
    *m_QddotRanges = QddotRanges;
}

const std::vector<utils::Scalar>& rigidbody::Segment::jointDampings() const
{
    return *m_jointDampings;
}

void rigidbody::Segment::setJointDampings(
    const std::vector<utils::Scalar>& jointDampings)
{
    utils::Error::check(isVectorHasDofDimension(jointDampings, false), "jointDampings must be empty or of the same size as the number of dof");
    
    if (jointDampings.size() == 0) {
        int nTrans = *m_nbDofTrans;
        int nRot = *m_isQuaternion ? 3 : *m_nbDofRot;
        *m_jointDampings = std::vector<utils::Scalar>(nTrans + nRot, 0);
    } else {
        *m_jointDampings = jointDampings;
    }
}

utils::RotoTrans rigidbody::Segment::localJCS() const
{
    return utils::SpatialTransform(m_cor->rotation().transpose(), m_cor->translation());
}

void  rigidbody::Segment::setLocalJCS(rigidbody::Joints& model, utils::RotoTrans &rototrans)
{
    *m_cor = utils::SpatialTransform(rototrans.rot().transpose(), rototrans.trans());
    // we also modify RBDL spatial transform from parent to child
    model.X_T[*m_idxDof->begin()] = *m_cor;
}


void rigidbody::Segment::updateCharacteristics(
    rigidbody::Joints& model,
    const rigidbody::SegmentCharacteristics& characteristics)
{

    *m_characteristics = characteristics.DeepCopy();
    RigidBodyDynamics::Math::SpatialRigidBodyInertia rbi = RigidBodyDynamics::Math::SpatialRigidBodyInertia::createFromMassComInertiaC (
            m_characteristics->mMass, m_characteristics->mCenterOfMass, m_characteristics->mInertia);

    model.Ic[*m_idxInModel] = rbi;
    model.I[*m_idxInModel] = rbi;
}

const rigidbody::SegmentCharacteristics&
rigidbody::Segment::characteristics() const
{
    return *m_characteristics;
}

void rigidbody::Segment::setDofs(
    rigidbody::Joints& model,
    const utils::String &seqT,
    const utils::String &seqR,
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QdotRanges,
    const std::vector<utils::Range>& QddotRanges,
    const std::vector<utils::Scalar>& jointDampings)
{
    determineIfRotIsQuaternion(seqR);
    setSequence(seqT, seqR);

    // Control for the size of dofRange then copy it
    size_t nRot(seqR.length());
    if (*m_isQuaternion) {
        nRot = 0;
    }

    setQRanges(QRanges);
    setQdotRanges(QdotRanges);
    setQddotRanges(QddotRanges);

    setJointDampings(jointDampings);

    setJoints(model);
}

template<typename T>
bool rigidbody::Segment::isVectorHasDofDimension(
    const std::vector<T>& vec, 
    bool isQLevel) const 
{
    int nTrans = *m_nbDofTrans;
    int nRot = *m_isQuaternion ? (isQLevel ? 4 : 3) : *m_nbDofRot;
    return vec.size() == 0 || vec.size() == nTrans + nRot;
}


void rigidbody::Segment::determineIfRotIsQuaternion(
    const utils::String &seqR)
{
    if (!seqR.tolower().compare("q")) {
        *m_isQuaternion = true;
    }
}


// Member functions
void rigidbody::Segment::str2numSequence(
    const utils::String &seqT,
    const utils::String &seqR)
{
    m_sequenceTrans->clear();
    m_sequenceTrans->resize(*m_nbDofTrans);
    m_sequenceRot->clear();
    if (*m_isQuaternion) {
        m_sequenceRot->resize(1);
    } else {
        m_sequenceRot->resize(*m_nbDofRot);
    }

    str2numSequence(*m_sequenceTrans, seqT);
    str2numSequence(*m_sequenceRot, seqR);

    // Store the names of the DoFs
    m_nameDof->clear();
    m_nameDof->resize(*m_nbDofTrue);
    for (size_t i=0; i<*m_nbDofTrans; ++i) {
        (*m_nameDof)[i] = "Trans" + seqT(i).toupper();
    }
    for (size_t i=0; i<*m_nbDofRot; ++i) {
        (*m_nameDof)[*m_nbDofTrans+i] = "Rot" + seqR(i).toupper();
    }
    utils::String xyz("XYZW");
    for (size_t i=0; i<*m_nbDofQuat; ++i) {
        (*m_nameDof)[*m_nbDofTrans + *m_nbDofRot + i] = utils::String("Quat") +
                xyz.substr(i, 1);
    }

}
void rigidbody::Segment::str2numSequence(
    std::vector<size_t>& sequenceInteger,
    const utils::String &sequenceText)
{
    for (size_t i=0; i<sequenceText.length(); i++) {
        char tp = sequenceText.tolower()[i];
        if      (tp == 'x') {
            sequenceInteger[i] = 0;
        } else if (tp == 'y') {
            sequenceInteger[i] = 1;
        } else if (tp == 'z') {
            sequenceInteger[i] = 2;
        } else if (tp == 'q') {
            sequenceInteger[i] = 3;
        } else {
            utils::Error::raise("Wrong sequence!");
        }
    }
}
void rigidbody::Segment::setNumberOfDof(size_t nTrans,
        size_t nRot)
{
    *m_nbDofTrans = nTrans;
    if (*m_isQuaternion) {
        *m_nbDofRot = 0;
        *m_nbDofQuat = 4;
        *m_nbDof = nTrans + 1;
        *m_nbQdot = nTrans + 3;
        *m_nbQddot = nTrans + 3;
        *m_nbDofTrue = nTrans + *m_nbDofQuat;
        *m_nbDofTrueOutside = nTrans + 3;
    } else {
        *m_nbDofRot = nRot;
        *m_nbDofQuat = 0;
        *m_nbDof = nTrans + *m_nbDofRot;
        *m_nbQdot = *m_nbDof;
        *m_nbQddot = *m_nbDof;
        *m_nbDofTrue = *m_nbDof;
        *m_nbDofTrueOutside = *m_nbDof;
    }
}

void rigidbody::Segment::setSequence(const utils::String &seqT,
        const utils::String
        &seqR)   // Find the x, y, and z positions in this sequence
{
    setNumberOfDof(seqT.length(), seqR.length());
    str2numSequence(seqT, seqR);
    fillSequence();
}
void rigidbody::Segment::fillSequence()
{
    m_dofPosition->clear();
    m_dofPosition->resize(*m_nbDof);

    for (size_t i=0; i<*m_nbDofTrans; i++) {
        (*m_dofPosition)[i] =
            (*m_sequenceTrans)[i];    // Place the translation first in the requested order
    }
    if (*m_isQuaternion) {
        (*m_dofPosition)[*m_nbDofTrans] = (*m_sequenceRot)[0];
    } else
        for (size_t i=0; i<*m_nbDofRot; i++) {
            (*m_dofPosition)[i+*m_nbDofTrans] =
                (*m_sequenceRot)[i];    // Place the rotation following the translations in the requested order
        }
}

void rigidbody::Segment::setDofCharacteristicsOnLastBody()
{
    m_dofCharacteristics->clear();

    if (*m_nbDof!=0) {
        m_dofCharacteristics->resize(*m_nbDof);
        for (size_t i=0; i<*m_nbDof-1; i++) {
            (*m_dofCharacteristics)[i] = rigidbody::SegmentCharacteristics();
        }

        (*m_dofCharacteristics)[*m_nbDof-1] = *m_characteristics;
    } else {
        m_dofCharacteristics->resize(1);
        (*m_dofCharacteristics)[0] = *m_characteristics;
    }
}

void rigidbody::Segment::setJointAxis()
{
    // Definition of the rotation axis
    utils::Vector3d axis[3];
    axis[0]  = utils::Vector3d(1,0,0); // axe x
    axis[1]  = utils::Vector3d(0,1,0); // axe y
    axis[2]  = utils::Vector3d(0,0,1); // axe z

    // Declaration of DoFs in translation
    m_dof->clear();
    if (*m_nbDof != 0) {
        m_dof->resize(*m_nbDof);
        for (size_t i = 0; i < *m_nbDofTrans; i++) {
            (*m_dof)[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypePrismatic, axis[(*m_dofPosition)[i]]);
        }

        // Declaration of the DoFs in rotation
        if (*m_isQuaternion) {
            (*m_dof)[*m_nbDofTrans] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeSpherical);
        } else {
            for (size_t i = *m_nbDofTrans; i < *m_nbDofRot + *m_nbDofTrans; i++) {
                (*m_dof)[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, axis[(*m_dofPosition)[i]]);
            }
        }
    } else {
        m_dof->resize(1);
        (*m_dof)[0] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);
    }
}

void rigidbody::Segment::setJoints(
    rigidbody::Joints& model)
{
    setDofCharacteristicsOnLastBody(); // Apply the segment caracteristics only to the last segment
    setJointAxis(); // Choose the axis order in relation to the selected sequence


    utils::SpatialTransform zero;
    // Create the articulations (intra segment)
    m_idxDof->clear();

    if (*m_nbDof==0) {
        m_idxDof->resize(1);
    } else {
        m_idxDof->resize(*m_nbDof);
    }

    unsigned int parent_id(model.GetBodyId(parent().c_str()));

    if (parent_id == std::numeric_limits<unsigned int>::max()) {
        parent_id = 0;
    }
    if (*m_nbDof==0)
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCharacteristics)[0], name());
    else if (*m_nbDof == 1)
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCharacteristics)[0], name());
    else {
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCharacteristics)[0]);
        for (size_t i=1; i<*m_nbDof; i++)
            if (i!=*m_nbDof-1)
                (*m_idxDof)[i] = model.AddBody(
                    static_cast<unsigned int>((*m_idxDof)[i-1]), zero, (*m_dof)[i], (*m_dofCharacteristics)[i]
                );
            else
                (*m_idxDof)[i] = model.AddBody(
                    static_cast<unsigned int>((*m_idxDof)[i-1]), zero, (*m_dof)[i],(*m_dofCharacteristics)[i], name()
                );
    }
    *m_idxInModel = static_cast<int>(model.I.size() - 1);
}

size_t rigidbody::Segment::getDofIdx(
    const utils::String &dofName) const
{

    size_t idx(INT_MAX);
    bool found = false;
    for (size_t i=0; i<nbDof(); ++i) {
        if (!dofName.tolower().compare((*m_nameDof)[i].tolower())) {
            idx = i;
            found = true;
            break;
        }
    }


    utils::Error::check(
        found,
        "Type should be \"Rot\" or \"Trans\" and axis should be \"X\", \"Y\" or \"Z\", e.g. "
        "\"RotY\" for Rotation around y or \"TransX\" for Translation on x"
    );

    return idx;

}

const rigidbody::Segment& rigidbody::Segment::findFirstSegmentWithDof(
    const rigidbody::Joints& model) const 
{
    const rigidbody::Segment* segment = this;
  
    do {
        if (segment->nbDof() > 0) return *segment;
        utils::Error::check(
            segment->parent().compare("root"), 
            "Could not find any dof for " + name() + " up to the root."
        );
        segment = &model.segment(segment->parent());
    } while (true);
}

size_t rigidbody::Segment::getFirstDofIndexInGeneralizedCoordinates(
    const rigidbody::Joints& model) const 
{
    const utils::String toCompare(findFirstSegmentWithDof(model).name());

    // Start at root and descend the Generalized Coordinate vector until we get to the current segment
    size_t dofCount(0); // Start 
    for (int i = 0; i < static_cast<int>(model.nbSegment()); ++i) {
        const rigidbody::Segment& segment(model.segment(i));

        if (!segment.name().compare(toCompare)) return dofCount;
        
        dofCount += segment.nbDof();
    }

    throw std::runtime_error("Segment " + name() + " not found in the model");
}


size_t rigidbody::Segment::getLastDofIndexInGeneralizedCoordinates(
    const rigidbody::Joints& model) const
{
    return getFirstDofIndexInGeneralizedCoordinates(model) + findFirstSegmentWithDof(model).nbDof() - 1;
}
