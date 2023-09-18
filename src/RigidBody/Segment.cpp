#define BIORBD_API_EXPORTS
#include "RigidBody/Segment.h"

#include <limits.h>
#include "Utils/String.h"
#include "Utils/Error.h"
#include "Utils/Matrix3d.h"
#include "Utils/Vector3d.h"
#include "Utils/RotoTrans.h"
#include "Utils/Rotation.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "Utils/Range.h"

using namespace BIORBD_NAMESPACE;

rigidbody::Segment::Segment() :
    utils::Node(),
    m_idxInModel(std::make_shared<int>(-1)),
    m_cor(std::make_shared<RigidBodyDynamics::Math::SpatialTransform>()),
    m_seqT(std::make_shared<utils::String>()),
    m_seqR(std::make_shared<utils::String>()),
    m_QRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QDotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QDDotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_nbDof(std::make_shared<unsigned int>(0)),
    m_nbQdot(std::make_shared<unsigned int>(0)),
    m_nbQddot(std::make_shared<unsigned int>(0)),
    m_nbDofTrue(std::make_shared<unsigned int>(0)),
    m_nbDofTrueOutside(std::make_shared<unsigned int>(0)),
    m_nbDofTrans(std::make_shared<unsigned int>(0)),
    m_nbDofRot(std::make_shared<unsigned int>(0)),
    m_nbDofQuat(std::make_shared<unsigned int>(0)),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceTrans(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceRot(std::make_shared<std::vector<unsigned int>>()),
    m_nameDof(std::make_shared<std::vector<utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<unsigned int>>()),
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
    const std::vector<utils::Range>& QDotRanges,
    const std::vector<utils::Range>& QDDotRanges,
    const rigidbody::SegmentCharacteristics& characteristics,
    const RigidBodyDynamics::Math::SpatialTransform& cor
) :

    utils::Node(name, parentName),
    m_idxInModel(std::make_shared<int>(-1)),
    m_cor(std::make_shared<RigidBodyDynamics::Math::SpatialTransform>(cor)),
    m_seqT(std::make_shared<utils::String>(seqT)),
    m_seqR(std::make_shared<utils::String>(seqR)),
    m_QRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QDotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QDDotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_nbDof(std::make_shared<unsigned int>(0)),
    m_nbQdot(std::make_shared<unsigned int>(0)),
    m_nbQddot(std::make_shared<unsigned int>(0)),
    m_nbDofTrue(std::make_shared<unsigned int>(0)),
    m_nbDofTrueOutside(std::make_shared<unsigned int>(0)),
    m_nbDofTrans(std::make_shared<unsigned int>(0)),
    m_nbDofRot(std::make_shared<unsigned int>(0)),
    m_nbDofQuat(std::make_shared<unsigned int>(0)),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceTrans(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceRot(std::make_shared<std::vector<unsigned int>>()),
    m_nameDof(std::make_shared<std::vector<utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<unsigned int>>()),
    m_characteristics(std::make_shared<rigidbody::SegmentCharacteristics>
                      (characteristics)),
    m_dofCharacteristics(
        std::make_shared<std::vector<rigidbody::SegmentCharacteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, seqT, seqR, QRanges, QDotRanges, QDDotRanges);
}
rigidbody::Segment::Segment(
    rigidbody::Joints& model,
    const utils::String &name, // Name of segment
    const utils::String &parentName, // Name of segment
    const utils::String
    &seqR, // Cardan sequence to classify the rotation DoF
    const std::vector<utils::Range>& QRanges,
    const std::vector<utils::Range>& QDotRanges,
    const std::vector<utils::Range>& QDDotRanges,
    const rigidbody::SegmentCharacteristics&
    characteristics, // Mass, Center of mass of segment, Inertia of segment, etc.
    const RigidBodyDynamics::Math::SpatialTransform&
    cor //  Transformation from parent to child
 ):


    utils::Node(name, parentName),
    m_idxInModel(std::make_shared<int>(-1)),
    m_cor(std::make_shared<RigidBodyDynamics::Math::SpatialTransform>(cor)),
    m_seqT(std::make_shared<utils::String>()),
    m_seqR(std::make_shared<utils::String>(seqR)),
    m_QRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QDotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_QDDotRanges(std::make_shared<std::vector<utils::Range>>()),
    m_nbDof(std::make_shared<unsigned int>(0)),
    m_nbQdot(std::make_shared<unsigned int>(0)),
    m_nbQddot(std::make_shared<unsigned int>(0)),
    m_nbDofTrue(std::make_shared<unsigned int>(0)),
    m_nbDofTrueOutside(std::make_shared<unsigned int>(0)),
    m_nbDofTrans(std::make_shared<unsigned int>(0)),
    m_nbDofRot(std::make_shared<unsigned int>(0)),
    m_nbDofQuat(std::make_shared<unsigned int>(0)),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceTrans(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceRot(std::make_shared<std::vector<unsigned int>>()),
    m_nameDof(std::make_shared<std::vector<utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<unsigned int>>()),
    m_characteristics(std::make_shared<rigidbody::SegmentCharacteristics>
                      (characteristics)),
    m_dofCharacteristics(
        std::make_shared<std::vector<rigidbody::SegmentCharacteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, "", seqR, QRanges, QDotRanges, QDDotRanges);
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
    *m_QRanges = *other.m_QRanges;
    *m_QDotRanges = *other.m_QDotRanges;
    *m_QDDotRanges = *other.m_QDDotRanges;
    *m_nbDof = *other.m_nbDof;
    *m_nbQdot = *other.m_nbQdot;
    *m_nbQddot = *other.m_nbQddot;
    *m_nbDofTrue = *other.m_nbDofTrue;
    *m_nbDofTrueOutside = *other.m_nbDofTrueOutside;
    *m_nbDofTrans = *other.m_nbDofTrans;
    *m_nbDofRot = *other.m_nbDofRot;
    *m_nbDofQuat = *other.m_nbDofQuat;
    *m_isQuaternion = *other.m_isQuaternion;
    m_dof->resize(other.m_dof->size());
    for (unsigned int i=0; i<other.m_dof->size(); ++i) {
        (*m_dof)[i] = (*other.m_dof)[i];
    }
    *m_idxDof = *other.m_idxDof;
    *m_sequenceTrans = *other.m_sequenceTrans;
    *m_sequenceRot = *other.m_sequenceRot;
    m_nameDof->resize(other.m_nameDof->size());
    for (unsigned int i=0; i<other.m_nameDof->size(); ++i) {
        (*m_nameDof)[i] = (*other.m_nameDof)[i];
    }
    *m_dofPosition = *other.m_dofPosition;
    *m_characteristics = other.m_characteristics->DeepCopy();
    m_dofCharacteristics->resize(other.m_dofCharacteristics->size());
    for (unsigned int i=0; i<other.m_dofCharacteristics->size(); ++i) {
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

unsigned int rigidbody::Segment::id() const
{
    if (*m_nbDof!=0) {
        return (*m_idxDof)[*m_nbDof-1];
    } else {
        return (*m_idxDof)[*m_nbDof];
    }
}

unsigned int rigidbody::Segment::nbGeneralizedTorque() const
{
    return nbQddot();
}
unsigned int rigidbody::Segment::nbDof() const
{
    return *m_nbDofTrueOutside;
}
unsigned int rigidbody::Segment::nbDofTrans() const
{
    return *m_nbDofTrans;
}
unsigned int rigidbody::Segment::nbDofRot() const
{
    return *m_nbDofRot;
}
unsigned int rigidbody::Segment::nbQ() const
{
    return *m_nbDofTrue;
}
unsigned int rigidbody::Segment::nbQdot() const
{
    return *m_nbQdot;
}
unsigned int rigidbody::Segment::nbQddot() const
{
    return *m_nbQddot;
}

const utils::String &rigidbody::Segment::nameDof(
    const unsigned int i) const
{
    // Return the number of DoF of the segment
    utils::Error::check(i<*m_nbDofTrue,
                                "Dof ouside N dof for this segment");
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

const std::vector<utils::Range>&
rigidbody::Segment::QRanges() const
{
    return *m_QRanges;
}

const std::vector<utils::Range>&
rigidbody::Segment::QDotRanges() const
{
    return *m_QDotRanges;
}

const std::vector<utils::Range>&
rigidbody::Segment::QDDotRanges() const
{
    return *m_QDDotRanges;
}

utils::RotoTrans rigidbody::Segment::localJCS() const
{
    return RigidBodyDynamics::Math::SpatialTransform(m_cor->E.transpose(),
            m_cor->r);
}

void  rigidbody::Segment::setLocalJCS(rigidbody::Joints& model, utils::RotoTrans &rototrans)
{
    *m_cor = RigidBodyDynamics::Math::SpatialTransform(
                rototrans.rot().transpose(),
                    rototrans.trans());
    // we also modify RBDL spatial transform from parent to child
    model.X_T[*m_idxDof->begin()]=*m_cor;
}


void rigidbody::Segment::updateCharacteristics(
    rigidbody::Joints& model,
    const rigidbody::SegmentCharacteristics& characteristics)
{

    *m_characteristics = characteristics.DeepCopy();
    RigidBodyDynamics::Math::SpatialRigidBodyInertia rbi =
        RigidBodyDynamics::Math::SpatialRigidBodyInertia::createFromMassComInertiaC (
            m_characteristics->mMass,
            m_characteristics->mCenterOfMass,
            m_characteristics->mInertia);

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
    const std::vector<utils::Range>& QDotRanges,
    const std::vector<utils::Range>& QDDotRanges)
{
    determineIfRotIsQuaternion(seqR);
    setSequence(seqT, seqR);

    // Control for the size of dofRange then copy it
    size_t nRot(seqR.length());
    if (*m_isQuaternion) {
        nRot = 0;
    }
    utils::Error::check(
        QRanges.size() == 0 ||
        QRanges.size() == seqT.length() + nRot ||
        (QRanges.size() == 4 && m_isQuaternion),
        "QRanges and number of dof must be equal");
    *m_QRanges = QRanges;

    utils::Error::check(
        QDotRanges.size() == 0 ||
        QDotRanges.size() == seqT.length() + nRot ||
        (QDotRanges.size() == 3 && m_isQuaternion),
        "QDotRanges and number of dof must be equal");
    *m_QDotRanges = QDotRanges;

    utils::Error::check(
        QDDotRanges.size() == 0 ||
        QDDotRanges.size() == seqT.length() + nRot ||
        (QDDotRanges.size() == 3 && m_isQuaternion),
        "QDDotRanges and number of dof must be equal");
    *m_QDDotRanges = QDDotRanges;
    setJoints(model);
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
    for (unsigned int i=0; i<*m_nbDofTrans; ++i) {
        (*m_nameDof)[i] = "Trans" + seqT(i).toupper();
    }
    for (unsigned int i=0; i<*m_nbDofRot; ++i) {
        (*m_nameDof)[*m_nbDofTrans+i] = "Rot" + seqR(i).toupper();
    }
    utils::String xyz("XYZW");
    for (unsigned int i=0; i<*m_nbDofQuat; ++i) {
        (*m_nameDof)[*m_nbDofTrans + *m_nbDofRot + i] = utils::String("Quat") +
                xyz.substr(i, 1);
    }

}
void rigidbody::Segment::str2numSequence(
    std::vector<unsigned int>& sequenceInteger,
    const utils::String &sequenceText)
{
    for (unsigned int i=0; i<sequenceText.length(); i++) {
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
void rigidbody::Segment::setNumberOfDof(unsigned int nTrans,
        unsigned int nRot)
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
    setNumberOfDof(static_cast<unsigned int>(seqT.length()),
                   static_cast<unsigned int>(seqR.length()));
    str2numSequence(seqT, seqR);
    fillSequence();
}
void rigidbody::Segment::fillSequence()
{
    m_dofPosition->clear();
    m_dofPosition->resize(*m_nbDof);

    for (unsigned int i=0; i<*m_nbDofTrans; i++) {
        (*m_dofPosition)[i] =
            (*m_sequenceTrans)[i];    // Place the translation first in the requested order
    }
    if (*m_isQuaternion) {
        (*m_dofPosition)[*m_nbDofTrans] = (*m_sequenceRot)[0];
    } else
        for (unsigned int i=0; i<*m_nbDofRot; i++) {
            (*m_dofPosition)[i+*m_nbDofTrans] =
                (*m_sequenceRot)[i];    // Place the rotation following the translations in the requested order
        }
}

void rigidbody::Segment::setDofCharacteristicsOnLastBody()
{
    m_dofCharacteristics->clear();

    if (*m_nbDof!=0) {
        m_dofCharacteristics->resize(*m_nbDof);
        for (unsigned int i=0; i<*m_nbDof-1; i++) {
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
        for (unsigned int i=0; i<*m_nbDofTrans; i++)
            (*m_dof)[i] = RigidBodyDynamics::Joint(
                              RigidBodyDynamics::JointTypePrismatic,
                              axis[(*m_dofPosition)[i]]);

        // Declaration of the DoFs in rotation
        if (*m_isQuaternion)
            (*m_dof)[*m_nbDofTrans] = RigidBodyDynamics::Joint(
                                          RigidBodyDynamics::JointTypeSpherical);
        else
            for (unsigned int i=*m_nbDofTrans; i<*m_nbDofRot+*m_nbDofTrans; i++)
                (*m_dof)[i] = RigidBodyDynamics::Joint(
                                  RigidBodyDynamics::JointTypeRevolute,
                                  axis[(*m_dofPosition)[i]]);
    } else {
        m_dof->resize(1);
        (*m_dof)[0] = RigidBodyDynamics::Joint
                      (RigidBodyDynamics::JointTypeFixed);
    }
}

void rigidbody::Segment::setJoints(
    rigidbody::Joints& model)
{
    setDofCharacteristicsOnLastBody(); // Apply the segment caracteristics only to the last segment
    setJointAxis(); // Choose the axis order in relation to the selected sequence


    RigidBodyDynamics::Math::SpatialTransform zero (
        utils::Matrix3d::Identity(),
        RigidBodyDynamics::Math::Vector3d(0,0,0));
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
        for (unsigned int i=1; i<*m_nbDof; i++)
            if (i!=*m_nbDof-1)
                (*m_idxDof)[i] = model.AddBody((*m_idxDof)[i-1], zero, (*m_dof)[i], (*m_dofCharacteristics)[i]);
            else
                (*m_idxDof)[i] = model.AddBody((*m_idxDof)[i-1], zero, (*m_dof)[i],(*m_dofCharacteristics)[i], name());
    }
    *m_idxInModel = model.I.size() - 1;
}

unsigned int rigidbody::Segment::getDofIdx(
    const utils::String &dofName) const
{

    unsigned int idx(INT_MAX);
    bool found = false;
    for (unsigned int i=0; i<nbDof(); ++i) {
        if (!dofName.tolower().compare((*m_nameDof)[i].tolower())) {
            idx = i;
            found = true;
            break;
        }
    }


    utils::Error::check(found,
                                "Type should be \"Rot\" or \"Trans\" and axis "
                                "should be \"X\", \"Y\" or \"Z\", e.g. "
                                "\"RotY\" for Rotation around y or \"TransX\" "
                                "for Translation on x");

    return idx;

}



