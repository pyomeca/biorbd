#define BIORBD_API_EXPORTS
#include "RigidBody/Bone.h"

#include <limits.h>
#include "Utils/String.h"
#include "Utils/Error.h"
#include "Utils/Node3d.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/Joints.h"
#include "RigidBody/BoneMesh.h"
#include "RigidBody/BoneCaracteristics.h"

biorbd::rigidbody::Bone::Bone() :
    biorbd::utils::Node(),
    m_idxPF (std::make_shared<int>(-1)),
    m_cor(std::make_shared<RigidBodyDynamics::Math::SpatialTransform>()),
    m_seqT(std::make_shared<biorbd::utils::String>()),
    m_seqR(std::make_shared<biorbd::utils::String>()),
    m_nDof(std::make_shared<unsigned int>(0)),
    m_nQdot(std::make_shared<unsigned int>(0)),
    m_nQddot(std::make_shared<unsigned int>(0)),
    m_nDofTrue(std::make_shared<unsigned int>(0)),
    m_nDofTrueOutside(std::make_shared<unsigned int>(0)),
    m_nDofTrans(std::make_shared<unsigned int>(0)),
    m_nDofRot(std::make_shared<unsigned int>(0)),
    m_nDofQuat(std::make_shared<unsigned int>(0)),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceTrans(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceRot(std::make_shared<std::vector<unsigned int>>()),
    m_nameDof(std::make_shared<std::vector<biorbd::utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<unsigned int>>()),
    m_caract(std::make_shared<biorbd::rigidbody::BoneCaracteristics>()),
    m_dofCaract(std::make_shared<std::vector<biorbd::rigidbody::BoneCaracteristics>>())
{
    setType();
}

biorbd::rigidbody::Bone::Bone(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &name, 
        const biorbd::utils::String &parentName, 
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR, 
        const biorbd::rigidbody::BoneCaracteristics& caract, 
        const RigidBodyDynamics::Math::SpatialTransform& cor, 
        int PF) : 
    biorbd::utils::Node(name, parentName),
    m_idxPF (std::make_shared<int>(PF)),
    m_cor(std::make_shared<RigidBodyDynamics::Math::SpatialTransform>(cor)),
    m_seqT(std::make_shared<biorbd::utils::String>(seqT)),
    m_seqR(std::make_shared<biorbd::utils::String>(seqR)),
    m_nDof(std::make_shared<unsigned int>(0)),
    m_nQdot(std::make_shared<unsigned int>(0)),
    m_nQddot(std::make_shared<unsigned int>(0)),
    m_nDofTrue(std::make_shared<unsigned int>(0)),
    m_nDofTrueOutside(std::make_shared<unsigned int>(0)),
    m_nDofTrans(std::make_shared<unsigned int>(0)),
    m_nDofRot(std::make_shared<unsigned int>(0)),
    m_nDofQuat(std::make_shared<unsigned int>(0)),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceTrans(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceRot(std::make_shared<std::vector<unsigned int>>()),
    m_nameDof(std::make_shared<std::vector<biorbd::utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<unsigned int>>()),
    m_caract(std::make_shared<biorbd::rigidbody::BoneCaracteristics>(caract)),
    m_dofCaract(std::make_shared<std::vector<biorbd::rigidbody::BoneCaracteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, seqT, seqR);
    // Add platform
    setPF(PF);
}
biorbd::rigidbody::Bone::Bone(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &name, 
        const biorbd::utils::String &parentName, 
        const biorbd::utils::String &seqR, 
        const biorbd::rigidbody::BoneCaracteristics& caract, 
        const RigidBodyDynamics::Math::SpatialTransform& cor,
        int PF): 
    biorbd::utils::Node(name, parentName),
    m_idxPF (std::make_shared<int>(PF)),
    m_cor(std::make_shared<RigidBodyDynamics::Math::SpatialTransform>(cor)),
    m_seqT(std::make_shared<biorbd::utils::String>()),
    m_seqR(std::make_shared<biorbd::utils::String>(seqR)),
    m_nDof(std::make_shared<unsigned int>(0)),
    m_nQdot(std::make_shared<unsigned int>(0)),
    m_nQddot(std::make_shared<unsigned int>(0)),
    m_nDofTrue(std::make_shared<unsigned int>(0)),
    m_nDofTrueOutside(std::make_shared<unsigned int>(0)),
    m_nDofTrans(std::make_shared<unsigned int>(0)),
    m_nDofRot(std::make_shared<unsigned int>(0)),
    m_nDofQuat(std::make_shared<unsigned int>(0)),
    m_isQuaternion(std::make_shared<bool>(false)),
    m_dof(std::make_shared<std::vector<RigidBodyDynamics::Joint>>()),
    m_idxDof(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceTrans(std::make_shared<std::vector<unsigned int>>()),
    m_sequenceRot(std::make_shared<std::vector<unsigned int>>()),
    m_nameDof(std::make_shared<std::vector<biorbd::utils::String>>()),
    m_dofPosition(std::make_shared<std::vector<unsigned int>>()),
    m_caract(std::make_shared<biorbd::rigidbody::BoneCaracteristics>(caract)),
    m_dofCaract(std::make_shared<std::vector<biorbd::rigidbody::BoneCaracteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, "", seqR);
    // Add platform
    setPF(PF);
}

biorbd::rigidbody::Bone biorbd::rigidbody::Bone::DeepCopy() const
{
    biorbd::rigidbody::Bone copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Bone::DeepCopy(const biorbd::rigidbody::Bone &other)
{
    biorbd::utils::Node::DeepCopy(other);
    *m_idxPF = *other.m_idxPF;
    *m_cor = *other.m_cor;
    *m_seqT = *other.m_seqT;
    *m_seqR = *other.m_seqR;
    *m_nDof = *other.m_nDof;
    *m_nQdot = *other.m_nQdot;
    *m_nQddot = *other.m_nQddot;
    *m_nDofTrue = *other.m_nDofTrue;
    *m_nDofTrueOutside = *other.m_nDofTrueOutside;
    *m_nDofTrans = *other.m_nDofTrans;
    *m_nDofRot = *other.m_nDofRot;
    *m_nDofQuat = *other.m_nDofQuat;
    *m_isQuaternion = *other.m_isQuaternion;
    m_dof->resize(other.m_dof->size());
    for (unsigned int i=0; i<other.m_dof->size(); ++i)
        (*m_dof)[i] = (*other.m_dof)[i];
    *m_idxDof = *other.m_idxDof;
    *m_sequenceTrans = *other.m_sequenceTrans;
    *m_sequenceRot = *other.m_sequenceRot;
    m_nameDof->resize(other.m_nameDof->size());
    for (unsigned int i=0; i<other.m_nameDof->size(); ++i)
        (*m_nameDof)[i] = (*other.m_nameDof)[i];
    *m_dofPosition = *other.m_dofPosition;
    *m_caract = other.m_caract->DeepCopy();
    m_dofCaract->resize(other.m_dofCaract->size());
    for (unsigned int i=0; i<other.m_dofCaract->size(); ++i)
        (*m_dofCaract)[i] = (*other.m_dofCaract)[i].DeepCopy();
}

biorbd::rigidbody::Bone::~Bone(){

}

bool biorbd::rigidbody::Bone::isRotationAQuaternion() const{
    return *m_isQuaternion;
}

void biorbd::rigidbody::Bone::setType()
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::BONE;
}

unsigned int biorbd::rigidbody::Bone::id() const{
    if (*m_nDof!=0)
        return (*m_idxDof)[*m_nDof-1];
    else
        return (*m_idxDof)[*m_nDof];
}

int biorbd::rigidbody::Bone::platformIdx() const{
    return *m_idxPF;
}
unsigned int biorbd::rigidbody::Bone::nGeneralizedTorque() const{
    return *m_nDof;
}
unsigned int biorbd::rigidbody::Bone::nDof() const{
    return *m_nDofTrueOutside;
}
unsigned int biorbd::rigidbody::Bone::nDofTrans() const{
    return *m_nDofTrans;
}
unsigned int biorbd::rigidbody::Bone::nDofRot() const{
    return *m_nDofRot;
}
unsigned int biorbd::rigidbody::Bone::nQ() const{
    return *m_nDofTrue;
}
unsigned int biorbd::rigidbody::Bone::nQdot() const{
    return *m_nQdot;
}
unsigned int biorbd::rigidbody::Bone::nQddot() const{
    return *m_nQddot;
}

// Add platform
void biorbd::rigidbody::Bone::setPF(int PF){
    *m_idxPF = PF;
}

const biorbd::utils::String &biorbd::rigidbody::Bone::nameDof(const unsigned int i) const {
    // Return the number of DoF of the segment
    biorbd::utils::Error::check(i<*m_nDofTrue, "Dof ouside N dof for this segment");
    return (*m_nameDof)[i];
}

const biorbd::utils::String& biorbd::rigidbody::Bone::seqT() const
{
    return *m_seqT;
}

const biorbd::utils::String& biorbd::rigidbody::Bone::seqR() const
{
    return *m_seqR;
}

biorbd::utils::RotoTrans biorbd::rigidbody::Bone::localJCS() const {
    return *m_cor;
}
const biorbd::rigidbody::BoneCaracteristics &biorbd::rigidbody::Bone::caract() const {
    return *m_caract;
}

void biorbd::rigidbody::Bone::setDofs(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR)
{
    determineIfRotIsQuaternion(seqR);
    setSequence(seqT, seqR);

    setJoints(model);
}

void biorbd::rigidbody::Bone::determineIfRotIsQuaternion(const biorbd::utils::String &seqR){
    if (!seqR.tolower().compare("q"))
        *m_isQuaternion = true;
}


// Member functions
void biorbd::rigidbody::Bone::str2numSequence(
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR){
    m_sequenceTrans->clear();
    m_sequenceTrans->resize(*m_nDofTrans);
    m_sequenceRot->clear();
    if (*m_isQuaternion)
        m_sequenceRot->resize(1);
    else
        m_sequenceRot->resize(*m_nDofRot);

    str2numSequence(*m_sequenceTrans, seqT);
    str2numSequence(*m_sequenceRot, seqR);

    // Store the names of the DoFs
    m_nameDof->clear();
    m_nameDof->resize(*m_nDofTrue);
    for (unsigned int i=0; i<*m_nDofTrans; ++i)
        (*m_nameDof)[i] = "Trans" + seqT(i).toupper();
    for (unsigned int i=0; i<*m_nDofRot; ++i)
        (*m_nameDof)[*m_nDofTrans+i] = "Rot" + seqR(i).toupper();
    for (unsigned int i=0; i<*m_nDofQuat; ++i)
        (*m_nameDof)[*m_nDofTrans+*m_nDofRot+i] =  biorbd::utils::String("Quat") + i;

}
void biorbd::rigidbody::Bone::str2numSequence(
        std::vector<unsigned int>& sequenceInteger,
        const biorbd::utils::String &sequenceText){
    for (unsigned int i=0; i<sequenceText.length(); i++){
        char tp = sequenceText.tolower()[i];
        if      (tp == 'x')
            sequenceInteger[i] = 0;
        else if (tp == 'y')
            sequenceInteger[i] = 1;
        else if (tp == 'z')
            sequenceInteger[i] = 2;
        else if (tp == 'q'){
            sequenceInteger[i] = 3;
        }
        else
            biorbd::utils::Error::raise("Wrong sequence!");
    }
}
void biorbd::rigidbody::Bone::setNumberOfDof(unsigned int nTrans, unsigned int nRot){
    *m_nDofTrans = nTrans;
    if (*m_isQuaternion){
        *m_nDofRot = 0;
        *m_nDofQuat = 4;
        *m_nDof = nTrans + 1;
        *m_nQdot = nTrans + 3;
        *m_nQddot = nTrans + 3;
        *m_nDofTrue = nTrans + *m_nDofQuat;
        *m_nDofTrueOutside = nTrans + 3;
    }
    else{
        *m_nDofRot = nRot;
        *m_nDofQuat = 0;
        *m_nDof = nTrans + *m_nDofRot;
        *m_nQdot = *m_nDof;
        *m_nQddot = *m_nDof;
        *m_nDofTrue = *m_nDof;
        *m_nDofTrueOutside = *m_nDof;
    }
}

void biorbd::rigidbody::Bone::setSequence(const biorbd::utils::String &seqT, const biorbd::utils::String &seqR) { // Find the x, y, and z positions in this sequence
    setNumberOfDof(static_cast<unsigned int>(seqT.length()), static_cast<unsigned int>(seqR.length()));
    str2numSequence(seqT, seqR);
    fillSequence();
}
void biorbd::rigidbody::Bone::fillSequence(){
    m_dofPosition->clear();
    m_dofPosition->resize(*m_nDof);

    for (unsigned int i=0; i<*m_nDofTrans; i++)
        (*m_dofPosition)[i] = (*m_sequenceTrans)[i]; // Place the translation first in the requested order
    if (*m_isQuaternion)
        (*m_dofPosition)[*m_nDofTrans] = (*m_sequenceRot)[0];
    else
        for (unsigned int i=0; i<*m_nDofRot; i++)
            (*m_dofPosition)[i+*m_nDofTrans] = (*m_sequenceRot)[i]; // Place the rotation following the translations in the requested order
}

void biorbd::rigidbody::Bone::setDofCaracteristicsOnLastSegment(){
    m_dofCaract->clear();

    if (*m_nDof!=0){
        m_dofCaract->resize(*m_nDof);
        for (unsigned int i=0; i<*m_nDof-1; i++)
            (*m_dofCaract)[i] = biorbd::rigidbody::BoneCaracteristics();

        (*m_dofCaract)[*m_nDof-1] = *m_caract;
    }
    else{
        m_dofCaract->resize(1);
        (*m_dofCaract)[0] = *m_caract;
    }
}

void biorbd::rigidbody::Bone::setJointAxis(){
        // Definition of the rotation axis
    RigidBodyDynamics::Math::Vector3d axis[3];
    axis[0]  = RigidBodyDynamics::Math::Vector3d(1,0,0); // axe x
    axis[1]  = RigidBodyDynamics::Math::Vector3d(0,1,0); // axe y
    axis[2]  = RigidBodyDynamics::Math::Vector3d(0,0,1); // axe z

    // Declaration of DoFs in translation
    m_dof->clear();
    if (*m_nDof != 0){
        m_dof->resize(*m_nDof);
        for (unsigned int i=0; i<*m_nDofTrans; i++)
            (*m_dof)[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypePrismatic, axis[(*m_dofPosition)[i]]);

        // Declaration of the DoFs in rotation
        if (*m_isQuaternion)
            (*m_dof)[*m_nDofTrans] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeSpherical); // Put a DoF in spherical
        else
            for (unsigned int i=*m_nDofTrans; i<*m_nDofRot+*m_nDofTrans; i++)
                (*m_dof)[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, axis[(*m_dofPosition)[i]]); // Put the rotation axis in the right order
    }
    else{
        m_dof->resize(1);
        (*m_dof)[0] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed); // A random axe, p
    }
}

void biorbd::rigidbody::Bone::setJoints(biorbd::rigidbody::Joints& model){
    setDofCaracteristicsOnLastSegment(); // Apply the segment caracteristics only to the last segment
    setJointAxis(); // Choose the axis order in relation to the selected sequence

    RigidBodyDynamics::Math::SpatialTransform zero (RigidBodyDynamics::Math::Matrix3dIdentity, RigidBodyDynamics::Math::Vector3d(0,0,0));
    // Create the articulations (intra segment)
    m_idxDof->clear();

    if (*m_nDof==0)
        m_idxDof->resize(1);
    else
        m_idxDof->resize(*m_nDof);

    unsigned int parent_id(model.GetBodyId(parent().c_str()));
    if (parent_id == std::numeric_limits<unsigned int>::max())
        parent_id = 0;
    if (*m_nDof==0)
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCaract)[0], name());
    else if (*m_nDof == 1)
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCaract)[0], name());
    else{
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCaract)[0]);
        for (unsigned int i=1; i<*m_nDof; i++)
            if (i!=*m_nDof-1)
                (*m_idxDof)[i] = model.AddBody((*m_idxDof)[i-1], zero, (*m_dof)[i], (*m_dofCaract)[i]);
            else
                (*m_idxDof)[i] = model.AddBody((*m_idxDof)[i-1], zero, (*m_dof)[i], (*m_dofCaract)[i], name());
    }

}

unsigned int biorbd::rigidbody::Bone::getDofIdx(const biorbd::utils::String &dofName) const{

    unsigned int idx(INT_MAX);
    bool found = false;
    for (unsigned int i=0; i<nDof(); ++i){
        if (!dofName.tolower().compare((*m_nameDof)[i].tolower())){
            idx = i;
            found = true;
            break;
        }
    }


    biorbd::utils::Error::check(found, "Type should be \"Rot\" or \"Trans\" and axis should be \"X\", \"Y\" or \"Z\", e.g. \"RotY\" for Rotation around y or \"TransX\" for Translation on x");

    return idx;

}



