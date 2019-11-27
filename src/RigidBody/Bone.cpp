#define BIORBD_API_EXPORTS
#include "RigidBody/Bone.h"

#include <limits.h>
#include "Utils/String.h"
#include "Utils/Error.h"
#include "Utils/Node3d.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/Joints.h"
#include "RigidBody/BoneMesh.h"
#include "RigidBody/BoneCharacteristics.h"

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
    m_characteristics(std::make_shared<biorbd::rigidbody::BoneCharacteristics>()),
    m_dofCharacteristics(std::make_shared<std::vector<biorbd::rigidbody::BoneCharacteristics>>())
{
    setType();
}

biorbd::rigidbody::Bone::Bone(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &name, // Nom du segment
        const biorbd::utils::String &parentName, // Nom du segment
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
        const biorbd::rigidbody::BoneCharacteristics& characteristics, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
        int PF) : // Numéro de la plateforme de force


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
    m_characteristics(std::make_shared<biorbd::rigidbody::BoneCharacteristics>(characteristics)),
    m_dofCharacteristics(std::make_shared<std::vector<biorbd::rigidbody::BoneCharacteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, seqT, seqR);
    // Add plateforme
    setPF(PF);
}
biorbd::rigidbody::Bone::Bone(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &name, // Nom du segment
        const biorbd::utils::String &parentName, // Nom du segment
        const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
        const biorbd::rigidbody::BoneCharacteristics& characteristics, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
        int PF): // Numéro de la plateforme de force


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
    m_characteristics(std::make_shared<biorbd::rigidbody::BoneCharacteristics>(characteristics)),
    m_dofCharacteristics(std::make_shared<std::vector<biorbd::rigidbody::BoneCharacteristics>>())
{
    setType();
    // Call proper functions
    setDofs(model, "", seqR);
    // Add plateforme
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
    *m_characteristics = other.m_characteristics->DeepCopy();
    m_dofCharacteristics->resize(other.m_dofCharacteristics->size());
    for (unsigned int i=0; i<other.m_dofCharacteristics->size(); ++i)
        (*m_dofCharacteristics)[i] = (*other.m_dofCharacteristics)[i].DeepCopy();
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

int biorbd::rigidbody::Bone::plateformeIdx() const{
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

// Ajout de la plateforme
void biorbd::rigidbody::Bone::setPF(int PF){
    *m_idxPF = PF;
}

const biorbd::utils::String &biorbd::rigidbody::Bone::nameDof(const unsigned int i) const {
    // Retourne le nombre de Dof de ce segment
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
const biorbd::rigidbody::BoneCharacteristics &biorbd::rigidbody::Bone::characteristics() const {
    return *m_characteristics;
}

void biorbd::rigidbody::Bone::setDofs(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR)// Séquence de Cardan pour classer les dof en rotation
{
    determineIfRotIsQuaternion(seqR);
    setSequence(seqT, seqR);

    setJoints(model);
}

void biorbd::rigidbody::Bone::determineIfRotIsQuaternion(const biorbd::utils::String &seqR){
    if (!seqR.tolower().compare("q"))
        *m_isQuaternion = true;
}


// Fonctions membres
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

    // Stocker les noms des dofs
    m_nameDof->clear();
    m_nameDof->resize(*m_nDofTrue);
    for (unsigned int i=0; i<*m_nDofTrans; ++i)
        (*m_nameDof)[i] = "Trans" + seqT(i).toupper();
    for (unsigned int i=0; i<*m_nDofRot; ++i)
        (*m_nameDof)[*m_nDofTrans+i] = "Rot" + seqR(i).toupper();
    biorbd::utils::String xyz("XYZW");
    for (unsigned int i=0; i<*m_nDofQuat; ++i)
        (*m_nameDof)[*m_nDofTrans+*m_nDofRot+i] =  biorbd::utils::String("Quat") + xyz.substr(i, 1);

}
void biorbd::rigidbody::Bone::str2numSequence(
        std::vector<unsigned int>& sequenceInteger,
        const biorbd::utils::String &seqTexte){
    for (unsigned int i=0; i<seqTexte.length(); i++){
        char tp = seqTexte.tolower()[i];
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

void biorbd::rigidbody::Bone::setSequence(const biorbd::utils::String &seqT, const biorbd::utils::String &seqR) { // Trouver les positions de x,y et z dans cette séquence
    setNumberOfDof(static_cast<unsigned int>(seqT.length()), static_cast<unsigned int>(seqR.length()));
    str2numSequence(seqT, seqR);
    fillSequence();
}
void biorbd::rigidbody::Bone::fillSequence(){
    m_dofPosition->clear();
    m_dofPosition->resize(*m_nDof);

    for (unsigned int i=0; i<*m_nDofTrans; i++)
        (*m_dofPosition)[i] = (*m_sequenceTrans)[i]; // Placer les translation en premier et dans l'ordre demandé
    if (*m_isQuaternion)
        (*m_dofPosition)[*m_nDofTrans] = (*m_sequenceRot)[0];
    else
        for (unsigned int i=0; i<*m_nDofRot; i++)
            (*m_dofPosition)[i+*m_nDofTrans] = (*m_sequenceRot)[i]; // Placer les rotation à la suite des translations dans l'ordre demandé
}

void biorbd::rigidbody::Bone::setDofCharacteristicsOnLastSegment(){
    m_dofCharacteristics->clear();

    if (*m_nDof!=0){
        m_dofCharacteristics->resize(*m_nDof);
        for (unsigned int i=0; i<*m_nDof-1; i++)
            (*m_dofCharacteristics)[i] = biorbd::rigidbody::BoneCharacteristics();

        (*m_dofCharacteristics)[*m_nDof-1] = *m_characteristics;
    }
    else{
        m_dofCharacteristics->resize(1);
        (*m_dofCharacteristics)[0] = *m_characteristics;
    }
}

void biorbd::rigidbody::Bone::setJointAxis(){
        // Définition des axes de rotation
    RigidBodyDynamics::Math::Vector3d axis[3];
    axis[0]  = RigidBodyDynamics::Math::Vector3d(1,0,0); // axe x
    axis[1]  = RigidBodyDynamics::Math::Vector3d(0,1,0); // axe y
    axis[2]  = RigidBodyDynamics::Math::Vector3d(0,0,1); // axe z

    // Déclaration des dof de translation
    m_dof->clear();
    if (*m_nDof != 0){
        m_dof->resize(*m_nDof);
        for (unsigned int i=0; i<*m_nDofTrans; i++)
            (*m_dof)[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypePrismatic, axis[(*m_dofPosition)[i]]);

        // Déclaration des dof de rotation
        if (*m_isQuaternion)
            (*m_dof)[*m_nDofTrans] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeSpherical); // Mettre un dof en sphérique
        else
            for (unsigned int i=*m_nDofTrans; i<*m_nDofRot+*m_nDofTrans; i++)
                (*m_dof)[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, axis[(*m_dofPosition)[i]]); // Mettre les axes de rotation dans le bon ordre
    }
    else{
        m_dof->resize(1);
        (*m_dof)[0] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed); // Un axe au hasard, p
    }
}

void biorbd::rigidbody::Bone::setJoints(biorbd::rigidbody::Joints& model){
    setDofCharacteristicsOnLastSegment(); // Put characteristics on the last segment
    setJointAxis(); // Choisir l'ordre des axes en fonction de la séquence sélectionnée

    RigidBodyDynamics::Math::SpatialTransform zero (RigidBodyDynamics::Math::Matrix3dIdentity, RigidBodyDynamics::Math::Vector3d(0,0,0));
    // Faire les articulations (intra segment)
    m_idxDof->clear();

    if (*m_nDof==0)
        m_idxDof->resize(1);
    else
        m_idxDof->resize(*m_nDof);

    unsigned int parent_id(model.GetBodyId(parent().c_str()));
    if (parent_id == std::numeric_limits<unsigned int>::max())
        parent_id = 0;
    if (*m_nDof==0)
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCharacteristics)[0], name());
    else if (*m_nDof == 1)
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCharacteristics)[0], name());
    else{
        (*m_idxDof)[0] = model.AddBody(parent_id, *m_cor, (*m_dof)[0], (*m_dofCharacteristics)[0]);
        for (unsigned int i=1; i<*m_nDof; i++)
            if (i!=*m_nDof-1)
                (*m_idxDof)[i] = model.AddBody((*m_idxDof)[i-1], zero, (*m_dof)[i], (*m_dofCharacteristics)[i]);
            else
                (*m_idxDof)[i] = model.AddBody((*m_idxDof)[i-1], zero, (*m_dof)[i], (*m_dofCharacteristics)[i], name());
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



