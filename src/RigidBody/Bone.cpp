#define BIORBD_API_EXPORTS
#include "RigidBody/Bone.h"

#include <limits.h>
#include <rbdl/rbdl_math.h>
#include "Utils/Error.h"
#include "Utils/Node.h"
#include "Utils/Attitude.h"
#include "RigidBody/Joints.h"
#include "RigidBody/BoneMesh.h"
#include "RigidBody/Patch.h"
#include "RigidBody/BoneCaracteristics.h"

biorbd::rigidbody::Bone::Bone(
        biorbd::rigidbody::Joints *model,
        const unsigned int &parent_id,
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
        const biorbd::rigidbody::Caracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
        const biorbd::utils::String &name, // Nom du segment
        const int &PF) : // Numéro de la plateforme de force
    m_name (name),
    m_parent_id(0),
    m_idxPF (PF),
    m_seqT(seqT),
    m_seqR(seqR),
    m_nDof(0),
    m_nDofTrue(0),
    m_nDofTrueOutside(0),
    m_nDofTrans(0),
    m_nDofRot(0),
    m_nDofQuat(0),
    m_isQuaternion(false),
    m_dof (nullptr),
    m_idxDof (nullptr),
    m_sequenceTrans (nullptr),
    m_sequenceRot (nullptr),
    m_nomDof (nullptr),
    m_dofPosition (nullptr),
    m_dofCaract (nullptr)
{
    // Call proper functions
    setParentToChildTransformation(cor);
    setDofs(model, parent_id, seqT, seqR, caract);
    // Add plateforme
    setPF(PF);
}
biorbd::rigidbody::Bone::Bone(
        biorbd::rigidbody::Joints *model,
        const unsigned int &parent_id,
        const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
        const biorbd::rigidbody::Caracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
        const biorbd::utils::String &name, // Nom du segment
        const int &PF): // Numéro de la plateforme de force
    m_name (name),
    m_parent_id(0),
    m_idxPF (PF),
    m_seqT(""),
    m_seqR(seqR),
    m_nDof(0),
    m_nDofTrue(0),
    m_nDofTrueOutside(0),
    m_nDofTrans(0),
    m_nDofRot(0),
    m_nDofQuat(0),
    m_isQuaternion(false),
    m_dof (nullptr),
    m_idxDof (nullptr),
    m_sequenceTrans (nullptr),
    m_sequenceRot (nullptr),
    m_nomDof (nullptr),
    m_dofPosition (nullptr),
    m_dofCaract (nullptr)
{
    // Call proper functions
    setParentToChildTransformation(cor);
    setDofs(model, parent_id, "", seqR, caract);
    // Add plateforme
    setPF(PF);
}
biorbd::rigidbody::Bone::Bone(const biorbd::rigidbody::Bone& bone)
{
    // Copy des attributs dimensionless
    m_name = bone.m_name;
    m_parent_id = bone.m_parent_id;
    m_idxPF = bone.m_idxPF;
    m_cor = bone.m_cor;
    m_seqT = bone.m_seqT;
    m_seqR = bone.m_seqR;
    m_nDof = bone.m_nDof;
    m_nQdot = bone.m_nQdot;
    m_nQddot = bone.m_nQddot;
    m_nDofTrue = bone.m_nDofTrue;
    m_nDofTrueOutside = bone.m_nDofTrueOutside;
    m_nDofTrans = bone.m_nDofTrans;
    m_nDofRot = bone.m_nDofRot;
    m_nDofQuat = bone.m_nDofQuat;
    m_isQuaternion = bone.m_isQuaternion;

    m_dof = nullptr;
    m_idxDof = nullptr;
    m_sequenceTrans = nullptr;
    m_sequenceRot = nullptr;
    m_nomDof = nullptr;
    m_caract = bone.m_caract;

    // Copy des attributs pointeurs
    m_dofPosition = nullptr;
    m_dofCaract = nullptr;


    m_sequenceTrans = new unsigned int[m_nDofTrans];
    for (unsigned int i=0; i<m_nDofTrans; ++i)
        m_sequenceTrans[i] = bone.m_sequenceTrans[i];
    if (m_isQuaternion){
        m_sequenceRot = new unsigned int[1];
        m_sequenceRot[0] = bone.m_sequenceRot[0];
    }
    else{
        m_sequenceRot = new unsigned int[m_nDofRot];
        for (unsigned int i=0; i<m_nDofRot; ++i)
            m_sequenceRot[i] = bone.m_sequenceRot[i];
    }
    m_nomDof = new biorbd::utils::String[m_nDofTrue];
    for (unsigned int i=0; i<m_nDofTrue; ++i)
        m_nomDof[i] = bone.m_nomDof[i];

    setDofCaracteristicsOnLastSegment(); // Take care of m_dofCaract
    fillSequence(); // Take care of m_dofPosition
    setJointAxis(); // Take care of m_dof
    if (m_nDof!=0){
        m_idxDof = new unsigned int[m_nDof];
        for (unsigned int i=0; i<m_nDof; ++i)
            m_idxDof[i] = bone.m_idxDof[i];
    }
    else{
        m_idxDof = new unsigned int[1];
        m_idxDof[0] = bone.m_idxDof[0];
    }
}

biorbd::rigidbody::Bone::~Bone(){
    delete[] m_dofPosition;
    delete[] m_dofCaract;
    delete[] m_dof;
    delete[] m_idxDof;
    delete[] m_sequenceTrans;
    delete[] m_sequenceRot;
    delete[] m_nomDof;
}

bool biorbd::rigidbody::Bone::isRotationAQuaternion() const{
    return m_isQuaternion;
}

unsigned int biorbd::rigidbody::Bone::id() const{
    if (m_nDof!=0)
        return m_idxDof[m_nDof-1];
    else
        return m_idxDof[m_nDof];
}

unsigned int biorbd::rigidbody::Bone::parent_rbdl_id() const
{
    return m_parent_id;
}

biorbd::utils::String biorbd::rigidbody::Bone::parentName(const biorbd::rigidbody::Joints &model) const
{
    return biorbd::utils::String(model.GetBodyName(m_parent_id));
}
int biorbd::rigidbody::Bone::plateformeIdx() const{
    return m_idxPF;
}
unsigned int biorbd::rigidbody::Bone::nGeneralizedTorque() const{
    return m_nDof;
}
unsigned int biorbd::rigidbody::Bone::nDof() const{
    return m_nDofTrueOutside;
}
unsigned int biorbd::rigidbody::Bone::nDofTrans() const{
    return m_nDofTrans;
}
unsigned int biorbd::rigidbody::Bone::nDofRot() const{
    return m_nDofRot;
}
unsigned int biorbd::rigidbody::Bone::nQ() const{
    return m_nDofTrue;
}
unsigned int biorbd::rigidbody::Bone::nQdot() const{
    return m_nQdot;
}
unsigned int biorbd::rigidbody::Bone::nQddot() const{
    return m_nQddot;
}

// Ajout de la plateforme
void biorbd::rigidbody::Bone::setPF(const int &PF){
    m_idxPF = PF;
}

const biorbd::utils::String &biorbd::rigidbody::Bone::nameDof(const unsigned int i) const {
    // Retourne le nombre de Dof de ce segment
    biorbd::utils::Error::error(i<m_nDofTrue, "Dof ouside N dof for this segment");
    return m_nomDof[i];
}

// Retourne le nom du segment
const biorbd::utils::String &biorbd::rigidbody::Bone::name() const {
    return m_name;
}

const biorbd::utils::String& biorbd::rigidbody::Bone::seqT() const
{
    return m_seqT;
}

const biorbd::utils::String& biorbd::rigidbody::Bone::seqR() const
{
    return m_seqR;
}

biorbd::utils::Attitude biorbd::rigidbody::Bone::localJCS() const {
    return m_cor;
}
const biorbd::rigidbody::Caracteristics &biorbd::rigidbody::Bone::caract() const {
    return m_caract;
}

// SetDofs => permet d'appeler correctement la fonction setJoints qui est nécessaire pour inclure les dof intrasegments
void biorbd::rigidbody::Bone::setDofs(
        biorbd::rigidbody::Joints *model,
        const unsigned int &parent_id,
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
        const double &mass, // Masse du segment
        const RigidBodyDynamics::Math::Vector3d &com,   // Centre de masse du segment
        const RigidBodyDynamics::Math::Matrix3d &inertia){ // Insérer les valeurs des caractéristiques
    setDofs(model, parent_id, seqT, seqR, biorbd::rigidbody::Caracteristics(mass, com, inertia));
}
void biorbd::rigidbody::Bone::setDofs(
        biorbd::rigidbody::Joints *model,
        const unsigned int &parent_id,
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR,// Séquence de Cardan pour classer les dof en rotation
        const biorbd::rigidbody::Caracteristics &b){
    m_caract = b;
    m_parent_id = parent_id;

    determineIfRotIsQuaternion(seqR);
    setSequence(seqT, seqR);

    setJoints(model);
}

void biorbd::rigidbody::Bone::determineIfRotIsQuaternion(const biorbd::utils::String &seqR){
    if (!seqR.tolower().compare("q"))
        m_isQuaternion = true;
}


// Fonctions membres
void biorbd::rigidbody::Bone::setParentToChildTransformation(const RigidBodyDynamics::Math::SpatialTransform &cor){
    m_cor = cor;
}
void biorbd::rigidbody::Bone::str2numSequence(
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR){
    delete[] m_sequenceTrans;
    m_sequenceTrans = new unsigned int[m_nDofTrans];
    delete[] m_sequenceRot;
    if (m_isQuaternion)
        m_sequenceRot = new unsigned int[1];
    else
        m_sequenceRot = new unsigned int[m_nDofRot];

    str2numSequence(m_sequenceTrans, seqT);
    str2numSequence(m_sequenceRot, seqR);

    // Stocker les noms des dofs
    delete[] m_nomDof;
    m_nomDof = new biorbd::utils::String[m_nDofTrue];
    for (unsigned int i=0; i<m_nDofTrans; ++i)
        m_nomDof[i] = "Trans" + seqT(i).toupper();
    for (unsigned int i=0; i<m_nDofRot; ++i)
        m_nomDof[m_nDofTrans+i] = "Rot" + seqR(i).toupper();
    for (unsigned int i=0; i<m_nDofQuat; ++i)
        m_nomDof[m_nDofTrans+m_nDofRot+i] =  biorbd::utils::String("Quat") + i;

}
void biorbd::rigidbody::Bone::str2numSequence(
        unsigned int * seqInt,
        const biorbd::utils::String &seqTexte){
    for (unsigned int i=0; i<seqTexte.length(); i++){
        char tp = seqTexte.tolower()[i];
        if      (tp == 'x')
            seqInt[i] = 0;
        else if (tp == 'y')
            seqInt[i] = 1;
        else if (tp == 'z')
            seqInt[i] = 2;
        else if (tp == 'q'){
            seqInt[i] = 3;
        }
        else
            biorbd::utils::Error::error(0,"Wrong sequence!");
    }
}
void biorbd::rigidbody::Bone::setNumberOfDof(const unsigned int &nTrans, const unsigned int &nRot){
    m_nDofTrans = nTrans;
    if (m_isQuaternion){
        m_nDofRot = 0;
        m_nDofQuat = 4;
        m_nDof = nTrans + 1;
        m_nQdot = nTrans + 3;
        m_nQddot = nTrans + 3;
        m_nDofTrue = nTrans + m_nDofQuat;
        m_nDofTrueOutside = nTrans + 3;
    }
    else{
        m_nDofRot = nRot;
        m_nDofQuat = 0;
        m_nDof = nTrans + m_nDofRot;
        m_nQdot = m_nDof;
        m_nQddot = m_nDof;
        m_nDofTrue = m_nDof;
        m_nDofTrueOutside = m_nDof;
    }
}

void biorbd::rigidbody::Bone::setSequence(const biorbd::utils::String &seqT, const biorbd::utils::String &seqR) { // Trouver les positions de x,y et z dans cette séquence
    setNumberOfDof(static_cast<unsigned int>(seqT.length()), static_cast<unsigned int>(seqR.length()));
    str2numSequence(seqT, seqR);
    fillSequence();
}
void biorbd::rigidbody::Bone::fillSequence(){
    delete[] m_dofPosition;
    m_dofPosition = new unsigned int[m_nDof];

    for (unsigned int i=0; i<m_nDofTrans; i++)
        m_dofPosition[i] = m_sequenceTrans[i]; // Placer les translation en premier et dans l'ordre demandé
    if (m_isQuaternion)
        m_dofPosition[m_nDofTrans] = m_sequenceRot[0];
    else
        for (unsigned int i=0; i<m_nDofRot; i++)
            m_dofPosition[i+m_nDofTrans] = m_sequenceRot[i]; // Placer les rotation à la suite des translations dans l'ordre demandé
}

void biorbd::rigidbody::Bone::setDofCaracteristicsOnLastSegment(){
    delete[] m_dofCaract;

    if (m_nDof!=0){
        m_dofCaract = new biorbd::rigidbody::Caracteristics[m_nDof];
        for (unsigned int i=0; i<m_nDof-1; i++)
            m_dofCaract[i] = biorbd::rigidbody::Caracteristics();

        m_dofCaract[m_nDof-1] = m_caract;
    }
    else{
        m_dofCaract = new biorbd::rigidbody::Caracteristics[1];
        m_dofCaract[0] = m_caract;
    }
}

void biorbd::rigidbody::Bone::setJointAxis(){
        // Définition des axes de rotation
    RigidBodyDynamics::Math::Vector3d axis[3];
    axis[0]  = RigidBodyDynamics::Math::Vector3d(1,0,0); // axe x
    axis[1]  = RigidBodyDynamics::Math::Vector3d(0,1,0); // axe y
    axis[2]  = RigidBodyDynamics::Math::Vector3d(0,0,1); // axe z

    // Déclaration des dof de translation
    delete[] m_dof;
    if (m_nDof != 0){
        m_dof = new biorbd::rigidbody::Joint[m_nDof];
        for (unsigned int i=0; i<m_nDofTrans; i++)
            m_dof[i] = biorbd::rigidbody::Joint(RigidBodyDynamics::JointTypePrismatic, axis[m_dofPosition[i]]);

        // Déclaration des dof de rotation
        if (m_isQuaternion)
            m_dof[m_nDofTrans] = biorbd::rigidbody::Joint(RigidBodyDynamics::JointTypeSpherical); // Mettre un dof en sphérique
        else
            for (unsigned int i=m_nDofTrans; i<m_nDofRot+m_nDofTrans; i++)
                m_dof[i] = biorbd::rigidbody::Joint(RigidBodyDynamics::JointTypeRevolute, axis[m_dofPosition[i]]); // Mettre les axes de rotation dans le bon ordre
    }
    else{
        m_dof = new biorbd::rigidbody::Joint[1];
        m_dof[0] = biorbd::rigidbody::Joint(RigidBodyDynamics::JointTypeFixed); // Un axe au hasard, p
    }
}

void biorbd::rigidbody::Bone::setJoints(biorbd::rigidbody::Joints *model){
    setDofCaracteristicsOnLastSegment(); // Mettre les caractéristiques segmentaires uniquement sur le dernier segment
    setJointAxis(); // Choisir l'ordre des axes en fonction de la séquence sélectionnée

    RigidBodyDynamics::Math::SpatialTransform zero (RigidBodyDynamics::Math::Matrix3dIdentity, RigidBodyDynamics::Math::Vector3d(0,0,0));
    // Faire les articulations (intra segment)
    delete[] m_idxDof;

    if (m_nDof==0)
        m_idxDof = new unsigned int[1];
    else
        m_idxDof = new unsigned int[m_nDof];

    if (m_nDof==0)
        m_idxDof[0] = model->AddBody(m_parent_id, m_cor, m_dof[0], m_dofCaract[0], m_name);
    else if (m_nDof == 1)
        m_idxDof[0] = model->AddBody(m_parent_id, m_cor, m_dof[0], m_dofCaract[0], m_name);
    else{
        m_idxDof[0] = model->AddBody(m_parent_id, m_cor, m_dof[0], m_dofCaract[0]);
        for (unsigned int i=1; i<m_nDof; i++)
            if (i!=m_nDof-1)
                m_idxDof[i] = model->AddBody(m_idxDof[i-1], zero, m_dof[i], m_dofCaract[i]);
            else
                m_idxDof[i] = model->AddBody(m_idxDof[i-1], zero, m_dof[i], m_dofCaract[i], m_name);
    }

}

unsigned int biorbd::rigidbody::Bone::getDofIdx(const biorbd::utils::String &dofName) const{

    unsigned int idx(INT_MAX);
    bool found = false;
    for (unsigned int i=0; i<nDof(); ++i){
        if (!dofName.tolower().compare(m_nomDof[i].tolower())){
            idx = i;
            found = true;
            break;
        }
    }


    biorbd::utils::Error::error(found, "Type should be \"Rot\" or \"Trans\" and axis should be \"X\", \"Y\" or \"Z\", e.g. \"RotY\" for Rotation around y or \"TransX\" for Translation on x");

    return idx;

}



