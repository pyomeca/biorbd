#define BIORBD_API_EXPORTS
#include "../include/s2mBone.h"

s2mBone::s2mBone(s2mJoints *model, const unsigned int &parent_id,
        const s2mString &seqT, const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
        const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
        const s2mString &name, // Nom du segment
        const int &PF) : // Numéro de la plateforme de force
            // Pointers initiation
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
s2mBone::s2mBone(s2mJoints *model, const unsigned int &parent_id,
        const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
        const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
        const s2mString &name, // Nom du segment
        const int &PF): // Numéro de la plateforme de force
            // Pointers initiation
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
s2mBone::s2mBone(const s2mBone& bone)
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
    m_nomDof = new s2mString[m_nDofTrue];
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

s2mBone::~s2mBone(){
    delete[] m_dofPosition;
    delete[] m_dofCaract;
    delete[] m_dof;
    delete[] m_idxDof;
    delete[] m_sequenceTrans;
    delete[] m_sequenceRot;
    delete[] m_nomDof;
}

bool s2mBone::isRotationAQuaternion() const{
    return m_isQuaternion;
}

unsigned int s2mBone::id() const{
    if (m_nDof!=0)
        return m_idxDof[m_nDof-1];
    else
        return m_idxDof[m_nDof];
}

unsigned int s2mBone::parent_rbdl_id() const
{
    return m_parent_id;
}

s2mString s2mBone::parentName(const s2mJoints &model) const
{
    return s2mString(model.GetBodyName(m_parent_id));
}
int s2mBone::plateformeIdx() const{
    return m_idxPF;
}
unsigned int s2mBone::nTau() const{
    return m_nDof;
}
unsigned int s2mBone::nDof() const{
    return m_nDofTrueOutside;
}
unsigned int s2mBone::nDofTrans() const{
    return m_nDofTrans;
}
unsigned int s2mBone::nDofRot() const{
    return m_nDofRot;
}
unsigned int s2mBone::nQ() const{
    return m_nDofTrue;
}
unsigned int s2mBone::nQdot() const{
    return m_nQdot;
}
unsigned int s2mBone::nQddot() const{
    return m_nQddot;
}

// Ajout de la plateforme
void s2mBone::setPF(const int &PF){
    m_idxPF = PF;
}

s2mString s2mBone::nameDof(const unsigned int i) const {
    // Retourne le nombre de Dof de ce segment
    s2mError::s2mAssert(i<m_nDofTrue, "Dof ouside N dof for this segment");
    return m_nomDof[i];
}

// Retourne le nom du segment
s2mString s2mBone::name() const {
    return m_name;
}

s2mString s2mBone::seqT() const
{
    return m_seqT;
}

s2mString s2mBone::seqR() const
{
    return m_seqR;
}

s2mAttitude s2mBone::localJCS() const {
    return m_cor;
}
s2mBoneCaracteristics s2mBone::caract() const {
    return m_caract;
}

// SetDofs => permet d'appeler correctement la fonction setJoints qui est nécessaire pour inclure les dof intrasegments
void s2mBone::setDofs(s2mJoints *model, const unsigned int &parent_id,
        const s2mString &seqT, const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
        const double &mass, // Masse du segment
        const RigidBodyDynamics::Math::Vector3d &com,   // Centre de masse du segment
        const RigidBodyDynamics::Math::Matrix3d &inertia){ // Insérer les valeurs des caractéristiques
    setDofs(model, parent_id, seqT, seqR, s2mBoneCaracteristics(mass, com, inertia));
}
void s2mBone::setDofs(s2mJoints *model, const unsigned int &parent_id,
        const s2mString &seqT, const s2mString &seqR,// Séquence de Cardan pour classer les dof en rotation
        const s2mBoneCaracteristics &b){
    m_caract = b;
    m_parent_id = parent_id;

    determineIfRotIsQuaternion(seqR);
    setSequence(seqT, seqR);

    setJoints(model);
}

void s2mBone::determineIfRotIsQuaternion(const s2mString &seqR){
    if (!seqR.tolower().compare("q"))
        m_isQuaternion = true;
}


// Fonctions membres
void s2mBone::setParentToChildTransformation(const RigidBodyDynamics::Math::SpatialTransform &cor){
    m_cor = cor;
}
void s2mBone::str2numSequence(const s2mString &seqT, const s2mString &seqR){
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
    m_nomDof = new s2mString[m_nDofTrue];
    for (unsigned int i=0; i<m_nDofTrans; ++i)
        m_nomDof[i] = "Trans" + seqT(i).toupper();
    for (unsigned int i=0; i<m_nDofRot; ++i)
        m_nomDof[m_nDofTrans+i] = "Rot" + seqR(i).toupper();
    for (unsigned int i=0; i<m_nDofQuat; ++i)
        m_nomDof[m_nDofTrans+m_nDofRot+i] =  s2mString("Quat") + i;

}
void s2mBone::str2numSequence(unsigned int * seqInt, const s2mString &seqTexte){
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
            s2mError::s2mAssert(0,"Wrong sequence!");
    }
}
void s2mBone::setNumberOfDof(const unsigned int &nTrans, const unsigned int &nRot){
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

void s2mBone::setSequence(const s2mString &seqT, const s2mString &seqR) { // Trouver les positions de x,y et z dans cette séquence
    setNumberOfDof(seqT.length(), seqR.length());
    str2numSequence(seqT, seqR);
    fillSequence();
}
void s2mBone::fillSequence(){
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

void s2mBone::setDofCaracteristicsOnLastSegment(){
    delete[] m_dofCaract;

    if (m_nDof!=0){
        m_dofCaract = new s2mBoneCaracteristics[m_nDof];
        for (unsigned int i=0; i<m_nDof-1; i++)
            m_dofCaract[i] = s2mBoneCaracteristics();

        m_dofCaract[m_nDof-1] = m_caract;
    }
    else{
        m_dofCaract = new s2mBoneCaracteristics[1];
        m_dofCaract[0] = m_caract;
    }
}

void s2mBone::setJointAxis(){
        // Définition des axes de rotation
    RigidBodyDynamics::Math::Vector3d axis[3];
    axis[0]  = RigidBodyDynamics::Math::Vector3d(1,0,0); // axe x
    axis[1]  = RigidBodyDynamics::Math::Vector3d(0,1,0); // axe y
    axis[2]  = RigidBodyDynamics::Math::Vector3d(0,0,1); // axe z

    // Déclaration des dof de translation
    delete[] m_dof;
    if (m_nDof != 0){
        m_dof = new s2mJointIntraBone[m_nDof];
        for (unsigned int i=0; i<m_nDofTrans; i++)
            m_dof[i] = s2mJointIntraBone(RigidBodyDynamics::JointTypePrismatic, axis[m_dofPosition[i]]);

        // Déclaration des dof de rotation
        if (m_isQuaternion)
            m_dof[m_nDofTrans] = s2mJointIntraBone(RigidBodyDynamics::JointTypeSpherical); // Mettre un dof en sphérique
        else
            for (unsigned int i=m_nDofTrans; i<m_nDofRot+m_nDofTrans; i++)
                m_dof[i] = s2mJointIntraBone(RigidBodyDynamics::JointTypeRevolute, axis[m_dofPosition[i]]); // Mettre les axes de rotation dans le bon ordre
    }
    else{
        m_dof = new s2mJointIntraBone[1];
        m_dof[0] = s2mJointIntraBone(RigidBodyDynamics::JointTypeFixed); // Un axe au hasard, p
    }
}

void s2mBone::setJoints(s2mJoints *model){
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

unsigned int s2mBone::getDofIdx(const s2mString &dofName) const{

    unsigned int idx(INT_MAX);
    bool found = false;
    for (unsigned int i=0; i<nDof(); ++i){
        if (!dofName.tolower().compare(m_nomDof[i].tolower())){
            idx = i;
            found = true;
            break;
        }
    }


    s2mError::s2mAssert(found, "Type should be \"Rot\" or \"Trans\" and axis should be \"X\", \"Y\" or \"Z\", e.g. \"RotY\" for Rotation around y or \"TransX\" for Translation on x");

    return idx;

}



