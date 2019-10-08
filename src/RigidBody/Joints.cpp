#define BIORBD_API_EXPORTS
#include "RigidBody/Joints.h"

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>
#include "Utils/String.h"
#include "Utils/Quaternion.h"
#include "Utils/Matrix.h"
#include "Utils/Error.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Integrator.h"
#include "RigidBody/Bone.h"
#include "RigidBody/Markers.h"
#include "RigidBody/NodeBone.h"
#include "RigidBody/Patch.h"
#include "RigidBody/BoneMesh.h"
#include "RigidBody/BoneCaracteristics.h"

biorbd::rigidbody::Joints::Joints() :
    RigidBodyDynamics::Model(),
    m_bones(std::make_shared<std::vector<biorbd::rigidbody::Bone>>()),
    m_integrator(std::make_shared<biorbd::rigidbody::Integrator>()),
    m_nbRoot(std::make_shared<unsigned int>(0)),
    m_nDof(std::make_shared<unsigned int>(0)),
    m_nbQ(std::make_shared<unsigned int>(0)),
    m_nbQdot(std::make_shared<unsigned int>(0)),
    m_nbQddot(std::make_shared<unsigned int>(0)),
    m_nRotAQuat(std::make_shared<unsigned int>(0)),
    m_isRootActuated(std::make_shared<bool>(true)),
    m_hasExternalForces(std::make_shared<bool>(false)),
    m_isKinematicsComputed(std::make_shared<bool>(false)),
    m_totalMass(std::make_shared<double>(0))
{
    this->gravity = RigidBodyDynamics::Math::Vector3d (0, 0, -9.81);  // Redéfinition de la gravité pour qu'elle soit en z
}

biorbd::rigidbody::Joints::Joints(const biorbd::rigidbody::Joints &other) :
    RigidBodyDynamics::Model(other),
    m_bones(other.m_bones),
    m_integrator(other.m_integrator),
    m_nbRoot(other.m_nbRoot),
    m_nDof(other.m_nDof),
    m_nbQ(other.m_nbQ),
    m_nbQdot(other.m_nbQdot),
    m_nbQddot(other.m_nbQddot),
    m_nRotAQuat(other.m_nRotAQuat),
    m_isRootActuated(other.m_isRootActuated),
    m_hasExternalForces(other.m_hasExternalForces),
    m_isKinematicsComputed(other.m_isKinematicsComputed),
    m_totalMass(other.m_totalMass)
{

}

biorbd::rigidbody::Joints::~Joints()
{

}

biorbd::rigidbody::Joints biorbd::rigidbody::Joints::DeepCopy() const
{
    biorbd::rigidbody::Joints copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Joints::DeepCopy(const biorbd::rigidbody::Joints &other)
{
    static_cast<RigidBodyDynamics::Model&>(*this) = other;
    m_bones->resize(other.m_bones->size());
    for (unsigned int i=0; i<other.m_bones->size(); ++i)
        (*m_bones)[i] = (*other.m_bones)[i].DeepCopy();
    *m_integrator = other.m_integrator->DeepCopy();
    *m_nbRoot = *other.m_nbRoot;
    *m_nDof = *other.m_nDof;
    *m_nbQ = *other.m_nbQ;
    *m_nbQdot = *other.m_nbQdot;
    *m_nbQddot = *other.m_nbQddot;
    *m_nRotAQuat = *other.m_nRotAQuat;
    *m_isRootActuated = *other.m_isRootActuated;
    *m_hasExternalForces = *other.m_hasExternalForces;
    *m_isKinematicsComputed = *other.m_isKinematicsComputed;
    *m_totalMass = *other.m_totalMass;
}

unsigned int biorbd::rigidbody::Joints::nbGeneralizedTorque() const {
    return dof_count-nbRoot();
}
unsigned int biorbd::rigidbody::Joints::nbDof() const {
    return *m_nDof;
}

std::vector<std::string> biorbd::rigidbody::Joints::nameDof() const
{
    std::vector<std::string> names;
    for (unsigned int i=0; i<nbBone(); ++i){
        for (unsigned int j=0; j<bone(i).nDof(); ++j){
            names.push_back(bone(i).name() + "_" + bone(i).nameDof(j));
        }
    }
    return names;
}
unsigned int biorbd::rigidbody::Joints::nbQ() const {
    return *m_nbQ;
}
unsigned int biorbd::rigidbody::Joints::nbQdot() const {
    return *m_nbQdot;
}
unsigned int biorbd::rigidbody::Joints::nbQddot() const {
    return *m_nbQddot;
}
unsigned int biorbd::rigidbody::Joints::nbRoot() const {
    if (*m_isRootActuated)
        return 0;
    else
        return *m_nbRoot;
}
void biorbd::rigidbody::Joints::setIsRootActuated(bool a) {
    *m_isRootActuated = a;
}
bool biorbd::rigidbody::Joints::isRootActuated() const {
    return *m_isRootActuated;
}
void biorbd::rigidbody::Joints::setHasExternalForces(bool f) {
    *m_hasExternalForces = f;
}
bool biorbd::rigidbody::Joints::hasExternalForces() const {
    return *m_hasExternalForces;
}
double biorbd::rigidbody::Joints::mass() const {
    return *m_totalMass;
}




void biorbd::rigidbody::Joints::integrateKinematics(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedCoordinates& QDot,
        const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorque)
{
    biorbd::utils::Vector v(static_cast<unsigned int>(Q.rows()+QDot.rows()));
    v << Q,QDot;
    m_integrator->integrate(*this, v, GeneralizedTorque, 0, 1, 0.1); // vecteur, t0, tend, pas, effecteurs
    *m_isKinematicsComputed = true;
}
void biorbd::rigidbody::Joints::getIntegratedKinematics(
        unsigned int step,
        biorbd::rigidbody::GeneralizedCoordinates &Q,
        biorbd::rigidbody::GeneralizedCoordinates &QDot)
{
    // Si la cinématique n'a pas été mise à jour
    biorbd::utils::Error::check(*m_isKinematicsComputed, "ComputeKinematics must be call before calling updateKinematics");

    const biorbd::utils::Vector& tp(m_integrator->getX(step));
    for (unsigned int i=0; i< static_cast<unsigned int>(tp.rows()/2); i++){
        Q(i) = tp(i);
        QDot(i) = tp(i+tp.rows()/2);
    }
}
unsigned int biorbd::rigidbody::Joints::nbInterationStep() const
{
    return m_integrator->steps();
}


unsigned int biorbd::rigidbody::Joints::AddBone(
        const biorbd::utils::String &segmentName, // Nom du segment
        const biorbd::utils::String &parentName, // Nom du segment
        const biorbd::utils::String &translationSequence,
        const biorbd::utils::String &rotationSequence, // Séquence de Cardan pour classer les dof en rotation
        const biorbd::rigidbody::BoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& centreOfRotation, // Transformation du parent vers l'enfant
        int forcePlates)
{ // Numéro de la plateforme de force attaché à cet os
    biorbd::rigidbody::Bone tp(*this, segmentName, parentName, translationSequence, rotationSequence, caract, centreOfRotation, forcePlates);
    if (this->GetBodyId(parentName.c_str()) == std::numeric_limits<unsigned int>::max())
        *m_nbRoot += tp.nDof(); //  Si le nom du segment est "Root" ajouter le nombre de dof de racine
    *m_nDof += tp.nDof();
    *m_nbQ += tp.nQ();
    *m_nbQdot += tp.nQdot();
    *m_nbQddot += tp.nQddot();

    if (tp.isRotationAQuaternion())
        ++*m_nRotAQuat;
		
    *m_totalMass += caract.mMass; // Ajouter la masse segmentaire a la masse totale du corps
    m_bones->push_back(tp);
    return 0;
}
unsigned int biorbd::rigidbody::Joints::AddBone(
        const biorbd::utils::String &segmentName, // Nom du segment
        const biorbd::utils::String &parentName, // Nom du segment
        const biorbd::utils::String &seqR, // Séquence de Cardan pour classer les dof en rotation
        const biorbd::rigidbody::BoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
        const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
        int forcePlates)
{ // Numéro de la plateforme de force attaché à cet os
    biorbd::rigidbody::Bone tp(*this, segmentName, parentName, seqR, caract, cor, forcePlates);
    if (this->GetBodyId(parentName.c_str()) == std::numeric_limits<unsigned int>::max())
        *m_nbRoot += tp.nDof(); //  Si le nom du segment est "Root" ajouter le nombre de dof de racine
    *m_nDof += tp.nDof();
	
    *m_totalMass += caract.mMass; // Ajouter la masse segmentaire a la masse totale du corps
    m_bones->push_back(tp);
    return 0;
}

const biorbd::rigidbody::Bone& biorbd::rigidbody::Joints::bone(unsigned int idxSegment) const {
    biorbd::utils::Error::check(idxSegment < m_bones->size(), "Asked for a wrong segment (out of range)");
    return (*m_bones)[idxSegment];
}

const biorbd::rigidbody::Bone &biorbd::rigidbody::Joints::bone(const biorbd::utils::String & nameSegment) const
{
    return bone(static_cast<unsigned int>(GetBodyBiorbdId(nameSegment.c_str())));
}

unsigned int biorbd::rigidbody::Joints::nbBone() const
{
     return static_cast<unsigned int>(m_bones->size());
}

std::vector<RigidBodyDynamics::Math::SpatialVector> biorbd::rigidbody::Joints::dispatchedForce(
        std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>> &spatialVector,
        unsigned int frame) const
{
    // Itérateur sur le tableau de force
    std::vector<RigidBodyDynamics::Math::SpatialVector> sv2; // Mettre dans un même tableau les valeurs d'un même instant de différentes plateformes
    for (auto vec : spatialVector)
        sv2.push_back(vec[frame]);

    // Appel de la fonction équivalente qui ne gere qu'a un instant
    return dispatchedForce(sv2);
}

std::vector<RigidBodyDynamics::Math::SpatialVector> biorbd::rigidbody::Joints::dispatchedForce(
        std::vector<RigidBodyDynamics::Math::SpatialVector> &sv) const{ // un SpatialVector par PF
    // Tableau de sortie
    std::vector<RigidBodyDynamics::Math::SpatialVector> sv_out;

    // Spatial vector nul pour remplir le tableau final
    RigidBodyDynamics::Math::SpatialVector sv_zero(0,0,0,0,0,0);
    sv_out.push_back(sv_zero); // Le premier est associé a l'univers

    // Dispatch des forces
    for (auto bone : *m_bones){
        unsigned int nDof = bone.nDof();
        if (nDof != 0){ // Ne rien ajouter si le nDof est à 0
            // Pour chaque segment,
            for (unsigned int i=0; i<nDof-1; ++i) // mettre un sv_zero sur tous les dof sauf le dernier
                sv_out.push_back(sv_zero);
            if (bone.plateformeIdx() >= 0){ // Si le solide fait contact avec la plateforme (!= -1)
                sv_out.push_back(sv[static_cast<unsigned int>(bone.plateformeIdx())]); // Mettre la force de la plateforme correspondante
            }
            else
                sv_out.push_back(sv_zero); // Sinon, mettre 0
        }
    }

    // Retour du STL vector de SpatialVector
    return sv_out;
}

int biorbd::rigidbody::Joints::GetBodyBiorbdId(const biorbd::utils::String &segmentName) const{
    for (int i=0; i<static_cast<int>(m_bones->size()); ++i)
        if (!(*m_bones)[static_cast<unsigned int>(i)].name().compare(segmentName))
            return i;
    return -1;
}

std::vector<biorbd::utils::RotoTrans> biorbd::rigidbody::Joints::allGlobalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return allGlobalJCS();
}

std::vector<biorbd::utils::RotoTrans> biorbd::rigidbody::Joints::allGlobalJCS() const
{
    std::vector<biorbd::utils::RotoTrans> out;
    for (unsigned int i=0; i<m_bones->size(); ++i)
        out.push_back(globalJCS(i));
    return out;
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::utils::String &name)
{
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return globalJCS(name);
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx)
{
    // update the Kinematics if necessary
    UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return globalJCS(idx);
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(const biorbd::utils::String &parentName) const
{
    return globalJCS(static_cast<unsigned int>(GetBodyBiorbdId(parentName)));
}

biorbd::utils::RotoTrans biorbd::rigidbody::Joints::globalJCS(
        unsigned int idx) const
{
    return CalcBodyWorldTransformation((*m_bones)[idx].id());
}

std::vector<biorbd::utils::RotoTrans> biorbd::rigidbody::Joints::localJCS() const{
    std::vector<biorbd::utils::RotoTrans> out;

    for (unsigned int i=0; i<m_bones->size(); ++i)
            out.push_back(localJCS(i));

    return out;
}
biorbd::utils::RotoTrans biorbd::rigidbody::Joints::localJCS(const biorbd::utils::String &segmentName) const{
    return localJCS(static_cast<unsigned int>(GetBodyBiorbdId(segmentName.c_str())));
}
biorbd::utils::RotoTrans biorbd::rigidbody::Joints::localJCS(const unsigned int i) const{
    return (*m_bones)[i].localJCS();
}


std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Joints::projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const std::vector<biorbd::rigidbody::NodeBone> &v,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);
    updateKin = false;

    // Assuming that this is also a marker type (via BiorbdModel)
    const biorbd::rigidbody::Markers &marks = dynamic_cast<biorbd::rigidbody::Markers &>(*this);

    // Sécurité
    biorbd::utils::Error::check(marks.nMarkers() == v.size(), "Number of marker must be equal to number of Vector3d");

    std::vector<biorbd::rigidbody::NodeBone> out;
    for (unsigned int i=0;i<marks.nMarkers();++i){
        biorbd::rigidbody::NodeBone tp(marks.marker(i));
        if (tp.nAxesToRemove()!=0){
            tp = v[i].applyRT(globalJCS(tp.parent()).transpose());
            // Prendre la position du nouveau marker avec les infos de celui du modèle
            out.push_back(projectPoint(Q,tp,updateKin));
        }
        else
            // S'il ne faut rien retirer (renvoyer tout de suite la même position)
            out.push_back( v[i] );
    }
    return out;
}

biorbd::rigidbody::NodeBone biorbd::rigidbody::Joints::projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const utils::Node3d &v,
        int boneIdx,
        const biorbd::utils::String& axesToRemove,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    // Créer un marqueur
    const biorbd::utils::String& boneName(bone(static_cast<unsigned int>(boneIdx)).name());
    biorbd::rigidbody::NodeBone node( v.applyRT(globalJCS(static_cast<unsigned int>(boneIdx)).transpose()), "tp", boneName,
                     true, true, axesToRemove, static_cast<int>(GetBodyId(boneName.c_str())));

    // projeté puis remettre dans le global
    return projectPoint(Q, node, updateKin);
}

biorbd::rigidbody::NodeBone biorbd::rigidbody::Joints::projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::NodeBone &n,
        bool updateKin)
{
    // Assuming that this is also a Marker type (via BiorbdModel)
    return dynamic_cast<biorbd::rigidbody::Markers &>(*this).marker(Q, n, true, updateKin);
}

biorbd::utils::Matrix biorbd::rigidbody::Joints::projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        biorbd::rigidbody::NodeBone node,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    // Assuming that this is also a Marker type (via BiorbdModel)
    biorbd::rigidbody::Markers &marks = dynamic_cast<biorbd::rigidbody::Markers &>(*this);

    // Si le point n'a pas été projeté, il n'y a donc aucun effet
    if (node.nAxesToRemove() != 0){
        // Jacobienne du marqueur
        node.applyRT(globalJCS(node.parent()).transpose());
        biorbd::utils::Matrix G_tp(marks.markersJacobian(Q, node.parent(), biorbd::utils::Node3d(0,0,0), updateKin));
        biorbd::utils::Matrix JCor(biorbd::utils::Matrix::Zero(9,nbQ()));
        CalcMatRotJacobian(Q, GetBodyId(node.parent().c_str()), Eigen::Matrix3d::Identity(3,3), JCor,false);
        for (unsigned int n=0; n<3; ++n)
            if (node.isAxisKept(n))
                G_tp += JCor.block(n*3,0,3,nbQ()) * node(n);

        return G_tp;
    }
    else {
        // Retourner la valeur
        return biorbd::utils::Matrix::Zero(3,nbQ());
    }
}

biorbd::utils::Matrix biorbd::rigidbody::Joints::projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const utils::Node3d &v,
        int boneIdx,
        const biorbd::utils::String& axesToRemove,
        bool updateKin)
{
    // Trouver le point
    const biorbd::rigidbody::NodeBone& p(projectPoint(Q, v, boneIdx, axesToRemove, updateKin));

    // Retourner la valeur
    return projectPointJacobian(Q, p, updateKin);
}

std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Joints::projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const std::vector<biorbd::rigidbody::NodeBone> &v,
        bool updateKin)
{
    // Receuillir les points
    const std::vector<biorbd::rigidbody::NodeBone>& tp(projectPoint(Q, v, updateKin));

    // Calculer la jacobienne si le point doit être projeté
    std::vector<biorbd::utils::Matrix> G;

    for (unsigned int i=0; i<tp.size(); ++i){
        // Marqueur actuel
        G.push_back(projectPointJacobian(Q, biorbd::rigidbody::NodeBone(v[i]), false));
    }
    return G;
}

RigidBodyDynamics::Math::SpatialTransform biorbd::rigidbody::Joints::CalcBodyWorldTransformation (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const unsigned int body_id,
        bool update_kinematics = true)
{
    // update the Kinematics if necessary
    if (update_kinematics)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);
    return CalcBodyWorldTransformation(body_id);
}

RigidBodyDynamics::Math::SpatialTransform biorbd::rigidbody::Joints::CalcBodyWorldTransformation(
        const unsigned int body_id) const
{

    if (body_id >= this->fixed_body_discriminator) {
        unsigned int fbody_id = body_id - this->fixed_body_discriminator;
        unsigned int parent_id = this->mFixedBodies[fbody_id].mMovableParent;
        biorbd::utils::RotoTrans parentRT(this->X_base[parent_id].E.transpose(), this->X_base[parent_id].r);
        biorbd::utils::RotoTrans bodyRT(this->mFixedBodies[fbody_id].mParentTransform.E.transpose(), this->mFixedBodies[fbody_id].mParentTransform.r);
        const biorbd::utils::RotoTrans& transfo_tp = parentRT * bodyRT;
        return RigidBodyDynamics::Math::SpatialTransform (transfo_tp.rot(), transfo_tp.trans());
    }

    return RigidBodyDynamics::Math::SpatialTransform (this->X_base[body_id].E.transpose(), this->X_base[body_id].r);
}

biorbd::utils::Node3d biorbd::rigidbody::Joints::CoM(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    // Retour la position du centre de masse a partir des coordonnées généralisées

    // S'assurer que le modele est dans la bonne configuration
    if (updateKin)
        UpdateKinematicsCustom(&Q, nullptr, nullptr);

    // Pour chaque segment, trouver le CoM (CoM = somme(masse_segmentaire * pos_com_seg)/masse_totale)
    const std::vector<biorbd::rigidbody::NodeBone>& com_segment(CoMbySegment(Q,true));
    biorbd::utils::Node3d com(0, 0, 0);
    for (unsigned int i=0; i<com_segment.size(); ++i)
        com += (*m_bones)[i].caract().mMass * com_segment[i];

    // Diviser par la masse totale
    com = com/this->mass();

    // Retourner le CoM
    return com;
}

RigidBodyDynamics::Math::Vector3d biorbd::rigidbody::Joints::angularMomentum(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool updateKin)
{
    return CalcAngularMomentum(*this, Q, Qdot, updateKin);
}


RigidBodyDynamics::Math::Vector3d biorbd::rigidbody::Joints::CoMdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot)
{
    // Retour la vitesse du centre de masse a partir des coordonnées généralisées

    // S'assurer que le modele est dans la bonne configuration
    UpdateKinematicsCustom(&Q, &Qdot, nullptr);

    // Pour chaque segment, trouver le CoM
    RigidBodyDynamics::Math::Vector3d com_dot(0,0,0);

    // CoMdot = somme(masse_seg * Jacobienne * qdot)/masse totale
    biorbd::utils::Matrix Jac(biorbd::utils::Matrix(3,this->dof_count));
    for (auto bone : *m_bones){
        Jac.setZero();
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId(bone.name().c_str()), bone.caract().mCenterOfMass, Jac, false); // False for speed
        com_dot += ((Jac*Qdot) * bone.caract().mMass);
    }
    // Diviser par la masse totale
    com_dot = com_dot/this->mass();
	
    // Retourner la vitesse du CoM
    return com_dot;
}
RigidBodyDynamics::Math::Vector3d biorbd::rigidbody::Joints::CoMddot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot)
{
    // Retour l'accélération du centre de masse a partir des coordonnées généralisées
    biorbd::utils::Error::raise("Com DDot is wrong, to be modified...");

    // S'assurer que le modele est dans la bonne configuration
    UpdateKinematicsCustom(&Q, &Qdot, &Qddot);

    // Pour chaque segment, trouver le CoM
    RigidBodyDynamics::Math::Vector3d com_ddot(0,0,0);

    // CoMdot = somme(masse_seg * Jacobienne * qdot)/masse totale
    biorbd::utils::Matrix Jac(biorbd::utils::Matrix(3,this->dof_count));
    for (auto bone : *m_bones){
        Jac.setZero();
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId(bone.name().c_str()), bone.caract().mCenterOfMass, Jac, false); // False for speed
        com_ddot += ((Jac*Qddot) * bone.caract().mMass);
    }
    // Diviser par la masse totale
    com_ddot = com_ddot/this->mass();

    // Retourner l'accélération du CoM
    return com_ddot;
}

biorbd::utils::Matrix biorbd::rigidbody::Joints::CoMJacobian(const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    // Retour la position du centre de masse a partir des coordonnées généralisées

    // S'assurer que le modele est dans la bonne configuration
    UpdateKinematicsCustom(&Q, nullptr, nullptr);

   // Jacobienne totale
    biorbd::utils::Matrix JacTotal(biorbd::utils::Matrix::Zero(3,this->dof_count));

    // CoMdot = somme(masse_seg * Jacobienne * qdot)/masse totale
    biorbd::utils::Matrix Jac(biorbd::utils::Matrix::Zero(3,this->dof_count));
    for (auto bone : *m_bones){
        Jac.setZero();
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId(bone.name().c_str()), bone.caract().mCenterOfMass, Jac, false); // False for speed
        JacTotal += bone.caract().mMass*Jac;
    }

    // Diviser par la masse totale
    JacTotal /= this->mass();

    // Retourner la jacobienne du CoM
    return JacTotal;
}


std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Joints::CoMbySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{// Position du centre de masse de chaque segment
    std::vector<biorbd::rigidbody::NodeBone> tp; // vecteur de vecteurs de sortie

    for (unsigned int i=0; i<m_bones->size(); ++i){
        tp.push_back(CoMbySegment(Q,i,updateKin));
        updateKin = false; // ne le faire que la premiere fois
    }
    return tp;
}


biorbd::utils::Node3d biorbd::rigidbody::Joints::CoMbySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const unsigned int i,
        bool updateKin)
{ // Position du centre de masse du segment i
    biorbd::utils::Error::check(i < m_bones->size(), "Choosen segment doesn't exist");
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
                *this, Q, (*m_bones)[i].id(), (*m_bones)[i].caract().mCenterOfMass, updateKin);
}


std::vector<RigidBodyDynamics::Math::Vector3d> biorbd::rigidbody::Joints::CoMdotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool updateKin)
{// Position du centre de masse de chaque segment
    std::vector<RigidBodyDynamics::Math::Vector3d> tp; // vecteur de vecteurs de sortie

    for (unsigned int i=0; i<m_bones->size(); ++i){
        tp.push_back(CoMdotBySegment(Q,Qdot,i,updateKin));
        updateKin = false; // ne le faire que la premiere fois
    }
    return tp;
}


RigidBodyDynamics::Math::Vector3d biorbd::rigidbody::Joints::CoMdotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const unsigned int i,
        bool updateKin)
{ // Position du centre de masse du segment i
    biorbd::utils::Error::check(i < m_bones->size(), "Choosen segment doesn't exist");
    return CalcPointVelocity(*this, Q, Qdot, (*m_bones)[i].id(),(*m_bones)[i].caract().mCenterOfMass,updateKin);
}


std::vector<RigidBodyDynamics::Math::Vector3d> biorbd::rigidbody::Joints::CoMddotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        bool updateKin)
{// Position du centre de masse de chaque segment
    std::vector<RigidBodyDynamics::Math::Vector3d> tp; // vecteur de vecteurs de sortie

    for (unsigned int i=0; i<m_bones->size(); ++i){
        tp.push_back(CoMddotBySegment(Q,Qdot,Qddot,i,updateKin));
        updateKin = false; // ne le faire que la premiere fois
    }
    return tp;
}


RigidBodyDynamics::Math::Vector3d biorbd::rigidbody::Joints::CoMddotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        const unsigned int i,
        bool updateKin)
{ // Position du centre de masse du segment i
    biorbd::utils::Error::check(i < m_bones->size(), "Choosen segment doesn't exist");
    return RigidBodyDynamics::CalcPointAcceleration(*this, Q, Qdot, Qddot, (*m_bones)[i].id(),(*m_bones)[i].caract().mCenterOfMass,updateKin);
}

std::vector<std::vector<biorbd::utils::Node3d>> biorbd::rigidbody::Joints::meshPoints(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    std::vector<std::vector<biorbd::utils::Node3d>> v; // Vecteur de vecteur de sortie (mesh par segment)

    // Trouver la position des segments
    const std::vector<biorbd::utils::RotoTrans>& RT(allGlobalJCS());

    // Pour tous les segments
    for (unsigned int i=0; i<nbBone(); ++i)
        v.push_back(meshPoints(RT,i));

    return v;
}
std::vector<biorbd::utils::Node3d> biorbd::rigidbody::Joints::meshPoints(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int i,
        bool updateKin)
{
    if (updateKin)
        UpdateKinematicsCustom (&Q, nullptr, nullptr);

    // Trouver la position des segments
    const std::vector<biorbd::utils::RotoTrans>& RT(allGlobalJCS());

    return meshPoints(RT,i);
}
std::vector<biorbd::utils::Node3d> biorbd::rigidbody::Joints::meshPoints(
        const std::vector<biorbd::utils::RotoTrans> &RT,
        unsigned int i) const{

    // Recueillir la position des meshings
    std::vector<biorbd::utils::Node3d> v;
    for (unsigned int j=0; j<boneMesh(i).size(); ++j){
        biorbd::utils::Node3d tp (boneMesh(i).point(j));
        tp.applyRT(RT[i]);
        v.push_back(tp);
    }

    return v;
}
std::vector<std::vector<biorbd::rigidbody::Patch>> biorbd::rigidbody::Joints::meshPatch() const{
    // Recueillir la position des meshings pour tous les segments
    std::vector<std::vector<biorbd::rigidbody::Patch>> v_all;
    for (unsigned int j=0; j<nbBone(); ++j)
        v_all.push_back(meshPatch(j));
    return v_all;
}
const std::vector<biorbd::rigidbody::Patch> &biorbd::rigidbody::Joints::meshPatch(unsigned int i) const{
    // Recueillir la position des meshings pour un segment i
    return boneMesh(i).patch();
}

std::vector<biorbd::rigidbody::BoneMesh> biorbd::rigidbody::Joints::boneMesh() const
{
    std::vector<biorbd::rigidbody::BoneMesh> boneOut;
    for (unsigned int i=0; i<nbBone(); ++i)
        boneOut.push_back(boneMesh(i));
    return boneOut;
}

const biorbd::rigidbody::BoneMesh &biorbd::rigidbody::Joints::boneMesh(unsigned int idx) const
{
    return bone(idx).caract().mesh();
}

RigidBodyDynamics::Math::Vector3d biorbd::rigidbody::Joints::CalcAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool update_kinematics)
{
    // Qddot was added later in the RBDL. In order to keep backward compatilibity, 
    biorbd::rigidbody::GeneralizedCoordinates Qddot(this->nbQddot());
    return CalcAngularMomentum(Q, Qdot, Qddot, update_kinematics);
}
RigidBodyDynamics::Math::Vector3d biorbd::rigidbody::Joints::CalcAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        bool update_kinematics)
{
    // Définition des variables
    RigidBodyDynamics::Math::Vector3d com,  angular_momentum;
    double mass;

    // Calcul du angular momentum par la fonction de la position du centre de masse
    RigidBodyDynamics::Utils::CalcCenterOfMass(*this, Q, Qdot, &Qddot, mass, com, nullptr, nullptr, &angular_momentum, nullptr, update_kinematics);

    return angular_momentum;
}

std::vector<RigidBodyDynamics::Math::Vector3d> biorbd::rigidbody::Joints::CalcSegmentsAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool update_kinematics) {
    biorbd::rigidbody::GeneralizedCoordinates Qddot(this->nbQddot());
    return CalcSegmentsAngularMomentum(Q, Qdot, Qddot, update_kinematics);
}

std::vector<RigidBodyDynamics::Math::Vector3d> biorbd::rigidbody::Joints::CalcSegmentsAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates &Qddot,
        bool update_kinematics) {
    if (update_kinematics)
        UpdateKinematicsCustom (&Q, &Qdot, &Qddot);

    double mass;
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Utils::CalcCenterOfMass (*this, Q, Qdot, &Qddot, mass, com, nullptr, nullptr, nullptr, nullptr, false);
    RigidBodyDynamics::Math::SpatialTransform X_to_COM (RigidBodyDynamics::Math::Xtrans(com));

    std::vector<RigidBodyDynamics::Math::Vector3d> h_segment;
    for (unsigned int i = 1; i < this->mBodies.size(); i++) {
        this->Ic[i] = this->I[i];
        this->hc[i] = this->Ic[i].toMatrix() * this->v[i];

        RigidBodyDynamics::Math::SpatialVector h = this->X_lambda[i].applyTranspose (this->hc[i]);
        if (this->lambda[i] != 0) {
            unsigned int j(i);
            do {
                j = this->lambda[j];
                h = this->X_lambda[j].applyTranspose (h);
            } while (this->lambda[j]!=0);
        }
        h = X_to_COM.applyAdjoint (h);
        h_segment.push_back(RigidBodyDynamics::Math::Vector3d(h[0],h[1],h[2]));
    }


    return h_segment;
}

void biorbd::rigidbody::Joints::ForwardDynamicsContactsLagrangian (
        const RigidBodyDynamics::Math::VectorNd &Q,
        const RigidBodyDynamics::Math::VectorNd &QDot,
        const RigidBodyDynamics::Math::VectorNd &GeneralizedTorque,
        RigidBodyDynamics::ConstraintSet &CS,
        RigidBodyDynamics::Math::VectorNd &QDDot
     ) {

   // Compute C
   CS.QDDot_0.setZero();
   RigidBodyDynamics::InverseDynamics (*this, Q, QDot, CS.QDDot_0, CS.C);

   // Compute H
   CS.H = RigidBodyDynamics::Math::MatrixNd::Zero(this->dof_count, this->dof_count);
   RigidBodyDynamics::CompositeRigidBodyAlgorithm (*this, Q, CS.H, false);

   // Compute G
   unsigned int i,j;

   // variables to check whether we need to recompute G
   unsigned int prev_body_id = 0;
   RigidBodyDynamics::Math::Vector3d prev_body_point = RigidBodyDynamics::Math::Vector3d::Zero();
   RigidBodyDynamics::Math::MatrixNd Gi (RigidBodyDynamics::Math::MatrixNd::Zero(3, this->dof_count));

   for (i = 0; i < CS.size(); i++) {
     // Only alow contact normals along the coordinate axes
//     unsigned int axis_index = 0;

     // only compute the matrix Gi if actually needed
     if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
        RigidBodyDynamics::CalcPointJacobian (*this, Q, CS.body[i], CS.point[i], Gi, false);
       prev_body_id = CS.body[i];
       prev_body_point = CS.point[i];
     }

     for (j = 0; j < this->dof_count; j++) {
       RigidBodyDynamics::Math::Vector3d gaxis (Gi(0,j), Gi(1,j), Gi(2,j));
       CS.G(i,j) = gaxis.transpose() * CS.normal[i];
 //      CS.G(i,j) = Gi(axis_index, j);
     }
  }

   // Compute gamma
   prev_body_id = 0;
   prev_body_point = RigidBodyDynamics::Math::Vector3d::Zero();
   RigidBodyDynamics::Math::Vector3d gamma_i = RigidBodyDynamics::Math::Vector3d::Zero();

   // update Kinematics just once
   UpdateKinematics (*this, Q, QDot, CS.QDDot_0);

   for (i = 0; i < CS.size(); i++) {
     // Only alow contact normals along the coordinate axes
     unsigned int axis_index = 0;

     if (CS.normal[i] == RigidBodyDynamics::Math::Vector3d(1., 0., 0.))
       axis_index = 0;
    else if (CS.normal[i] == RigidBodyDynamics::Math::Vector3d(0., 1., 0.))
       axis_index = 1;
    else if (CS.normal[i] == RigidBodyDynamics::Math::Vector3d(0., 0., 1.))
       axis_index = 2;
    else
       biorbd::utils::Error::raise("Invalid contact normal axis!");

        // only compute point accelerations when necessary
     if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
       gamma_i = CalcPointAcceleration (*this, Q, QDot, CS.QDDot_0, CS.body[i], CS.point[i], false);
       prev_body_id = CS.body[i];
       prev_body_point = CS.point[i];
     }

    // we also substract ContactData[i].acceleration such that the contact
    // point will have the desired acceleration
     CS.gamma[i] = gamma_i[axis_index] - CS.acceleration[i];
   }

   // Build the system
   CS.A.setZero();
  CS.b.setZero();
  CS.x.setZero();

   // Build the system: Copy H
  for (i = 0; i < this->dof_count; i++) {
     for (j = 0; j < this->dof_count; j++) {
       CS.A(i,j) = CS.H(i,j);
     }
  }

   // Build the system: Copy G, and G^T
   for (i = 0; i < CS.size(); i++) {
     for (j = 0; j < this->dof_count; j++) {
       CS.A(i + this->dof_count, j) = CS.G (i,j);
       CS.A(j, i + this->dof_count) = CS.G (i,j);
     }
   }

   // Build the system: Copy -C + \GeneralizedTorque
   for (i = 0; i < this->dof_count; i++) {
    CS.b[i] = -CS.C[i] + GeneralizedTorque[i];
   }

   // Build the system: Copy -gamma
   for (i = 0; i < CS.size(); i++) {
     CS.b[i + this->dof_count] = - CS.gamma[i];
   }

//   std::cout << "A = " << std::endl << CS.A << std::endl;
   //std::cout << "b = " << std::endl << CS.b << std::endl;

  switch (CS.linear_solver) {
    case (RigidBodyDynamics::Math::LinearSolverPartialPivLU) :
     CS.x = CS.A.partialPivLu().solve(CS.b);
      break;
  case (RigidBodyDynamics::Math::LinearSolverColPivHouseholderQR) :
       CS.x = CS.A.colPivHouseholderQr().solve(CS.b);
       break;
     default:
#ifdef RBDL_ENABLE_LOGGING
       LOG << "Error: Invalid linear solver: " << CS.linear_solver << std::endl;
#endif
       biorbd::utils::Error::raise("Error: Invalid linear solver");
     break;
   }

   //std::cout << "x = " << std::endl << CS.x << std::endl;

   // Copy back QDDot
for (i = 0; i < this->dof_count; i++)
     QDDot[i] = CS.x[i];

  // Copy back contact forces
  for (i = 0; i < CS.size(); i++) {
     CS.force[i] = -CS.x[this->dof_count + i];
   }
 }

unsigned int biorbd::rigidbody::Joints::nbQuat() const{
    return *m_nRotAQuat;
}

void biorbd::rigidbody::Joints::computeQdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &QDot,
        biorbd::rigidbody::GeneralizedCoordinates &QDotOut)
{
    // Vérifier s'il y a des quaternions, sinon la dérivée est directement QDot
    if (!m_nRotAQuat){
        QDotOut = QDot;
        return;
    }

    QDotOut.resize(Q.size()); // Créer un vecteur vide de la dimension finale.
    unsigned int cmpQuat(0);
    unsigned int cmpDof(0);
    for (unsigned int i=0; i<nbBone(); ++i){
        biorbd::rigidbody::Bone bone_i=bone(i);
        if (bone_i.isRotationAQuaternion()){
            // Extraire le quaternion
            biorbd::utils::Quaternion quat_tp(Q.block(cmpDof+bone_i.nDofTrans(), 0, 3, 1), Q(Q.size()-*m_nRotAQuat+cmpQuat));

            // Placer dans le vecteur de sortie
            QDotOut.block(cmpDof, 0, bone_i.nDofTrans(), 1) = QDot.block(cmpDof, 0, bone_i.nDofTrans(), 1); // La dérivée des translations est celle directement de qdot

            // Dériver
            quat_tp.derivate(QDot.block(cmpDof+bone_i.nDofTrans(), 0, 3, 1));
            QDotOut.block(cmpDof+bone_i.nDofTrans(), 0, 3, 1) = quat_tp.block(0,0,3,1);
            QDotOut(Q.size()-*m_nRotAQuat+cmpQuat) = quat_tp(3);// Placer dans le vecteur de sortie

           // Incrémenter le nombre de quaternions faits
            ++cmpQuat;
        }
        else{
            // Si c'est un normal, faire ce qu'il est fait d'habitude
            QDotOut.block(cmpDof, 0, bone_i.nDof(), 1) = QDot.block(cmpDof, 0, bone_i.nDof(), 1);
        }
        cmpDof += bone_i.nDof();
    }

}



unsigned int biorbd::rigidbody::Joints::getDofIndex(
        const biorbd::utils::String& boneName,
        const biorbd::utils::String& dofName)
{
    unsigned int idx = 0;

    unsigned int iB = 0;
    bool found = false;
    while (1){
        biorbd::utils::Error::check(iB!=m_bones->size(), "Bone not found");

        if (boneName.compare(  (*m_bones)[iB].name() )   )
            idx +=  (*m_bones)[iB].nDof();
        else{
            idx += (*m_bones)[iB].getDofIdx(dofName);
            found = true;
            break;
        }

        ++iB;
    }

    biorbd::utils::Error::check(found, "Dof not found");
    return idx;
}

void biorbd::rigidbody::Joints::UpdateKinematicsCustom(
        const biorbd::rigidbody::GeneralizedCoordinates *Q,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates *Qddot)
{
    RigidBodyDynamics::UpdateKinematicsCustom(*this, Q, Qdot, Qddot);
}

void biorbd::rigidbody::Joints::CalcMatRotJacobian(
        const RigidBodyDynamics::Math::VectorNd &Q,
        unsigned int body_id,
        const RigidBodyDynamics::Math::Matrix3d &rotation,
        RigidBodyDynamics::Math::MatrixNd &G,
        bool update_kinematics)
{
#ifdef RBDL_ENABLE_LOGGING
    LOG << "-------- " << __func__ << " --------" << std::endl;
#endif

    // update the Kinematics if necessary
    if (update_kinematics) {
        RigidBodyDynamics::UpdateKinematicsCustom (*this, &Q, nullptr, nullptr);
    }

    assert (G.rows() == 9 && G.cols() == this->qdot_size );

    std::vector<biorbd::utils::Node3d> axes;
    axes.push_back(biorbd::utils::Node3d(1,0,0));
    axes.push_back(biorbd::utils::Node3d(0,1,0));
    axes.push_back(biorbd::utils::Node3d(0,0,1));
    for (unsigned int iAxes=0; iAxes<3; ++iAxes){
        RigidBodyDynamics::Math::Matrix3d bodyMatRot (
                    RigidBodyDynamics::CalcBodyWorldOrientation (*this, Q, body_id, false).transpose());
        RigidBodyDynamics::Math::SpatialTransform point_trans(
                    RigidBodyDynamics::Math::SpatialTransform (
                        RigidBodyDynamics::Math::Matrix3d::Identity(), bodyMatRot * rotation * *(axes.begin()+iAxes)));


        unsigned int reference_body_id = body_id;

        if (this->IsFixedBodyId(body_id)) {
            unsigned int fbody_id = body_id - this->fixed_body_discriminator;
            reference_body_id = this->mFixedBodies[fbody_id].mMovableParent;
        }

        unsigned int j = reference_body_id;

        // e[j] is set to 1 if joint j contributes to the jacobian that we are
        // computing. For all other joints the column will be zero.
        while (j != 0) {
            unsigned int q_index = this->mJoints[j].q_index;
            // Si ce n'est pas un dof en translation (3 4 5 dans this->S)
            if (this->S[j](3)!=1.0 && this->S[j](4)!=1.0 && this->S[j](5)!=1.0)
            {
                RigidBodyDynamics::Math::SpatialTransform X_base = this->X_base[j];
                X_base.r << 0,0,0; // Retirer tout concept de translation (ne garder que la matrice rotation)

                if (this->mJoints[j].mDoFCount == 3) {
                    G.block(iAxes*3, q_index, 3, 3) = ((point_trans * X_base.inverse()).toMatrix() * this->multdof3_S[j]).block(3,0,3,3);
                } else {
                    G.block(iAxes*3,q_index, 3, 1) = point_trans.apply(X_base.inverse().apply(this->S[j])).block(3,0,3,1);
                }
            }
            j = this->lambda[j]; // Passer au segment parent
        }
    }
}










