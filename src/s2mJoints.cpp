#define BIORBD_API_EXPORTS
#include "../include/s2mJoints.h"


s2mJoints::s2mJoints() :
    m_nbRoot(0),
    m_nDof(0),
    m_nbQ(0),
    m_nbQdot(0),
    m_nbQddot(0),
    m_nRotAQuat(0),
    m_isRootActuated(true),
    m_hasExternalForces(false),
    m_isKinematicsComputed(false),
    m_totalMass(0)
{
	rbdl_check_api_version (RBDL_API_VERSION);
    this->gravity = RigidBodyDynamics::Math::Vector3d (0, 0, -9.81);  // Redéfinition de la gravité pour qu'elle soit en z
    integrator = new s2mIntegrator();
    //ctor
}

s2mJoints::s2mJoints(const s2mJoints& j) :
    RigidBodyDynamics::Model(j),
    m_bones(j.m_bones),
    m_nbRoot(j.m_nbRoot),
    m_nDof(j.m_nDof),
    m_nbQ(j.m_nbQ),
    m_nbQdot(j.m_nbQdot),
    m_nbQddot(j.m_nbQddot),
    m_nRotAQuat(j.m_nRotAQuat),
    m_isRootActuated(j.m_isRootActuated),
    m_hasExternalForces(j.m_hasExternalForces),
    m_isKinematicsComputed(j.m_isKinematicsComputed),
    m_totalMass(j.m_totalMass)
{
    integrator = new s2mIntegrator(*(j.integrator));
}

s2mJoints::~s2mJoints()
{
    delete integrator;
}

unsigned int s2mJoints::nbTau() const {
    return dof_count-nbRoot();
}
unsigned int s2mJoints::nbDof() const {
    return m_nDof;
}

std::vector<std::string> s2mJoints::nameDof() const
{
    std::vector<std::string> names;
    for (unsigned int i=0; i<nbBone(); ++i){
        for (unsigned int j=0; j<bone(i).nDof(); ++j){
            names.push_back(bone(i).name() + "_" + bone(i).nameDof(j));
        }
    }
    return names;
}
unsigned int s2mJoints::nbQ() const {
    return m_nbQ;
}
unsigned int s2mJoints::nbQdot() const {
    return m_nbQdot;
}
unsigned int s2mJoints::nbQddot() const {
    return m_nbQddot;
}
unsigned int s2mJoints::nbRoot() const {
    if (m_isRootActuated) return 0; else return m_nbRoot;
}
void s2mJoints::setIsRootActuated(const bool &a) {
    m_isRootActuated = a;
}
bool s2mJoints::isRootActuated() const {
    return m_isRootActuated;
}
void s2mJoints::setHasExternalForces(const bool &f) {
    m_hasExternalForces = f;
}
bool s2mJoints::hasExternalForces() const {
    return m_hasExternalForces;
}
double s2mJoints::mass() const {
    return m_totalMass;
}




void s2mJoints::computeKinematics(const s2mGenCoord& Q, const s2mGenCoord& QDot, const s2mTau& Tau){
    s2mGenCoord v(static_cast<unsigned int>(Q.rows()+QDot.rows()));
    v << Q,QDot;
    integrator->integrate(this, v, Tau.vector(), 0, 1, 0.1); // vecteur, t0, tend, pas, effecteurs
    m_isKinematicsComputed = true;
}
void s2mJoints::kinematics(const unsigned int &step, s2mGenCoord &Q, s2mGenCoord &QDot){
    // Si la cinématique n'a pas été mise à jour
    s2mError::s2mAssert(m_isKinematicsComputed, "ComputeKinematics must be call before calling updateKinematics");

    s2mGenCoord tp(integrator->getX(step));
    for (unsigned int i=0; i< static_cast<unsigned int>(tp.rows()/2); i++){
        Q(i) = tp(i);
        QDot(i) = tp(i+tp.rows()/2);
    }

}

unsigned int s2mJoints::AddBone(const unsigned int &parent_id, // Numéro du parent
                     const s2mString &seqT, const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
                     const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
                     const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
                     const s2mString &name, // Nom du segment
                     const int &PF){ // Numéro de la plateforme de force attaché à cet os
    s2mBone tp(this, parent_id, seqT, seqR, caract, cor, name, PF);
	if (parent_id == 0)
		m_nbRoot += tp.nDof(); //  Si le nom du segment est "Root" ajouter le nombre de dof de racine
	m_nDof += tp.nDof();
    m_nbQ += tp.nQ();
    m_nbQdot += tp.nQdot();
    m_nbQddot += tp.nQddot();

    if (tp.isRotationAQuaternion())
        ++m_nRotAQuat;
		
    m_totalMass += caract.mMass; // Ajouter la masse segmentaire a la masse totale du corps
    m_bones.push_back(tp);
    return 0;
}
unsigned int s2mJoints::AddBone(const unsigned int &parent_id, // Numéro du parent
                     const s2mString &seqR, // Séquence de Cardan pour classer les dof en rotation
                     const s2mBoneCaracteristics& caract, // Mase, Centre de masse du segment, Inertie du segment, etc.
                     const RigidBodyDynamics::Math::SpatialTransform& cor, // Transformation du parent vers l'enfant
                     const s2mString &name, // Nom du segment
                     const int &PF){ // Numéro de la plateforme de force attaché à cet os
    s2mBone tp(this, parent_id, seqR, caract, cor, name, PF);
	if (parent_id == 0)
        m_nbRoot += tp.nDof(); //  Si le nom du segment est "Root" ajouter le nombre de dof de racine
	m_nDof += tp.nDof();
	
    m_totalMass += caract.mMass; // Ajouter la masse segmentaire a la masse totale du corps
    m_bones.push_back(tp);
    return 0;
}

const s2mBone& s2mJoints::bone(unsigned int i) const {
    s2mError::s2mAssert(i < m_bones.size(), "Asked for a wrong segment (out of range)");
    return *(m_bones.begin()+i);
}

const s2mBone &s2mJoints::bone(const s2mString & name) const
{
    return bone(static_cast<unsigned int>(GetBodyS2MId(name.c_str())));
}

unsigned int s2mJoints::nbBone() const
{
     return static_cast<unsigned int>(m_bones.size());
}

std::vector<RigidBodyDynamics::Math::SpatialVector> s2mJoints::dispatchedForce(std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector> > &sv, const unsigned int &frame){
    // Tableau de sortie
    std::vector<RigidBodyDynamics::Math::SpatialVector> sv_out;

    // Spatial vector nul pour remplir le tableau final
    RigidBodyDynamics::Math::SpatialVector sv_zero(0,0,0,0,0,0);

    // Itérateur sur le tableau de force
    std::vector<RigidBodyDynamics::Math::SpatialVector> sv2; // Mettre dans un même tableau les valeurs d'un même instant de différentes plateformes
    for (std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector> >::iterator it = sv.begin(); it!=sv.end(); ++it){
        std::vector<RigidBodyDynamics::Math::SpatialVector>::iterator sv2_tp = (*it).begin();
        sv2.push_back(*(sv2_tp+frame));
    }

    // Appel de la fonction équivalente qui ne gere qu'a un instant
    return dispatchedForce(sv2);
}

std::vector<RigidBodyDynamics::Math::SpatialVector> s2mJoints::dispatchedForce(std::vector<RigidBodyDynamics::Math::SpatialVector> &sv){ // un SpatialVector par PF
    // Tableau de sortie
    std::vector<RigidBodyDynamics::Math::SpatialVector> sv_out;

    // Spatial vector nul pour remplir le tableau final
    RigidBodyDynamics::Math::SpatialVector sv_zero(0,0,0,0,0,0);
    sv_out.push_back(sv_zero); // Le premier est associé a l'univers

    std::vector<RigidBodyDynamics::Math::SpatialVector>::iterator sv_it = sv.begin();
    // Dispatch des forces
    for (std::vector<s2mBone>::iterator it=m_bones.begin(); it!=m_bones.end(); ++it){
        unsigned int nDof = (*it).nDof();
        if (nDof != 0){ // Ne rien ajouter si le nDof est à 0
            // Pour chaque segment,
            for (unsigned int i=0; i<nDof-1; ++i) // mettre un sv_zero sur tous les dof sauf le dernier
                sv_out.push_back(sv_zero);
            if ((*it).plateformeIdx() >= 0){ // Si le solide fait contact avec la plateforme (!= -1)
				sv_out.push_back(*(sv_it + (*it).plateformeIdx())); // Mettre la force de la plateforme correspondante
            }
            else
                sv_out.push_back(sv_zero); // Sinon, mettre 0
        }
    }

    // Retour du STL vector de SpatialVector
    return sv_out;
}

std::vector<s2mAttitude> s2mJoints::globalJCS(const s2mGenCoord &Q, const bool updateKin){
    std::vector<s2mAttitude> out;

    for (unsigned int i=0; i<m_bones.size(); ++i)
        if (i==0)
            out.push_back(globalJCS(Q,i,updateKin));
        else
            out.push_back(globalJCS(Q,i,false));

    return out;
}

int s2mJoints::GetBodyS2MId(const s2mString &s) const{
    for (int i=0; i<static_cast<int>(m_bones.size()); ++i)
        if (!m_bones[static_cast<unsigned int>(i)].name().compare(s))
            return i;
    return -1;
}

s2mAttitude s2mJoints::globalJCS(const s2mGenCoord &Q, const s2mString &name, const bool updateKin){
    return globalJCS(Q,static_cast<unsigned int>(GetBodyS2MId(name.c_str())),updateKin);
}

s2mAttitude s2mJoints::globalJCS(const s2mGenCoord &Q, const unsigned int i, const bool updateKin){
    unsigned int id = m_bones[i].id();

    s2mAttitude RT;
    RigidBodyDynamics::Math::SpatialTransform tp_ST;

    tp_ST = CalcBodyWorldTransformation(*this,Q, id,updateKin);
    RT.block(0,0,3,3) = tp_ST.E;
    RT.block(0,3,3,1) = tp_ST.r;
    RT.block(3,0,1,4) << 0,0,0,1;

    return RT;
}


std::vector<s2mAttitude> s2mJoints::localJCS() const{
    std::vector<s2mAttitude> out;

    for (unsigned int i=0; i<m_bones.size(); ++i)
            out.push_back(localJCS(i));

    return out;
}
s2mAttitude s2mJoints::localJCS(const s2mString &name) const{
    return localJCS(static_cast<unsigned int>(GetBodyS2MId(name.c_str())));
}
s2mAttitude s2mJoints::localJCS(const unsigned int i) const{
    return m_bones[i].localJCS();
}


std::vector<s2mNodeBone> s2mJoints::projectPoint(const s2mMarkers &marks, const s2mGenCoord &Q, const std::vector<Eigen::Vector3d> &v, bool updateKin)
{
    // Sécurité
    s2mError::s2mAssert(marks.nTags() == v.size(), "Number of marker must be equal to number of Vector3d");

    std::vector<s2mNodeBone> out;
    for (unsigned int i=0;i<marks.nTags();++i){
        s2mNodeBone tp(marks.marker(i));
        if (tp.nAxesToRemove()!=0){
            tp.setPosition(globalJCS(Q,static_cast<unsigned int>(GetBodyS2MId(tp.parent())),true).transpose()* (*(v.begin()+i)) );
            // Prendre la position du nouveau marker avec les infos de celui du modèle
            out.push_back(projectPoint(Q,tp,updateKin));
            updateKin = false;
        }
        else
            // S'il ne faut rien retirer (renvoyer tout de suite la même position)
            out.push_back( *(v.begin()+i) );
    }
    return out;
}

s2mNodeBone s2mJoints::projectPoint(const s2mGenCoord &Q, const Eigen::Vector3d &v, int boneIdx, const s2mString& axesToRemove, bool updateKin)
{
    // Créer un marqueur
    s2mString boneName(bone(static_cast<unsigned int>(boneIdx)).name());
    s2mNodeBone node(globalJCS(Q,static_cast<unsigned int>(boneIdx),true).transpose()*v, "tp", boneName,
                     true, true, axesToRemove, static_cast<int>(GetBodyId(boneName.c_str())));

    // projeté puis remettre dans le global
    return projectPoint(Q, node, updateKin);
}

s2mNodeBone s2mJoints::projectPoint(const s2mGenCoord &Q, const s2mNodeBone &n, bool updateKin)
{
    s2mNodeBone out(n);
    out.setPosition(s2mMarkers::Tags(*this, Q, n, true, updateKin));
    return out;
}

s2mMatrix s2mJoints::projectPointJacobian(s2mJoints& model, const s2mGenCoord &Q, s2mNodeBone node, bool updateKin)
{
    // Si le point n'a pas été projeté, il n'y a donc aucun effet
    if (node.nAxesToRemove() != 0){
        // Jacobienne du marqueur
        node.applyRT(globalJCS(Q, node.parent(),updateKin).transpose());
        s2mMatrix G_tp(s2mMarkers::TagsJacobian(model, Q, node.parent(), Eigen::Vector3d(0,0,0), updateKin));
        s2mMatrix JCor(s2mMatrix::Zero(9,nbQ()));
        CalcMatRotJacobian(model, Q, GetBodyId(node.parent().c_str()), Eigen::Matrix3d::Identity(3,3), JCor,false);
        for (int n=0; n<3; ++n)
            if (node.isAxisKept(n))
                G_tp += JCor.block(n*3,0,3,nbQ())* node(n);

        return G_tp;
    }
    else
        // Retourner la valeur
        return s2mMatrix(s2mMatrix::Zero(3,nbQ()));
}

s2mMatrix s2mJoints::projectPointJacobian(s2mJoints& model, const s2mGenCoord &Q, const Eigen::Vector3d &v, int boneIdx, const s2mString& axesToRemove, bool updateKin)
{
    // Trouver le point
    s2mNodeBone p(projectPoint(Q, v, boneIdx, axesToRemove, updateKin));

    // Retourner la valeur
    return projectPointJacobian(model, Q, p, updateKin);
}

std::vector<s2mMatrix> s2mJoints::projectPointJacobian(s2mJoints& model, const s2mMarkers &marks, const s2mGenCoord &Q, const std::vector<Eigen::Vector3d> &v, bool updateKin)
{
    // Receuillir les points
    std::vector<s2mNodeBone> tp(projectPoint(marks, Q, v, updateKin));

    // Calculer la jacobienne si le point doit être projeté
    std::vector<s2mMatrix> G;

    for (unsigned int i=0; i<tp.size(); ++i){
        // Marqueur actuel
        s2mNodeBone node = *(tp.begin()+i);
        node.setPosition((*(v.begin()+i)));
        G.push_back(projectPointJacobian(model, Q, node, false));
    }
    return G;
}

RigidBodyDynamics::Math::SpatialTransform s2mJoints::CalcBodyWorldTransformation (
    s2mJoints &model,
    const s2mGenCoord &Q,
    const unsigned int body_id,
    bool update_kinematics = true)
{
    // update the Kinematics if necessary
    if (update_kinematics) {
        RigidBodyDynamics::UpdateKinematicsCustom (model, &Q, nullptr, nullptr);
    }

    if (body_id >= model.fixed_body_discriminator) {
        unsigned int fbody_id = body_id - model.fixed_body_discriminator;
        unsigned int parent_id = model.mFixedBodies[fbody_id].mMovableParent;
        s2mAttitude parentRT(model.X_base[parent_id].E.transpose(), model.X_base[parent_id].r);
        s2mAttitude bodyRT(model.mFixedBodies[fbody_id].mParentTransform.E.transpose(), model.mFixedBodies[fbody_id].mParentTransform.r);
        s2mAttitude transfo_tp = parentRT * bodyRT;
        RigidBodyDynamics::Math::SpatialTransform transfo(transfo_tp.rot(), transfo_tp.trans());
        return transfo;
    }

    RigidBodyDynamics::Math::SpatialTransform transfo(model.X_base[body_id].E.transpose(), model.X_base[body_id].r);
    return transfo;
}

s2mNode s2mJoints::CoM(const s2mGenCoord &Q, bool updateKin){
    // Retour la position du centre de masse a partir des coordonnées généralisées

    // S'assurer que le modele est dans la bonne configuration
    if (updateKin)
        RigidBodyDynamics::UpdateKinematicsCustom(*this,&Q,nullptr,nullptr);

    // Pour chaque segment, trouver le CoM (CoM = somme(masse_segmentaire * pos_com_seg)/masse_totale)
    std::vector<s2mNodeBone> com_segment(CoMbySegment(Q,true));
    s2mNode com;
    for (unsigned int i=0; i<com_segment.size(); ++i)
        com += m_bones[i].caract().mMass * (*(com_segment.begin()+i));

    // Diviser par la masse totale
    com = com/this->mass();

    // Retourner le CoM
    return com;
}

RigidBodyDynamics::Math::Vector3d s2mJoints::angularMomentum(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const bool updateKin){
    return CalcAngularMomentum(*this, Q, Qdot, updateKin);

}


RigidBodyDynamics::Math::Vector3d s2mJoints::CoMdot(const s2mGenCoord &Q, const s2mGenCoord &Qdot){
    // Retour la vitesse du centre de masse a partir des coordonnées généralisées

    // S'assurer que le modele est dans la bonne configuration
    RigidBodyDynamics::UpdateKinematicsCustom(*this,&Q,&Qdot,nullptr);

    // Pour chaque segment, trouver le CoM
    RigidBodyDynamics::Math::Vector3d com_dot = RigidBodyDynamics::Math::Vector3d(0,0,0);

    // CoMdot = somme(masse_seg * Jacobienne * qdot)/masse totale
    for (std::vector<s2mBone>::iterator b_it=m_bones.begin(); b_it!=m_bones.end(); ++b_it){
        s2mMatrix Jac(s2mMatrix::Zero(3,this->dof_count));
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId((*b_it).name().c_str()), (*b_it).caract().mCenterOfMass, Jac, false); // False for speed
        com_dot += (*b_it).caract().mMass*(Jac*Qdot);
    }
    // Diviser par la masse totale
    com_dot = com_dot/this->mass();
	
    // Retourner la vitesse du CoM
    return com_dot;
}
RigidBodyDynamics::Math::Vector3d s2mJoints::CoMddot(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot){
    // Retour l'accélération du centre de masse a partir des coordonnées généralisées
    s2mError::s2mAssert(0, "Com DDot is wrong, to be modified...");

    // S'assurer que le modele est dans la bonne configuration
    RigidBodyDynamics::UpdateKinematicsCustom(*this,&Q,&Qdot,&Qddot);

    // Pour chaque segment, trouver le CoM
    RigidBodyDynamics::Math::Vector3d com_ddot = RigidBodyDynamics::Math::Vector3d(0,0,0);

    // CoMdot = somme(masse_seg * Jacobienne * qdot)/masse totale
    for (std::vector<s2mBone>::iterator b_it=m_bones.begin(); b_it!=m_bones.end(); ++b_it){
        s2mMatrix Jac(s2mMatrix::Zero(3,this->dof_count));
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId((*b_it).name().c_str()), (*b_it).caract().mCenterOfMass, Jac, false); // False for speed
        com_ddot += (*b_it).caract().mMass*(Jac*Qddot);
    }
    // Diviser par la masse totale
    com_ddot = com_ddot/this->mass();

    // Retourner l'accélération du CoM
    return com_ddot;
}

s2mMatrix s2mJoints::CoMJacobian(const s2mGenCoord &Q){
    // Retour la position du centre de masse a partir des coordonnées généralisées

    // S'assurer que le modele est dans la bonne configuration
    RigidBodyDynamics::UpdateKinematicsCustom(*this,&Q,nullptr,nullptr);

   // Jacobienne totale
    s2mMatrix JacTotal(s2mMatrix::Zero(3,this->dof_count));

    // CoMdot = somme(masse_seg * Jacobienne * qdot)/masse totale
    for (std::vector<s2mBone>::iterator b_it=m_bones.begin(); b_it!=m_bones.end(); ++b_it){
        s2mMatrix Jac(s2mMatrix::Zero(3,this->dof_count));
        RigidBodyDynamics::CalcPointJacobian(*this, Q, this->GetBodyId((*b_it).name().c_str()), (*b_it).caract().mCenterOfMass, Jac, false); // False for speed
        JacTotal += (*b_it).caract().mMass*Jac;
    }

    // Diviser par la masse totale
    JacTotal = JacTotal/this->mass();

    // Retourner la jacobienne du CoM
    return JacTotal;
}


std::vector<s2mNodeBone> s2mJoints::CoMbySegment(const s2mGenCoord &Q, bool updateKin){// Position du centre de masse de chaque segment
    std::vector<s2mNodeBone> tp; // vecteur de vecteurs de sortie

    for (unsigned int i=0; i<m_bones.size(); ++i){
        tp.push_back(CoMbySegment(Q,i,updateKin));
        updateKin = false; // ne le faire que la premiere fois
    }
    return tp;
}


s2mNodeBone s2mJoints::CoMbySegment(const s2mGenCoord &Q, const unsigned int i, bool updateKin){ // Position du centre de masse du segment i

    s2mError::s2mAssert(i < m_bones.size(), "Choosen segment doesn't exist");
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(*this, Q, m_bones[i].id(),m_bones[i].caract().mCenterOfMass,updateKin);

}


std::vector<RigidBodyDynamics::Math::Vector3d> s2mJoints::CoMdotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, bool updateKin){// Position du centre de masse de chaque segment
    std::vector<RigidBodyDynamics::Math::Vector3d> tp; // vecteur de vecteurs de sortie

    for (unsigned int i=0; i<m_bones.size(); ++i){
        tp.push_back(CoMdotBySegment(Q,Qdot,i,updateKin));
        updateKin = false; // ne le faire que la premiere fois
    }
    return tp;
}


RigidBodyDynamics::Math::Vector3d s2mJoints::CoMdotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const unsigned int i, bool updateKin){ // Position du centre de masse du segment i

    s2mError::s2mAssert(i < m_bones.size(), "Choosen segment doesn't exist");
    return CalcPointVelocity(*this, Q, Qdot, m_bones[i].id(),m_bones[i].caract().mCenterOfMass,updateKin);

}


std::vector<RigidBodyDynamics::Math::Vector3d> s2mJoints::CoMddotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot, bool updateKin){// Position du centre de masse de chaque segment
    std::vector<RigidBodyDynamics::Math::Vector3d> tp; // vecteur de vecteurs de sortie

    for (unsigned int i=0; i<m_bones.size(); ++i){
        tp.push_back(CoMddotBySegment(Q,Qdot,Qddot,i,updateKin));
        updateKin = false; // ne le faire que la premiere fois
    }
    return tp;
}


RigidBodyDynamics::Math::Vector3d s2mJoints::CoMddotBySegment(const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot, const unsigned int i, bool updateKin){ // Position du centre de masse du segment i

    s2mError::s2mAssert(i < m_bones.size(), "Choosen segment doesn't exist");
    return RigidBodyDynamics::CalcPointAcceleration(*this, Q, Qdot, Qddot, m_bones[i].id(),m_bones[i].caract().mCenterOfMass,updateKin);

}

std::vector<std::vector<s2mNodeBone> > s2mJoints::meshPoints(const s2mGenCoord &Q, const bool updateKin){

    std::vector<std::vector<s2mNodeBone> > v; // Vecteur de vecteur de sortie (mesh par segment)

    // Trouver la position des segments
    std::vector<s2mAttitude> RT(globalJCS(Q, updateKin));

    // Pour tous les segments
    for (unsigned int i=0; i<nbBone(); ++i)
        v.push_back(meshPoints(RT,i));

    return v;
}
std::vector<s2mNodeBone> s2mJoints::meshPoints(const s2mGenCoord &Q, const unsigned int &i, const bool updateKin){

    // Trouver la position des segments
    std::vector<s2mAttitude> RT(globalJCS(Q, updateKin));

    return meshPoints(RT,i);
}
std::vector<s2mNodeBone> s2mJoints::meshPoints(const std::vector<s2mAttitude> &RT, const unsigned int &i){

    // Recueillir la position des meshings
    std::vector<s2mNodeBone> v;
    for (unsigned int j=0; j<boneMesh(i).size(); ++j){
        s2mNode tp (boneMesh(i).point(j));
        tp.applyRT(*(RT.begin()+i));
        v.push_back(tp);
    }

    return v;
}
std::vector<std::vector<s2mPatch> > s2mJoints::meshPatch(){
    // Recueillir la position des meshings pour tous les segments
    std::vector<std::vector<s2mPatch> > v_all;
    for (unsigned int j=0; j<nbBone(); ++j)
        v_all.push_back(meshPatch(j));
    return v_all;
}
std::vector<s2mPatch> s2mJoints::meshPatch(const unsigned int &i){
    // Recueillir la position des meshings pour un segment i
    return boneMesh(i).patch();
}

std::vector<s2mBoneMesh> s2mJoints::boneMesh()
{
    std::vector<s2mBoneMesh> boneOut;
    for (unsigned int i=0; i<nbBone(); ++i)
        boneOut.push_back(boneMesh(i));
    return boneOut;
}

s2mBoneMesh s2mJoints::boneMesh(const unsigned int &idx)
{
    return bone(idx).caract().mesh();
}

RigidBodyDynamics::Math::Vector3d s2mJoints::CalcAngularMomentum (
        s2mJoints &model,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        bool update_kinematics)
{
    // Qddot was added later in the RBDL. In order to keep backward compatilibity, 
    s2mGenCoord Qddot(this->nbQddot());
    return CalcAngularMomentum(model, Q, Qdot, Qddot, update_kinematics);
}
RigidBodyDynamics::Math::Vector3d s2mJoints::CalcAngularMomentum (
        s2mJoints &model,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mGenCoord &Qddot,
        bool update_kinematics)
{
    // Définition des variables
    RigidBodyDynamics::Math::Vector3d com,  angular_momentum;
    double mass;

    // Calcul du angular momentum par la fonction de la position du centre de masse
    RigidBodyDynamics::Utils::CalcCenterOfMass(model, Q, Qdot, &Qddot, mass, com, nullptr, nullptr, &angular_momentum, nullptr, update_kinematics);

    return angular_momentum;
}

std::vector<RigidBodyDynamics::Math::Vector3d> s2mJoints::CalcSegmentsAngularMomentum (s2mJoints &model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, bool update_kinematics) {
    s2mGenCoord Qddot(this->nbQddot());
    return CalcSegmentsAngularMomentum(model, Q, Qdot, Qddot, update_kinematics);
}

std::vector<RigidBodyDynamics::Math::Vector3d> s2mJoints::CalcSegmentsAngularMomentum (s2mJoints &model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Qddot, bool update_kinematics) {
    if (update_kinematics)
        UpdateKinematicsCustom (model, &Q, &Qdot, &Qddot);

    double mass;
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Utils::CalcCenterOfMass (*this, Q, Qdot, &Qddot, mass, com, nullptr, nullptr, nullptr, nullptr, false);
    RigidBodyDynamics::Math::SpatialTransform X_to_COM (RigidBodyDynamics::Math::Xtrans(com));

    std::vector<RigidBodyDynamics::Math::Vector3d> h_segment;
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
        model.Ic[i] = model.I[i];
        model.hc[i] = model.Ic[i].toMatrix() * model.v[i];

        RigidBodyDynamics::Math::SpatialVector h = model.X_lambda[i].applyTranspose (model.hc[i]);
        if (model.lambda[i] != 0) {
            unsigned int j(i);
            do {
                j = model.lambda[j];
                h = model.X_lambda[j].applyTranspose (h);
            } while (model.lambda[j]!=0);
        }
        h = X_to_COM.applyAdjoint (h);
        h_segment.push_back(RigidBodyDynamics::Math::Vector3d(h[0],h[1],h[2]));
    }


    return h_segment;
}

void s2mJoints::ForwardDynamicsContactsLagrangian (
     s2mJoints &model,
     const RigidBodyDynamics::Math::VectorNd &Q,
     const RigidBodyDynamics::Math::VectorNd &QDot,
     const RigidBodyDynamics::Math::VectorNd &Tau,
     RigidBodyDynamics::ConstraintSet &CS,
     RigidBodyDynamics::Math::VectorNd &QDDot
     ) {

   // Compute C
   CS.QDDot_0.setZero();
   InverseDynamics (model, Q, QDot, CS.QDDot_0, CS.C);

   // Compute H
   CS.H = RigidBodyDynamics::Math::MatrixNd::Zero(model.dof_count, model.dof_count);
   CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

   // Compute G
   unsigned int i,j;

   // variables to check whether we need to recompute G
   unsigned int prev_body_id = 0;
   RigidBodyDynamics::Math::Vector3d prev_body_point = RigidBodyDynamics::Math::Vector3d::Zero();
   RigidBodyDynamics::Math::MatrixNd Gi (RigidBodyDynamics::Math::MatrixNd::Zero(3, model.dof_count));

   for (i = 0; i < CS.size(); i++) {
     // Only alow contact normals along the coordinate axes
//     unsigned int axis_index = 0;

     // only compute the matrix Gi if actually needed
     if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
        RigidBodyDynamics::CalcPointJacobian (model, Q, CS.body[i], CS.point[i], Gi, false);
       prev_body_id = CS.body[i];
       prev_body_point = CS.point[i];
     }

     for (j = 0; j < model.dof_count; j++) {
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
   UpdateKinematics (model, Q, QDot, CS.QDDot_0);

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
       s2mError::s2mAssert (0, "Invalid contact normal axis!");

        // only compute point accelerations when necessary
     if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
       gamma_i = CalcPointAcceleration (model, Q, QDot, CS.QDDot_0, CS.body[i], CS.point[i], false);
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
  for (i = 0; i < model.dof_count; i++) {
     for (j = 0; j < model.dof_count; j++) {
       CS.A(i,j) = CS.H(i,j);
     }
  }

   // Build the system: Copy G, and G^T
   for (i = 0; i < CS.size(); i++) {
     for (j = 0; j < model.dof_count; j++) {
       CS.A(i + model.dof_count, j) = CS.G (i,j);
       CS.A(j, i + model.dof_count) = CS.G (i,j);
     }
   }

   // Build the system: Copy -C + \tau
   for (i = 0; i < model.dof_count; i++) {
    CS.b[i] = -CS.C[i] + Tau[i];
   }

   // Build the system: Copy -gamma
   for (i = 0; i < CS.size(); i++) {
     CS.b[i + model.dof_count] = - CS.gamma[i];
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
       s2mError::s2mAssert(0, "Error: Invalid linear solver");
     break;
   }

   //std::cout << "x = " << std::endl << CS.x << std::endl;

   // Copy back QDDot
for (i = 0; i < model.dof_count; i++)
     QDDot[i] = CS.x[i];

  // Copy back contact forces
  for (i = 0; i < CS.size(); i++) {
     CS.force[i] = -CS.x[model.dof_count + i];
   }
 }

unsigned int s2mJoints::nbQuat() const{
    return m_nRotAQuat;
}

void s2mJoints::computeQdot(s2mJoints &, const s2mGenCoord &Q, const s2mGenCoord &QDot, s2mGenCoord &QDotOut){
    // Vérifier s'il y a des quaternions, sinon la dérivée est directement QDot
    if (!m_nRotAQuat){
        QDotOut = QDot;
        return;
    }

    QDotOut.resize(Q.size()); // Créer un vecteur vide de la dimension finale.
    unsigned int cmpQuat(0);
    unsigned int cmpDof(0);
    for (unsigned int i=0; i<nbBone(); ++i){
        s2mBone bone_i=bone(i);
        if (bone_i.isRotationAQuaternion()){
            // Extraire le quaternion
            s2mQuaternion quat_tp(Q.block(cmpDof+bone_i.nDofTrans(),0,3,1), Q(Q.size()-m_nRotAQuat+cmpQuat));

            // Placer dans le vecteur de sortie
            QDotOut.block(cmpDof, 0, bone_i.nDofTrans(),1) = QDot.block(cmpDof, 0, bone_i.nDofTrans(),1); // La dérivée des translations est celle directement de qdot

            // Dériver
            quat_tp.derivate(QDot.block(cmpDof+bone_i.nDofTrans(),0,3,1));
            QDotOut.block(cmpDof+bone_i.nDofTrans(),0,3,1) = quat_tp.block(0,0,3,1);
            QDotOut(Q.size()-m_nRotAQuat+cmpQuat) = quat_tp(3);// Placer dans le vecteur de sortie

           // Incrémenter le nombre de quaternions faits
            ++cmpQuat;
        }
        else{
            // Si c'est un normal, faire ce qu'il est fait d'habitude
            QDotOut.block(cmpDof,0,bone_i.nDof(),1) = QDot.block(cmpDof,0,bone_i.nDof(),1);
        }
        cmpDof += bone_i.nDof();
    }

}



unsigned int s2mJoints::getDofIndex(const s2mString& boneName, const s2mString& dofName){
    unsigned int idx = 0;

    unsigned int iB = 0;
    bool found = false;
    while (1){
        s2mError::s2mAssert(iB!=m_bones.size(), "Bone not found");

        if (boneName.compare(  (*(m_bones.begin()+iB)).name() )   )
            idx +=  (*(m_bones.begin()+iB)).nDof();
        else{
            idx += (*(m_bones.begin()+iB)).getDofIdx(dofName);
            found = true;
            break;
        }

        ++iB;
    }

    s2mError::s2mAssert(found, "Dof not found");
    return idx;
}

void s2mJoints::UpdateKinematicsCustom(s2mJoints &model, const s2mGenCoord *Q, const s2mGenCoord *Qdot, const s2mGenCoord *Qddot)
{
    RigidBodyDynamics::UpdateKinematicsCustom(model, Q, Qdot, Qddot);
}

void s2mJoints::CalcMatRotJacobian(s2mJoints &model, const RigidBodyDynamics::Math::VectorNd &Q, unsigned int body_id, const RigidBodyDynamics::Math::Matrix3d &rotation, RigidBodyDynamics::Math::MatrixNd &G, bool update_kinematics)
{
#ifdef RBDL_ENABLE_LOGGING
    LOG << "-------- " << __func__ << " --------" << std::endl;
#endif

    // update the Kinematics if necessary
    if (update_kinematics) {
        RigidBodyDynamics::UpdateKinematicsCustom (model, &Q, nullptr, nullptr);
    }

    assert (G.rows() == 9 && G.cols() == model.qdot_size );

    std::vector<Eigen::Vector3d> axes;
    axes.push_back(Eigen::Vector3d(1,0,0));
    axes.push_back(Eigen::Vector3d(0,1,0));
    axes.push_back(Eigen::Vector3d(0,0,1));
    for (unsigned int iAxes=0; iAxes<3; ++iAxes){
        RigidBodyDynamics::Math::Matrix3d bodyMatRot (RigidBodyDynamics::CalcBodyWorldOrientation (model, Q, body_id, false).transpose());
        RigidBodyDynamics::Math::SpatialTransform point_trans = RigidBodyDynamics::Math::SpatialTransform (RigidBodyDynamics::Math::Matrix3d::Identity(), bodyMatRot * rotation * *(axes.begin()+iAxes));


        unsigned int reference_body_id = body_id;

        if (model.IsFixedBodyId(body_id)) {
            unsigned int fbody_id = body_id - model.fixed_body_discriminator;
            reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
        }

        unsigned int j = reference_body_id;

        // e[j] is set to 1 if joint j contributes to the jacobian that we are
        // computing. For all other joints the column will be zero.
        while (j != 0) {
            unsigned int q_index = model.mJoints[j].q_index;
            // Si ce n'est pas un dof en translation (3 4 5 dans model.S)
            if (model.S[j](3)!=1.0 && model.S[j](4)!=1.0 && model.S[j](5)!=1.0)
            {
                RigidBodyDynamics::Math::SpatialTransform X_base = model.X_base[j];
                X_base.r << 0,0,0; // Retirer tout concept de translation (ne garder que la matrice rotation)

                if (model.mJoints[j].mDoFCount == 3) {
                    G.block(iAxes*3, q_index, 3, 3) = ((point_trans * X_base.inverse()).toMatrix() * model.multdof3_S[j]).block(3,0,3,3);
                } else {
                    G.block(iAxes*3,q_index, 3, 1) = point_trans.apply(X_base.inverse().apply(model.S[j])).block(3,0,3,1);
                }
            }
            j = model.lambda[j]; // Passer au segment parent
        }
    }
}





unsigned int s2mJoints::nbInterationStep() const
{
    return integrator->steps();
}






