#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleGeometry.h"

s2mMuscleGeometry::s2mMuscleGeometry(const s2mNodeMuscle &o, const s2mNodeMuscle &i) :
    m_origin(o),
    m_insertion(i),
    m_originInGlobal(o),
    m_insertionInGlobal(i),
    m_length(0),
    m_velocity(0),
    m_isGeometryComputed(false),
    m_isVelocityComputed(false),
    m_posAndJacoWereForced(false)
{
    m_originInGlobal.block(0,0,3,1) = Eigen::Vector3d::Zero();
    m_insertionInGlobal.block(0,0,3,1) = Eigen::Vector3d::Zero();
}



/********* FONCTIONS PUBLIQUES *********/
void s2mMuscleGeometry::updateKinematics(s2mJoints &model, const s2mGenCoord *Q, const s2mGenCoord *Qdot, const s2mMuscleCaracteristics &c, const s2mMusclePathChangers& o, int updateKin){
    if (m_posAndJacoWereForced){
        s2mError::s2mWarning(false, "Warning, using updateKinematics overrides the previously sent position and jacobian");
        m_posAndJacoWereForced = false;
    }

    // S'assurer que le modele est dans la bonne configuration
    if (updateKin > 1)
        RigidBodyDynamics::UpdateKinematicsCustom(model, Q, Qdot, nullptr);

    // Position des points dans l'espace
    musclesPointsInGlobal(model, *Q, o);

    // Calcul de la jacobienne des muscles points
    jacobian(model, *Q);

    // Compléter l'update
    _updateKinematics(Qdot, c, &o);
}

void s2mMuscleGeometry::updateKinematics(std::vector<s2mNodeMuscle>& musclePointsInGlobal, s2mMatrix &jacoPointsInGlobal, const s2mGenCoord *Qdot, const s2mMuscleCaracteristics &c){
    m_posAndJacoWereForced = true;

    // Position des points dans l'espace
    musclesPointsInGlobal(musclePointsInGlobal);

    // Calcul de la jacobienne des muscles points
    jacobian(jacoPointsInGlobal);

    // Compléter l'update
    _updateKinematics(Qdot, c);
}

void s2mMuscleGeometry::_updateKinematics(const s2mGenCoord *Qdot, const s2mMuscleCaracteristics &c, const s2mMusclePathChangers * o){
    // Calculer les longueurs et vitesses
    length(c,o);
    m_isGeometryComputed = true;

    // Calcul de la jacobienne des longueurs
    computeJacobianLength();
    if (Qdot != nullptr){
        velocity(*Qdot);
        m_isVelocityComputed = true;
    }
    else
        m_isVelocityComputed = false;
}


// Get and set des position d'origine et insertions
const s2mNodeMuscle& s2mMuscleGeometry::originInLocal() const {
    return m_origin;
}
const s2mNodeMuscle &s2mMuscleGeometry::insertionInLocal() const {
    return m_insertion;
}
void s2mMuscleGeometry::originInLocal(const s2mNodeMuscle &val) {
    m_origin = val;
}
void s2mMuscleGeometry::insertionInLocal(const s2mNodeMuscle &val) {
    m_insertion = val;
}

// Position des muscles dans l'espace
const s2mNodeMuscle &s2mMuscleGeometry::originInGlobal() const {
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed at least once before calling originInLocal()");
    return m_originInGlobal;
}
const s2mNodeMuscle &s2mMuscleGeometry::insertionInGlobal() const {
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed at least once before calling insertionInGlobal()");
    return m_insertionInGlobal;
}
const std::vector<s2mNodeMuscle> &s2mMuscleGeometry::musclesPointsInGlobal() const{
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed at least once before calling musclesPointsInGlobal()");
    return m_pointsInGlobal;
}

// Retour des longueur et vitesse musculaires
double s2mMuscleGeometry::length() const {
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return m_length;
}
double s2mMuscleGeometry::musculoTendonLength() const {
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return m_muscleTendonLength;
}
double s2mMuscleGeometry::velocity() const {
    s2mError::s2mAssert(m_isVelocityComputed, "Geometry must be computed before calling velocity()");
    return m_velocity;
}

// Retour des jacobiennes
const s2mMatrix &s2mMuscleGeometry::jacobian() const {
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed before calling jacobian()");
    return m_jacobian;
} // Retourne la derniere jacobienne
s2mMatrix s2mMuscleGeometry::jacobianOrigin() const{
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed before calling jacobianOrigin()");
    return s2mMatrix(m_jacobian.block(0,0,3,m_jacobian.cols()));
}
s2mMatrix s2mMuscleGeometry::jacobianInsertion() const {
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed before calling jacobianInsertion()");
    return s2mMatrix(m_jacobian.block(m_jacobian.rows()-3,0,3,m_jacobian.cols()));
}
s2mMatrix s2mMuscleGeometry::jacobian(const unsigned int i) const {
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed before calling jacobian(i)");
    return s2mMatrix(m_jacobian.block(3*i,0,3,m_jacobian.cols()));
}

const s2mMatrix &s2mMuscleGeometry::jacobianLength() const{
    s2mError::s2mAssert(m_isGeometryComputed, "Geometry must be computed before calling jacobianLength()");
    return m_jacobianLength;
}

/*********************************************/




const s2mNodeMuscle &s2mMuscleGeometry::originInGlobal(s2mJoints &model, const s2mGenCoord &Q){
    // Sortir la position du marqueur en fonction de la position donnée
    m_originInGlobal.block(0,0,3,1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(m_origin.parent().c_str()), m_origin.position(),false);
    return m_originInGlobal;
}

const s2mNodeMuscle &s2mMuscleGeometry::insertionInGlobal(s2mJoints &model, const s2mGenCoord &Q){
    // Sortir la position du marqueur en fonction de la position donnée
    m_insertionInGlobal.block(0,0,3,1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(m_insertion.parent().c_str()), m_insertion.position(),false);
    return m_insertionInGlobal;
}

double s2mMuscleGeometry::length(const s2mMuscleCaracteristics &c, const s2mMusclePathChangers *objects){
    m_muscleTendonLength = 0;

    // puisqu'on ne peut pas combiner, tester le premier (0) revient a savoir tous les types si plus d'un
    if (objects != nullptr && objects->nbWraps()!=0){
        // CHECK A MODIFIER AVEC L'AVANCEMENT DES PROJETS
        s2mError::s2mAssert(objects->nbVia() == 0, "Cannot mix wrapping and via points yet" ) ;
        s2mError::s2mAssert(objects->nbWraps() < 2, "Cannot compute more than one wrapping yet");

        s2mNodeMuscle pi_wrap; // point sur le wrapping coté insertion
        s2mNodeMuscle po_wrap; // point sur le wrapping coté origine
        double lengthWrap(0);
        std::static_pointer_cast<s2mWrappingObject>(objects->object(0))->wrapPoints(po_wrap, pi_wrap, &lengthWrap);
        m_muscleTendonLength = (*m_pointsInGlobal.begin() - pi_wrap).norm()   + // longueur avant le wrap
                    lengthWrap                 + // longueur sur le wrap
                    (*m_pointsInGlobal.end() - po_wrap).norm();   // longueur apres le wrap

    }
    else{
        std::vector<s2mNodeMuscle>::iterator it = m_pointsInGlobal.begin();
        for (unsigned int i=0; i<m_pointsInGlobal.size()-1; ++i)
            m_muscleTendonLength += (*(it+i) - *(it+i+1)).norm();

    }

    m_length = (m_muscleTendonLength - c.tendonSlackLength())/cos(c.pennationAngle());

    return m_length;
}


void s2mMuscleGeometry::musclesPointsInGlobal(std::vector<s2mNodeMuscle>& ptsInGlobal){
    m_pointsInLocal.clear(); // Dans ce mode, nous n'avons pas besoin de de local, puisque la jacobienne des points DOIT également être donnée
    s2mError::s2mAssert(ptsInGlobal.size() >= 2, "ptsInGlobal must at least have an origin and an insertion");
    m_pointsInGlobal = ptsInGlobal;
}

void s2mMuscleGeometry::musclesPointsInGlobal(s2mJoints &model, const s2mGenCoord &Q, const s2mMusclePathChangers& objects){
    // Variable de sortie (remettre a zero)
    m_pointsInLocal.clear();
    m_pointsInGlobal.clear();

    // Ne pas le faire sur les wrappings objects
    if (objects.nbWraps()!=0){
        // CHECK A MODIFIER AVEC L'AVANCEMENT DES PROJETS
        s2mError::s2mAssert(objects.nbVia() == 0, "Cannot mix wrapping and via points yet") ;
        s2mError::s2mAssert(objects.nbWraps() < 2, "Cannot compute more than one wrapping yet");

        // Récupérer la matrice de RT du wrap
        std::shared_ptr<s2mWrappingObject> w = std::static_pointer_cast<s2mWrappingObject>(objects.object(0));
        s2mAttitude RT = w->RT(model,Q);

        // Alias
        s2mNodeMuscle po_mus = originInGlobal(model, Q);  // Origine sur l'os
        s2mNodeMuscle pi_mus = insertionInGlobal(model,Q); // Insertion sur l'os

        s2mNodeMuscle pi_wrap; // point sur le wrapping coté insertion
        s2mNodeMuscle po_wrap; // point sur le wrapping coté origine

        std::static_pointer_cast<s2mWrappingObject>(objects.object(0))->wrapPoints(RT,po_mus,pi_mus,po_wrap, pi_wrap);

        // Stocker les points dans le local
        s2mError::s2mWarning(0, "Attention le push_back de m_pointsInLocal n'a pas été validé");
        m_pointsInLocal.push_back(originInLocal());
        m_pointsInLocal.push_back(s2mNodeMuscle(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(w->parent().c_str()),po_wrap.position(), false), "wrap_o", w->parent()));
        m_pointsInLocal.push_back(s2mNodeMuscle(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(w->parent().c_str()),pi_wrap.position(), false), "wrap_i", w->parent()));
        m_pointsInLocal.push_back(insertionInLocal());

        // Stocker les points dans le global
        m_pointsInGlobal.push_back(originInGlobal());
        m_pointsInGlobal.push_back(po_wrap);
        m_pointsInGlobal.push_back(pi_wrap);
        m_pointsInGlobal.push_back(insertionInGlobal());

    }

    else if (objects.nbObjects()!=0 && !objects.object(0)->type().tolower().compare("via")){
        m_pointsInLocal.push_back(originInLocal());
        m_pointsInGlobal.push_back(originInGlobal(model, Q));
        for (unsigned int i=0; i<objects.nbObjects(); ++i){
            s2mViaPoint node ( *(std::static_pointer_cast<s2mViaPoint>(objects.object(i))) );
            m_pointsInLocal.push_back(node);
            node.setPosition(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(node.parent().c_str()), node, false));
            m_pointsInGlobal.push_back(node);
        }
        m_pointsInLocal.push_back(insertionInLocal());
        m_pointsInGlobal.push_back(insertionInGlobal(model,Q));

    }
    else if (objects.nbObjects()==0){
        m_pointsInLocal.push_back(originInLocal());
        m_pointsInLocal.push_back(insertionInLocal());
        m_pointsInGlobal.push_back(originInGlobal(model, Q));
        m_pointsInGlobal.push_back(insertionInGlobal(model,Q));
    }
    else
        s2mError::s2mAssert(0, "Length for this type of object was not implemented");

    // Set the dimension of jacobian
    setJacobianDimension(model);
}

double s2mMuscleGeometry::velocity(const s2mGenCoord &Qdot){

    // Calculer la vitesse d'élongation musculaire
    s2mGenCoord velocity(jacobianLength()*Qdot); // Vector 1 element, mais le compilateur ne le sais pas
    m_velocity = velocity[0];

    return m_velocity;
}

void s2mMuscleGeometry::setJacobianDimension(s2mJoints &model)
{
    m_jacobian = s2mMatrix::Zero(static_cast<unsigned int>(m_pointsInLocal.size()*3), model.dof_count);
    m_G = s2mMatrix::Zero(3, model.dof_count);
}

void s2mMuscleGeometry::jacobian(const s2mMatrix &jaco){
    s2mError::s2mAssert(jaco.rows()/3 == static_cast<int>(m_pointsInGlobal.size()), "Jacobian is the wrong size");
    m_jacobian = jaco;
}

void s2mMuscleGeometry::jacobian(s2mJoints &model, const s2mGenCoord &Q){
    for (unsigned int i=0; i<m_pointsInLocal.size(); ++i){
        m_G.setZero();
        RigidBodyDynamics::CalcPointJacobian(model, Q, model.GetBodyId((m_pointsInLocal[i]).parent().c_str()), (m_pointsInLocal[i]).position(), m_G, false); // False for speed
        m_jacobian.block(3*i,0,3,model.dof_count) = m_G;
    }
}

void s2mMuscleGeometry::computeJacobianLength(){
    m_jacobianLength = s2mMatrix::Zero(1, m_jacobian.cols());
    std::vector<s2mNodeMuscle>::iterator p = m_pointsInGlobal.begin();
    for (unsigned int i=0; i<m_pointsInGlobal.size()-1 ; ++i){
        m_jacobianLength +=   (( *(p+i+1) - *(p+i) ).transpose() * (jacobian(i+1) - jacobian(i)))
                                            /
                             ( *(p+i+1) - *(p+i) ).norm();
    }
}
