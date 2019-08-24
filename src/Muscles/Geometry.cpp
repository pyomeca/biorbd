#define BIORBD_API_EXPORTS
#include "Muscles/Geometry.h"

#include <rbdl/Kinematics.h>
#include "Utils/RotoTrans.h"
#include "Muscles/WrappingObject.h"
#include "Muscles/ViaPoint.h"
#include "Utils/Attitude.h"

biorbd::muscles::Geometry::Geometry(
        const biorbd::muscles::MuscleNode &o,
        const biorbd::muscles::MuscleNode &i) :
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
    m_originInGlobal.block(0,0,3,1) = biorbd::muscles::MuscleNode::Zero();
    m_insertionInGlobal.block(0,0,3,1) = biorbd::muscles::MuscleNode::Zero();
}

biorbd::muscles::Geometry::~Geometry()
{

}



// ------ FONCTIONS PUBLIQUES ------ //
void biorbd::muscles::Geometry::updateKinematics(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates *Q,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        const biorbd::muscles::Caracteristics &c,
        const biorbd::muscles::PathChangers& o,
        int updateKin){
    if (m_posAndJacoWereForced){
        biorbd::utils::Error::warning(false, "Warning, using updateKinematics overrides the previously sent position and jacobian");
        m_posAndJacoWereForced = false;
    }

    // S'assurer que le modele est dans la bonne configuration
    if (updateKin > 1)
        model.UpdateKinematicsCustom(Q, Qdot, nullptr);

    // Position des points dans l'espace
    musclesPointsInGlobal(model, *Q, o);

    // Calcul de la jacobienne des muscles points
    jacobian(model, *Q);

    // Compléter l'update
    _updateKinematics(Qdot, c, &o);
}

void biorbd::muscles::Geometry::updateKinematics(
        std::vector<biorbd::muscles::MuscleNode>& musclePointsInGlobal,
        biorbd::utils::Matrix &jacoPointsInGlobal,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        const biorbd::muscles::Caracteristics &c){
    m_posAndJacoWereForced = true;

    // Position des points dans l'espace
    musclesPointsInGlobal(musclePointsInGlobal);

    // Calcul de la jacobienne des muscles points
    jacobian(jacoPointsInGlobal);

    // Compléter l'update
    _updateKinematics(Qdot, c);
}

void biorbd::muscles::Geometry::_updateKinematics(
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        const biorbd::muscles::Caracteristics &c,
        const biorbd::muscles::PathChangers * o){
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
const biorbd::muscles::MuscleNode& biorbd::muscles::Geometry::originInLocal() const {
    return m_origin;
}
const biorbd::muscles::MuscleNode &biorbd::muscles::Geometry::insertionInLocal() const {
    return m_insertion;
}
void biorbd::muscles::Geometry::originInLocal(const biorbd::muscles::MuscleNode &val) {
    m_origin = val;
}
void biorbd::muscles::Geometry::insertionInLocal(const biorbd::muscles::MuscleNode &val) {
    m_insertion = val;
}

// Position des muscles dans l'espace
const biorbd::muscles::MuscleNode &biorbd::muscles::Geometry::originInGlobal() const {
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed at least once before calling originInLocal()");
    return m_originInGlobal;
}
const biorbd::muscles::MuscleNode &biorbd::muscles::Geometry::insertionInGlobal() const {
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed at least once before calling insertionInGlobal()");
    return m_insertionInGlobal;
}
const std::vector<biorbd::muscles::MuscleNode> &biorbd::muscles::Geometry::musclesPointsInGlobal() const{
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed at least once before calling musclesPointsInGlobal()");
    return m_pointsInGlobal;
}

// Retour des longueur et vitesse musculaires
double biorbd::muscles::Geometry::length() const {
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return m_length;
}
double biorbd::muscles::Geometry::musculoTendonLength() const {
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return m_muscleTendonLength;
}
double biorbd::muscles::Geometry::velocity() const {
    biorbd::utils::Error::error(m_isVelocityComputed, "Geometry must be computed before calling velocity()");
    return m_velocity;
}

// Retour des jacobiennes
const biorbd::utils::Matrix &biorbd::muscles::Geometry::jacobian() const {
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed before calling jacobian()");
    return m_jacobian;
} // Retourne la derniere jacobienne
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobianOrigin() const{
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed before calling jacobianOrigin()");
    return biorbd::utils::Matrix(m_jacobian.block(0,0,3,m_jacobian.cols()));
}
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobianInsertion() const {
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed before calling jacobianInsertion()");
    return biorbd::utils::Matrix(m_jacobian.block(m_jacobian.rows()-3,0,3,m_jacobian.cols()));
}
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobian(const unsigned int i) const {
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed before calling jacobian(i)");
    return biorbd::utils::Matrix(m_jacobian.block(3*i,0,3,m_jacobian.cols()));
}

const biorbd::utils::Matrix &biorbd::muscles::Geometry::jacobianLength() const{
    biorbd::utils::Error::error(m_isGeometryComputed, "Geometry must be computed before calling jacobianLength()");
    return m_jacobianLength;
}

// --------------------------------------- //




const biorbd::muscles::MuscleNode &biorbd::muscles::Geometry::originInGlobal(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q){
    // Sortir la position du marqueur en fonction de la position donnée
    m_originInGlobal.block(0,0,3,1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(m_origin.parent().c_str()), m_origin.position(),false);
    return m_originInGlobal;
}

const biorbd::muscles::MuscleNode &biorbd::muscles::Geometry::insertionInGlobal(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q){
    // Sortir la position du marqueur en fonction de la position donnée
    m_insertionInGlobal.block(0,0,3,1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(m_insertion.parent().c_str()), m_insertion.position(),false);
    return m_insertionInGlobal;
}

double biorbd::muscles::Geometry::length(
        const biorbd::muscles::Caracteristics &c,
        const biorbd::muscles::PathChangers *objects){
    m_muscleTendonLength = 0;

    // puisqu'on ne peut pas combiner, tester le premier (0) revient a savoir tous les types si plus d'un
    if (objects != nullptr && objects->nbWraps()!=0){
        // CHECK A MODIFIER AVEC L'AVANCEMENT DES PROJETS
        biorbd::utils::Error::error(objects->nbVia() == 0, "Cannot mix wrapping and via points yet" ) ;
        biorbd::utils::Error::error(objects->nbWraps() < 2, "Cannot compute more than one wrapping yet");

        biorbd::muscles::MuscleNode pi_wrap(0, 0, 0); // point sur le wrapping coté insertion
        biorbd::muscles::MuscleNode po_wrap(0, 0, 0); // point sur le wrapping coté origine
        double lengthWrap(0);
        std::static_pointer_cast<biorbd::muscles::WrappingObject>(objects->object(0))->wrapPoints(po_wrap, pi_wrap, &lengthWrap);
        m_muscleTendonLength = (*m_pointsInGlobal.begin() - pi_wrap).norm()   + // longueur avant le wrap
                    lengthWrap                 + // longueur sur le wrap
                    (*m_pointsInGlobal.end() - po_wrap).norm();   // longueur apres le wrap

    }
    else{
        std::vector<biorbd::muscles::MuscleNode>::iterator it = m_pointsInGlobal.begin();
        for (unsigned int i=0; i<m_pointsInGlobal.size()-1; ++i)
            m_muscleTendonLength += (*(it+i) - *(it+i+1)).norm();

    }

    m_length = (m_muscleTendonLength - c.tendonSlackLength())/cos(c.pennationAngle());

    return m_length;
}


void biorbd::muscles::Geometry::musclesPointsInGlobal(std::vector<biorbd::muscles::MuscleNode>& ptsInGlobal){
    m_pointsInLocal.clear(); // Dans ce mode, nous n'avons pas besoin de de local, puisque la jacobienne des points DOIT également être donnée
    biorbd::utils::Error::error(ptsInGlobal.size() >= 2, "ptsInGlobal must at least have an origin and an insertion");
    m_pointsInGlobal = ptsInGlobal;
}

void biorbd::muscles::Geometry::musclesPointsInGlobal(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::muscles::PathChangers& objects){
    // Variable de sortie (remettre a zero)
    m_pointsInLocal.clear();
    m_pointsInGlobal.clear();

    // Ne pas le faire sur les wrappings objects
    if (objects.nbWraps()!=0){
        // CHECK A MODIFIER AVEC L'AVANCEMENT DES PROJETS
        biorbd::utils::Error::error(objects.nbVia() == 0, "Cannot mix wrapping and via points yet") ;
        biorbd::utils::Error::error(objects.nbWraps() < 2, "Cannot compute more than one wrapping yet");

        // Récupérer la matrice de RT du wrap
        std::shared_ptr<biorbd::muscles::WrappingObject> w = std::static_pointer_cast<biorbd::muscles::WrappingObject>(objects.object(0));
        const biorbd::utils::RotoTrans& RT = w.RT(model,Q);

        // Alias
        biorbd::muscles::MuscleNode po_mus = originInGlobal(model, Q);  // Origine sur l'os
        biorbd::muscles::MuscleNode pi_mus = insertionInGlobal(model,Q); // Insertion sur l'os

        biorbd::muscles::MuscleNode pi_wrap(0, 0, 0); // point sur le wrapping coté insertion
        biorbd::muscles::MuscleNode po_wrap(0, 0, 0); // point sur le wrapping coté origine

        std::static_pointer_cast<biorbd::muscles::WrappingObject>(objects.object(0))->wrapPoints(RT,po_mus,pi_mus,po_wrap, pi_wrap);

        // Stocker les points dans le local
        biorbd::utils::Error::warning(0, "Attention le push_back de m_pointsInLocal n'a pas été validé");
        m_pointsInLocal.push_back(originInLocal());
        m_pointsInLocal.push_back(biorbd::muscles::MuscleNode(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(w->parent().c_str()),po_wrap.position(), false), "wrap_o", w->parent()));
        m_pointsInLocal.push_back(biorbd::muscles::MuscleNode(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(w->parent().c_str()),pi_wrap.position(), false), "wrap_i", w->parent()));
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
            biorbd::muscles::ViaPoint node ( *(std::static_pointer_cast<biorbd::muscles::ViaPoint>(objects.object(i))) );
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
        biorbd::utils::Error::error(0, "Length for this type of object was not implemented");

    // Set the dimension of jacobian
    setJacobianDimension(model);
}

double biorbd::muscles::Geometry::velocity(const biorbd::rigidbody::GeneralizedCoordinates &Qdot){

    // Calculer la vitesse d'élongation musculaire
    // TODO remove the construction of GeneralizedCoordinates and see for the calculation itself
    biorbd::rigidbody::GeneralizedCoordinates velocity(jacobianLength()*Qdot); // Vector 1 element, mais le compilateur ne le sais pas
    m_velocity = velocity[0];

    return m_velocity;
}

void biorbd::muscles::Geometry::setJacobianDimension(biorbd::rigidbody::Joints &model)
{
    m_jacobian = biorbd::utils::Matrix::Zero(static_cast<unsigned int>(m_pointsInLocal.size()*3), model.dof_count);
    m_G = biorbd::utils::Matrix::Zero(3, model.dof_count);
}

void biorbd::muscles::Geometry::jacobian(const biorbd::utils::Matrix &jaco){
    biorbd::utils::Error::error(jaco.rows()/3 == static_cast<int>(m_pointsInGlobal.size()), "Jacobian is the wrong size");
    m_jacobian = jaco;
}

void biorbd::muscles::Geometry::jacobian(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q){
    for (unsigned int i=0; i<m_pointsInLocal.size(); ++i){
        m_G.setZero();
        RigidBodyDynamics::CalcPointJacobian(model, Q, model.GetBodyId((m_pointsInLocal[i]).parent().c_str()), (m_pointsInLocal[i]).position(), m_G, false); // False for speed
        m_jacobian.block(3*i,0,3,model.dof_count) = m_G;
    }
}

void biorbd::muscles::Geometry::computeJacobianLength(){
    m_jacobianLength = biorbd::utils::Matrix::Zero(1, m_jacobian.cols());
    const std::vector<biorbd::muscles::MuscleNode>& p = m_pointsInGlobal;
    for (unsigned int i=0; i<m_pointsInGlobal.size()-1 ; ++i){
        m_jacobianLength +=   (( p[i+1] - p[i] ).transpose() * (jacobian(i+1) - jacobian(i)))
                                            /
                             ( p[i+1] - p[i] ).norm();
    }
}
