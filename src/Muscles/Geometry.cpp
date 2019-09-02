#define BIORBD_API_EXPORTS
#include "Muscles/Geometry.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Muscles/WrappingObject.h"
#include "Muscles/PathChangers.h"
#include "Muscles/Caracteristics.h"
#include "Muscles/ViaPoint.h"

biorbd::muscles::Geometry::Geometry() :
    m_origin(std::make_shared<biorbd::utils::Node3d>()),
    m_insertion(std::make_shared<biorbd::utils::Node3d>()),
    m_originInGlobal(std::make_shared<biorbd::utils::Node3d>(biorbd::utils::Node3d::Zero())),
    m_insertionInGlobal(std::make_shared<biorbd::utils::Node3d>(biorbd::utils::Node3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<biorbd::utils::Node3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<biorbd::utils::Node3d>>()),
    m_jacobian(std::make_shared<biorbd::utils::Matrix>()),
    m_G(std::make_shared<biorbd::utils::Matrix>()),
    m_jacobianLength(std::make_shared<biorbd::utils::Matrix>()),
    m_length(std::make_shared<double>(0)),
    m_muscleTendonLength(std::make_shared<double>(0)),
    m_velocity(std::make_shared<double>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

biorbd::muscles::Geometry::Geometry(
        const biorbd::utils::Node3d &origin,
        const biorbd::utils::Node3d &insertion) :
    m_origin(std::make_shared<biorbd::utils::Node3d>(origin)),
    m_insertion(std::make_shared<biorbd::utils::Node3d>(insertion)),
    m_originInGlobal(std::make_shared<biorbd::utils::Node3d>(biorbd::utils::Node3d::Zero())),
    m_insertionInGlobal(std::make_shared<biorbd::utils::Node3d>(biorbd::utils::Node3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<biorbd::utils::Node3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<biorbd::utils::Node3d>>()),
    m_jacobian(std::make_shared<biorbd::utils::Matrix>()),
    m_G(std::make_shared<biorbd::utils::Matrix>()),
    m_jacobianLength(std::make_shared<biorbd::utils::Matrix>()),
    m_length(std::make_shared<double>(0)),
    m_muscleTendonLength(std::make_shared<double>(0)),
    m_velocity(std::make_shared<double>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

biorbd::muscles::Geometry biorbd::muscles::Geometry::DeepCopy() const
{
    biorbd::muscles::Geometry copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::Geometry::DeepCopy(const biorbd::muscles::Geometry &other)
{
    *m_origin = other.m_origin->DeepCopy();
    *m_insertion = other.m_insertion->DeepCopy();
    *m_originInGlobal = other.m_originInGlobal->DeepCopy();
    *m_insertionInGlobal = other.m_insertionInGlobal->DeepCopy();
    m_pointsInGlobal->resize(other.m_pointsInGlobal->size());
    for (unsigned int i=0; i<other.m_pointsInGlobal->size(); ++i)
        (*m_pointsInGlobal)[i] = (*other.m_pointsInGlobal)[i].DeepCopy();
    m_pointsInLocal->resize(other.m_pointsInLocal->size());
    for (unsigned int i=0; i<other.m_pointsInLocal->size(); ++i)
        (*m_pointsInLocal)[i] = (*other.m_pointsInLocal)[i].DeepCopy();
    *m_jacobian = other.m_jacobian->DeepCopy();
    *m_G = other.m_G->DeepCopy();
    *m_jacobianLength = other.m_jacobianLength->DeepCopy();
    *m_length = *other.m_length;
    *m_muscleTendonLength = *other.m_muscleTendonLength;
    *m_velocity = *other.m_velocity;
    *m_isGeometryComputed = *other.m_isGeometryComputed;
    *m_isVelocityComputed = *other.m_isVelocityComputed;
    *m_posAndJacoWereForced = *other.m_posAndJacoWereForced;
}


// ------ FONCTIONS PUBLIQUES ------ //
void biorbd::muscles::Geometry::updateKinematics(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates *Q,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        int updateKin)
{
    if (*m_posAndJacoWereForced){
        biorbd::utils::Error::warning(false, "Warning, using updateKinematics overrides the previously sent position and jacobian");
        *m_posAndJacoWereForced = false;
    }

    // S'assurer que le modele est dans la bonne configuration
    if (updateKin > 1)
        model.UpdateKinematicsCustom(Q, Qdot, nullptr);

    // Position des points dans l'espace
    musclesPointsInGlobal(model, *Q);

    // Calcul de la jacobienne des muscles points
    jacobian(model, *Q);

    // Compléter l'update
    _updateKinematics(Qdot);
}

void biorbd::muscles::Geometry::updateKinematics(
        biorbd::rigidbody::Joints &model,
        const biorbd::muscles::Caracteristics& c,
        biorbd::muscles::PathChangers& o,
        const biorbd::rigidbody::GeneralizedCoordinates *Q,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        int updateKin)
{
    if (*m_posAndJacoWereForced){
        biorbd::utils::Error::warning(false, "Warning, using updateKinematics overrides the previously sent position and jacobian");
        *m_posAndJacoWereForced = false;
    }

    // S'assurer que le modele est dans la bonne configuration
    if (updateKin > 1)
        model.UpdateKinematicsCustom(Q, Qdot, nullptr);

    // Position des points dans l'espace
    musclesPointsInGlobal(model, *Q, &o);

    // Calcul de la jacobienne des muscles points
    jacobian(model, *Q);

    // Compléter l'update
    _updateKinematics(Qdot, &c, &o);
}

void biorbd::muscles::Geometry::updateKinematics(
        std::vector<utils::Node3d> &musclePointsInGlobal,
        biorbd::utils::Matrix &jacoPointsInGlobal,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot)
{
    *m_posAndJacoWereForced = true;

    // Position des points dans l'espace
    musclesPointsInGlobal(musclePointsInGlobal);

    // Calcul de la jacobienne des muscles points
    jacobian(jacoPointsInGlobal);

    // Compléter l'update
    _updateKinematics(Qdot);
}

void biorbd::muscles::Geometry::updateKinematics(
        std::vector<utils::Node3d> &musclePointsInGlobal,
        biorbd::utils::Matrix &jacoPointsInGlobal,
        const biorbd::muscles::Caracteristics &c,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot)
{
    *m_posAndJacoWereForced = true;

    // Position des points dans l'espace
    musclesPointsInGlobal(musclePointsInGlobal);

    // Calcul de la jacobienne des muscles points
    jacobian(jacoPointsInGlobal);

    // Compléter l'update
    _updateKinematics(Qdot, &c);
}

// Get and set des position d'origine et insertions
const biorbd::utils::Node3d& biorbd::muscles::Geometry::originInLocal() const
{
    return *m_origin;
}
void biorbd::muscles::Geometry::setOriginInLocal(const utils::Node3d &val)
{
    *m_origin = val;
}
const biorbd::utils::Node3d &biorbd::muscles::Geometry::insertionInLocal() const
{
    return *m_insertion;
}
void biorbd::muscles::Geometry::setInsertionInLocal(const utils::Node3d &val)
{
    *m_insertion = val;
}

// Position des muscles dans l'espace
const biorbd::utils::Node3d &biorbd::muscles::Geometry::originInGlobal() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed at least once before calling originInLocal()");
    return *m_originInGlobal;
}
const biorbd::utils::Node3d &biorbd::muscles::Geometry::insertionInGlobal() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed at least once before calling insertionInGlobal()");
    return *m_insertionInGlobal;
}
const std::vector<biorbd::utils::Node3d> &biorbd::muscles::Geometry::musclesPointsInGlobal() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed at least once before calling musclesPointsInGlobal()");
    return *m_pointsInGlobal;
}

// Retour des longueur et vitesse musculaires
double biorbd::muscles::Geometry::length() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return *m_length;
}
double biorbd::muscles::Geometry::musculoTendonLength() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return *m_muscleTendonLength;
}
double biorbd::muscles::Geometry::velocity() const
{
    biorbd::utils::Error::error(*m_isVelocityComputed, "Geometry must be computed before calling velocity()");
    return *m_velocity;
}

// Retour des jacobiennes
const biorbd::utils::Matrix& biorbd::muscles::Geometry::jacobian() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed before calling jacobian()");
    return *m_jacobian;
} // Retourne la derniere jacobienne
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobianOrigin() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed before calling jacobianOrigin()");
    return m_jacobian->block(0,0,3,m_jacobian->cols());
}
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobianInsertion() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed before calling jacobianInsertion()");
    return m_jacobian->block(m_jacobian->rows()-3,0,3,m_jacobian->cols());
}
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobian(unsigned int idxMarker) const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed before calling jacobian(i)");
    return m_jacobian->block(3*idxMarker,0,3,m_jacobian->cols());
}

const biorbd::utils::Matrix &biorbd::muscles::Geometry::jacobianLength() const
{
    biorbd::utils::Error::error(*m_isGeometryComputed, "Geometry must be computed before calling jacobianLength()");
    return *m_jacobianLength;
}

// --------------------------------------- //

void biorbd::muscles::Geometry::_updateKinematics(
        const biorbd::rigidbody::GeneralizedCoordinates* Qdot,
        const biorbd::muscles::Caracteristics* c,
        biorbd::muscles::PathChangers* o)
{
    // Calculer les longueurs et vitesses
    length(c,o);
    *m_isGeometryComputed = true;

    // Calcul de la jacobienne des longueurs
    computeJacobianLength();
    if (Qdot != nullptr){
        velocity(*Qdot);
        *m_isVelocityComputed = true;
    }
    else
        *m_isVelocityComputed = false;
}

const biorbd::utils::Node3d &biorbd::muscles::Geometry::originInGlobal(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    // Sortir la position du marqueur en fonction de la position donnée
    m_originInGlobal->block(0,0,3,1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(m_origin->parent().c_str()), *m_origin,false);
    return *m_originInGlobal;
}

const biorbd::utils::Node3d &biorbd::muscles::Geometry::insertionInGlobal(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    // Sortir la position du marqueur en fonction de la position donnée
    m_insertionInGlobal->block(0,0,3,1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(m_insertion->parent().c_str()), *m_insertion,false);
    return *m_insertionInGlobal;
}

void biorbd::muscles::Geometry::musclesPointsInGlobal(std::vector<utils::Node3d> &ptsInGlobal)
{
    biorbd::utils::Error::error(ptsInGlobal.size() >= 2, "ptsInGlobal must at least have an origin and an insertion");
    m_pointsInLocal->clear(); // Dans ce mode, nous n'avons pas besoin de de local, puisque la jacobienne des points DOIT également être donnée
    *m_pointsInGlobal = ptsInGlobal;
}

void biorbd::muscles::Geometry::musclesPointsInGlobal(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        biorbd::muscles::PathChangers* objects)
{
    // Variable de sortie (remettre a zero)
    m_pointsInLocal->clear();
    m_pointsInGlobal->clear();

    // Ne pas le faire sur les wrappings objects
    if (objects->nbWraps()!=0){
        // CHECK A MODIFIER AVEC L'AVANCEMENT DES PROJETS
        biorbd::utils::Error::error(objects->nbVia() == 0, "Cannot mix wrapping and via points yet") ;
        biorbd::utils::Error::error(objects->nbWraps() < 2, "Cannot compute more than one wrapping yet");

        // Récupérer la matrice de RT du wrap
        biorbd::muscles::WrappingObject& w = static_cast<biorbd::muscles::WrappingObject&>(objects->object(0));
        const biorbd::utils::RotoTrans& RT = w.RT(model,Q);

        // Alias
        const biorbd::utils::Node3d& po_mus = originInGlobal(model, Q);  // Origine sur l'os
        const biorbd::utils::Node3d& pi_mus = insertionInGlobal(model,Q); // Insertion sur l'os

        biorbd::utils::Node3d pi_wrap(0, 0, 0); // point sur le wrapping coté insertion
        biorbd::utils::Node3d po_wrap(0, 0, 0); // point sur le wrapping coté origine

        w.wrapPoints(RT,po_mus,pi_mus,po_wrap, pi_wrap);

        // Stocker les points dans le local
        biorbd::utils::Error::warning(0, "Attention le push_back de m_pointsInLocal n'a pas été validé");
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(
                    biorbd::utils::Node3d(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                                              model, Q, model.GetBodyId(w.parent().c_str()),po_wrap, false),
                                          "wrap_o", w.parent()));
        m_pointsInLocal->push_back(
                    biorbd::utils::Node3d(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                                              model, Q, model.GetBodyId(w.parent().c_str()),pi_wrap, false),
                                          "wrap_i", w.parent()));
        m_pointsInLocal->push_back(insertionInLocal());

        // Stocker les points dans le global
        m_pointsInGlobal->push_back(originInGlobal());
        m_pointsInGlobal->push_back(po_wrap);
        m_pointsInGlobal->push_back(pi_wrap);
        m_pointsInGlobal->push_back(insertionInGlobal());

    }

    else if (objects->nbObjects()!=0 && objects->object(0).typeOfNode() == biorbd::utils::NODE_TYPE::VIA_POINT){
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        for (unsigned int i=0; i<objects->nbObjects(); ++i){
            const biorbd::muscles::ViaPoint& node(static_cast<biorbd::muscles::ViaPoint&>(objects->object(i)));
            m_pointsInLocal->push_back(node);
            m_pointsInGlobal->push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, model.GetBodyId(node.parent().c_str()), node, false));
        }
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));

    }
    else if (objects->nbObjects()==0){
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));
    }
    else
        biorbd::utils::Error::error(0, "Length for this type of object was not implemented");

    // Set the dimension of jacobian
    setJacobianDimension(model);
}

double biorbd::muscles::Geometry::length(
        const biorbd::muscles::Caracteristics *caract,
        biorbd::muscles::PathChangers *objects)
{
    *m_muscleTendonLength = 0;

    // puisqu'on ne peut pas combiner, tester le premier (0) revient a savoir tous les types si plus d'un
    if (objects != nullptr && objects->nbWraps()!=0){
        // CHECK A MODIFIER AVEC L'AVANCEMENT DES PROJETS
        biorbd::utils::Error::error(objects->nbVia() == 0, "Cannot mix wrapping and via points yet" ) ;
        biorbd::utils::Error::error(objects->nbWraps() < 2, "Cannot compute more than one wrapping yet");

        biorbd::utils::Node3d pi_wrap(0, 0, 0); // point sur le wrapping coté insertion
        biorbd::utils::Node3d po_wrap(0, 0, 0); // point sur le wrapping coté origine
        double lengthWrap(0);
        static_cast<biorbd::muscles::WrappingObject&>(objects->object(0)).wrapPoints(po_wrap, pi_wrap, &lengthWrap);
        *m_muscleTendonLength = ((*m_pointsInGlobal)[0] - pi_wrap).norm()   + // longueur avant le wrap
                    lengthWrap                 + // longueur sur le wrap
                    (*m_pointsInGlobal->end() - po_wrap).norm();   // longueur apres le wrap

    }
    else{
        for (unsigned int i=0; i<m_pointsInGlobal->size()-1; ++i)
            *m_muscleTendonLength += ((*m_pointsInGlobal)[i+1] - (*m_pointsInGlobal)[i]).norm();
    }

    *m_length = (*m_muscleTendonLength - caract->tendonSlackLength())/cos(caract->pennationAngle());

    return *m_length;
}

double biorbd::muscles::Geometry::velocity(const biorbd::rigidbody::GeneralizedCoordinates &Qdot)
{
    // Calculer la vitesse d'élongation musculaire
    *m_velocity = (jacobianLength()*Qdot)[0]; // This a double but the compiler doesn't know it
    return *m_velocity;
}

void biorbd::muscles::Geometry::setJacobianDimension(biorbd::rigidbody::Joints &model)
{
    *m_jacobian = biorbd::utils::Matrix::Zero(static_cast<unsigned int>(m_pointsInLocal->size()*3), model.dof_count);
    *m_G = biorbd::utils::Matrix::Zero(3, model.dof_count);
}

void biorbd::muscles::Geometry::jacobian(const biorbd::utils::Matrix &jaco)
{
    biorbd::utils::Error::error(jaco.rows()/3 == static_cast<int>(m_pointsInGlobal->size()), "Jacobian is the wrong size");
    *m_jacobian = jaco;
}

void biorbd::muscles::Geometry::jacobian(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    for (unsigned int i=0; i<m_pointsInLocal->size(); ++i){
        m_G->setZero();
        RigidBodyDynamics::CalcPointJacobian(model, Q, model.GetBodyId((*m_pointsInLocal)[i].parent().c_str()), (*m_pointsInLocal)[i], *m_G, false); // False for speed
        m_jacobian->block(3*i,0,3,model.dof_count) = *m_G;
    }
}

void biorbd::muscles::Geometry::computeJacobianLength()
{
    *m_jacobianLength = biorbd::utils::Matrix::Zero(1, m_jacobian->cols());
    const std::vector<biorbd::utils::Node3d>& p = *m_pointsInGlobal;
    for (unsigned int i=0; i<p.size()-1 ; ++i){
        *m_jacobianLength += (( p[i+1] - p[i] ).transpose() * (jacobian(i+1) - jacobian(i)))
                                            /
                             ( p[i+1] - p[i] ).norm();
    }
}
