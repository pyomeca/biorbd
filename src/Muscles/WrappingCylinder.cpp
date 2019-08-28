#define BIORBD_API_EXPORTS
#include "Muscles/WrappingCylinder.h"

#include "Utils/String.h"
#include "RigidBody/Joints.h"

biorbd::muscles::WrappingCylinder::WrappingCylinder() :
    biorbd::muscles::WrappingObject (),
    m_dia(std::make_shared<double>(0)),
    m_length(std::make_shared<double>(0)),
    m_isCylinderPositiveSign(std::make_shared<bool>(true)),
    m_RTtoParent(std::make_shared<biorbd::utils::RotoTrans>()),
    m_p1Wrap(std::make_shared<biorbd::utils::Node3d>()),
    m_p2Wrap(std::make_shared<biorbd::utils::Node3d>()),
    m_lengthAroundWrap(std::make_shared<double>(0))
{
    *m_typeOfNode = "WrappingCylinder";
}

biorbd::muscles::WrappingCylinder::WrappingCylinder(
        const biorbd::utils::RotoTrans &rt,
        double diameter,
        double length,
        bool isCylinderPositiveSign) :
    biorbd::muscles::WrappingObject (rt.trans()),
    m_dia(std::make_shared<double>(diameter)),
    m_length(std::make_shared<double>(length)),
    m_isCylinderPositiveSign(std::make_shared<bool>(isCylinderPositiveSign)),
    m_RTtoParent(std::make_shared<biorbd::utils::RotoTrans>(rt)),
    m_p1Wrap(std::make_shared<biorbd::utils::Node3d>()),
    m_p2Wrap(std::make_shared<biorbd::utils::Node3d>()),
    m_lengthAroundWrap(std::make_shared<double>(0))
{
    *m_typeOfNode = "WrappingCylinder";
}

biorbd::muscles::WrappingCylinder::WrappingCylinder(
        const biorbd::utils::RotoTrans &rt,
        double diameter,
        double length,
        bool isCylinderPositiveSign,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::muscles::WrappingObject (rt.trans(), name, parentName),
    m_dia(std::make_shared<double>(diameter)),
    m_length(std::make_shared<double>(length)),
    m_isCylinderPositiveSign(std::make_shared<bool>(isCylinderPositiveSign)),
    m_RTtoParent(std::make_shared<biorbd::utils::RotoTrans>(rt)),
    m_p1Wrap(std::make_shared<biorbd::utils::Node3d>()),
    m_p2Wrap(std::make_shared<biorbd::utils::Node3d>()),
    m_lengthAroundWrap(std::make_shared<double>(0))
{
    *m_typeOfNode = "WrappingCylinder";
}

void biorbd::muscles::WrappingCylinder::wrapPoints(
        const biorbd::utils::RotoTrans& rt,
        const biorbd::utils::Node3d& p1_bone,
        const biorbd::utils::Node3d& p2_bone,
        biorbd::utils::Node3d& p1,
        biorbd::utils::Node3d& p2,
        double *length)
{
    // Cette fonction une position du wrapping et trouve l'endroit ou le muscle 1 et 2 quittent le wrapping object

    // Trouver les nodes dans le repere RT (du cylindre)
    NodeMusclePair p_glob(p1_bone, p2_bone);
    p_glob.m_p1->applyRT(rt.transpose());
    p_glob.m_p2->applyRT(rt.transpose());

    // Trouver les tangeantes de ces points au cercle (cylindre vu de dessus)
    biorbd::utils::Node3d p1_tan(0, 0, 0);
    biorbd::utils::Node3d p2_tan(0, 0, 0);
    findTangentToCircle(*p_glob.m_p1, p1_tan);
    findTangentToCircle(*p_glob.m_p2, p2_tan);

    // Trouver la composante verticale
    NodeMusclePair tanPoints(p1_tan, p2_tan);
    findVerticalNode(p_glob, tanPoints);

    // Si demandé, calculé la distance parcourue sur le pourtours du cylindre
    // Le calcul consiste a appliquer pythagore sur l'arc de cercle
    if (length != nullptr) // Si ce n'est pas nullptr
        *length = computeLength(tanPoints);

    // Remettre les points dans le global
    tanPoints.m_p1->applyRT(rt);
    tanPoints.m_p2->applyRT(rt);

    // Retourner les valeurs souhaitées
    p1 = *tanPoints.m_p1;
    p2 = *tanPoints.m_p2;

    // Stocker les valeurs pour un rappel futur
    m_p1Wrap = tanPoints.m_p1;
    m_p2Wrap = tanPoints.m_p2;
    if (length != nullptr) // Si ce n'est pas nullptr
        *m_lengthAroundWrap = *length;
}

void biorbd::muscles::WrappingCylinder::wrapPoints(
        biorbd::rigidbody::Joints& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::utils::Node3d& p1_bone,
        const biorbd::utils::Node3d& p2_bone,
        biorbd::utils::Node3d& p1,
        biorbd::utils::Node3d& p2,
        double *length) {
    // Cette fonction prend un modele et une position du modele et trouve l'endroit ou le muscle 1 et 2 quittent le wrapping object

    wrapPoints(RT(model,Q), p1_bone, p2_bone, p1, p2, length);
}

void biorbd::muscles::WrappingCylinder::wrapPoints(
        biorbd::utils::Node3d& p1,
        biorbd::utils::Node3d& p2,
        double *length){
    p1 = *m_p1Wrap;
    p2 = *m_p2Wrap;
    if (length != nullptr)
        *length = *m_lengthAroundWrap;
}

const biorbd::utils::RotoTrans& biorbd::muscles::WrappingCylinder::RT(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        bool updateKin)
{
    if (updateKin)
        model.UpdateKinematicsCustom(&Q);

    // Récupérer la matrice de rototrans du cylindre dans le global
    *m_RT = model.globalJCS(*m_parentName) * *m_RTtoParent;
    return *m_RT;
}

double biorbd::muscles::WrappingCylinder::diameter() const
{
    return *m_dia;
}

void biorbd::muscles::WrappingCylinder::setDiameter(double val)
{
    *m_dia = val;
}

double biorbd::muscles::WrappingCylinder::rayon() const
{
    return *m_dia/2;
}

double biorbd::muscles::WrappingCylinder::length() const
{
    return *m_length;
}

void biorbd::muscles::WrappingCylinder::setLength(double val)
{
    *m_length = val;
}

void biorbd::muscles::WrappingCylinder::findTangentToCircle(
        const biorbd::utils::Node3d& p,
        biorbd::utils::Node3d& p_tan) const {
    double p_dot = p.block(0,0,2,1).dot(p.block(0,0,2,1));
    Eigen::Vector2d Q0 = rayon()*rayon()/p_dot*p.block(0,0,2,1);
    Eigen::Matrix2d tp;
    tp << 0, -1,
          1, 0;

    Eigen::Vector2d T = rayon()/p_dot*std::sqrt(p_dot-rayon()*rayon()) * tp * p.block(0,0,2,1);

    // Sortir la tangente des deux cotés
    NodeMusclePair m(p,p);
    m.m_p1->block(0,0,2,1) = Q0 + T;
    m.m_p2->block(0,0,2,1) = Q0 - T;

    // Choisir une des deux tangente
    selectTangents(m, p_tan);
}

void biorbd::muscles::WrappingCylinder::selectTangents(
        const NodeMusclePair &p1,
        biorbd::utils::Node3d &p_tan) const {
    if (m_isCylinderPositiveSign){
        if ((*p1.m_p2)(0) >= (*p1.m_p1)(0))
            p_tan = *p1.m_p2;
        else
            p_tan = *p1.m_p1;
    } else {
        if ( (*p1.m_p2)(0) < (*p1.m_p1)(0))
            p_tan = *p1.m_p2;
        else
            p_tan = *p1.m_p1;
    }

}
bool biorbd::muscles::WrappingCylinder::findVerticalNode(
        const NodeMusclePair &glob,
        NodeMusclePair &wrapper) const {
    // Avant toute chose, s'assurer que les points wrapent
    if (!checkIfWraps(glob, wrapper)){ // S'ils ne doivent pas passer par le wrap, mettre des nan et arreter
        for (unsigned int i=0; i<3; ++i){
            (*wrapper.m_p1)(i) = static_cast<double>(NAN);
            (*wrapper.m_p2)(i) = static_cast<double>(NAN);
        }
        return false;
    }
    else{
        // S'assurer que la composante Z ne posera pas de probleme dans le calcul de la rotation
        (*wrapper.m_p1)(2) = 0;
        (*wrapper.m_p2)(2) = 0;
    }


    // Stratégie : Trouver la matrice de passage entre les points alignés en x et le cylindre. Trouver l'endroit
    // ou les points croisent avec le cylindre

    // X est la droite entre les deux points
    biorbd::utils::Node3d X(*glob.m_p2 - *glob.m_p1);
    // Z est l'axe du vide du cylindre
    biorbd::utils::Node3d Z(0,0,1);

    biorbd::utils::Node3d Y(Z.cross(X));
    // Recalculer X pour qu'il soit aligné avec le cylindre
    X = Y.cross(Z);
    // Normaliser le tout
    X = X/X.norm();
    Y = Y/Y.norm();
    Z = Z/Z.norm();
    // Concatener pour obtenir la matrice de rotation
    biorbd::utils::RotoTrans R;
    R << X(0), X(1), X(2), 0,
         Y(0), Y(1), Y(2), 0,
         Z(0), Z(1), Z(2), 0,
         0,    0,    0,    1;

    // Tourner les points dans le repere R
    biorbd::utils::Node3d globA(*glob.m_p1);
    biorbd::utils::Node3d globB(*glob.m_p2);
    biorbd::utils::Node3d wrapA(*wrapper.m_p1);
    biorbd::utils::Node3d wrapB(*wrapper.m_p2);
    globA.applyRT(R);
    globB.applyRT(R);
    wrapA.applyRT(R);
    wrapB.applyRT(R);

    // La hauteur est selon la distance relative
    (*wrapper.m_p1)(2) = (wrapA(0)-globB(0)) / (globA(0)-globB(0)) * ((*glob.m_p1)(2)-(*glob.m_p2)(2)) + (*glob.m_p2)(2);
    (*wrapper.m_p2)(2) = (wrapB(0)-globB(0)) / (globA(0)-globB(0)) * ((*glob.m_p1)(2)-(*glob.m_p2)(2)) + (*glob.m_p2)(2);

    return true;
}
bool biorbd::muscles::WrappingCylinder::checkIfWraps(
        const NodeMusclePair &glob,
        NodeMusclePair &wrapper) const {
    bool isWrap = true;

    // Premiers tests rapides
    // si les deux points sont a gauche et on doit passer a gauche
    if (m_isCylinderPositiveSign){
        if ((*glob.m_p1)(0) > rayon() && (*glob.m_p2)(0) > rayon())
            isWrap = false;
    }
    // si les deux points sont a droite et on doit passer a droite
    else{
        if ((*glob.m_p1)(0) < -rayon() && (*glob.m_p2)(0) < -rayon())
            isWrap = false;
    }

    // Si on est en haut du wrap*, il est impossible de déterminer puisque le wrap
    // n'est pas un cylindre, mais un demi-cylindre
    // * en haut lorsque vue de dessus avec l'axe y pointant vers le haut...
    if ( ( (*glob.m_p1)(1) > 0 && (*glob.m_p2)(1) > 0) || ( (*glob.m_p1)(1) < 0 && (*glob.m_p2)(1) < 0) )
        isWrap = false;

    // Si on a une hauteur* plus petite que le rayon, il y a une abération numérique
    // * en haut lorsque vue de dessus avec l'axe y pointant vers le haut...
    if ( abs( (*glob.m_p1)(1)) < rayon() || abs( (*glob.m_p2)(1)) < rayon() )
        isWrap = false;

    // Si on est rendu ici, c'est qu'il ne reste un test
    // Si la ligne droite entre les deux points traversent le cylindre, c'est qu'il y a un wrap
    if (    ( (*wrapper.m_p1)(0) < (*wrapper.m_p2)(0) && (*glob.m_p1)(0) > (*glob.m_p2)(0)) ||
            ( (*wrapper.m_p1)(0) > (*wrapper.m_p2)(0) && (*glob.m_p1)(0) < (*glob.m_p2)(0))   )
        isWrap = false;
    else
        isWrap = true;

    // Retourner la réponse
    return isWrap;
}
double biorbd::muscles::WrappingCylinder::computeLength(const NodeMusclePair &p) const {
    double arc = std::acos(    ( (*p.m_p1)(0) * (*p.m_p2)(0) + (*p.m_p1)(1) * (*p.m_p2)(1))
                                            /
             std::sqrt( ((*p.m_p1)(0) * (*p.m_p1)(0) + (*p.m_p1)(1) * (*p.m_p1)(1)) *
                        ( (*p.m_p2)(0) * (*p.m_p2)(0) + (*p.m_p2)(1) * (*p.m_p2)(1))   )
            ) * rayon();

    return std::sqrt(arc*arc + ( (*p.m_p1)(2) - (*p.m_p2)(2)) * ( (*p.m_p1)(2) - (*p.m_p2)(2))  );
}
