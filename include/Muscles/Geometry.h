#ifndef BIORBD_MUSCLES_GEOMETRY_H
#define BIORBD_MUSCLES_GEOMETRY_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Matrix;
class Vector;
class Node3d;
}

namespace rigidbody {
class Joints;
class GeneralizedCoordinates;
}

namespace muscles {
class PathChangers;
class Characteristics;

class BIORBD_API Geometry
{
public:
    Geometry();
    Geometry(
            const biorbd::utils::Node3d &origin,
            const biorbd::utils::Node3d &insertion);
    biorbd::muscles::Geometry DeepCopy() const;
    void DeepCopy(const biorbd::muscles::Geometry& other);

    // Fonction a appeler avant d'appeler longueur/velocity ou autres!
    void updateKinematics(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr,
            int updateKin = 2);
    void updateKinematics(
            biorbd::rigidbody::Joints &model,
            const biorbd::muscles::Characteristics&,
            biorbd::muscles::PathChangers&,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr,
            int updateKin = 2);
    void updateKinematics(
            std::vector<biorbd::utils::Node3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr);
    void updateKinematics(
            std::vector<biorbd::utils::Node3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr);

    // Get and set des position d'origine et insertions
    const biorbd::utils::Node3d& originInLocal() const;
    void setOriginInLocal(
            const biorbd::utils::Node3d &val);
    const biorbd::utils::Node3d& insertionInLocal() const;
    void setInsertionInLocal(
            const biorbd::utils::Node3d &val);

    // Position des muscles dans l'espace
    const biorbd::utils::Node3d& originInGlobal() const; // Attention il est impératif d'avoir update une premiere fois pour que ceci fonctionne!
    const biorbd::utils::Node3d& insertionInGlobal() const; // Attention il est impératif d'avoir update une premiere fois pour que ceci fonctionne!
    const std::vector<biorbd::utils::Node3d>& musclesPointsInGlobal() const; // Return already computed via points

    // Retour des longueur et vitesse musculaires
    double length() const; // Return the already computed muscle length
    double musculoTendonLength() const; // Return the already computed muscle-tendon length
    double velocity() const; // Return the already computed velocity

    // Retour des jacobiennes
    const biorbd::utils::Matrix& jacobian() const; // Retourne la derniere jacobienne
    biorbd::utils::Matrix jacobianOrigin() const;
    biorbd::utils::Matrix jacobianInsertion() const ;
    biorbd::utils::Matrix jacobian(
            unsigned int idxMarker) const;
    const biorbd::utils::Matrix& jacobianLength() const;


protected:
    // Update commun de la cinématique
    void _updateKinematics(
            const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            const biorbd::muscles::Characteristics* c = nullptr,
            biorbd::muscles::PathChangers* o = nullptr);

    // Calcul de la position des points dans le global
    const biorbd::utils::Node3d& originInGlobal(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q); // Update la cinématique puis retourne la position de l'origine dans l'espace
    const biorbd::utils::Node3d& insertionInGlobal(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q); // Update la cinématique puis retourne la position de l'insertion dans l'espace
    void musclesPointsInGlobal(
            std::vector<biorbd::utils::Node3d>& ptsInGlobal); // Forcer les points dans le global
    void musclesPointsInGlobal(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            biorbd::muscles::PathChangers* pathChangers = nullptr);

    // Calcul de la longueur musculaire
    double length(
            const biorbd::muscles::Characteristics* characteristics = nullptr,
            biorbd::muscles::PathChangers* pathChanger = nullptr); // Update the kinematics and compute and return muscle length
    // Calcul de la vitesse musculaire
    double velocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot); // Update the kinematics and compute and return muscle velocity assuming no via points nor wrapping objects
    // Calcul des jacobiennes des points
    void setJacobianDimension(
            biorbd::rigidbody::Joints &model);
    void jacobian(
            const biorbd::utils::Matrix &jaco); // Forcer une jacobienne
    void jacobian(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q);
    void computeJacobianLength();

    // Position des nodes dans le repere local
    std::shared_ptr<biorbd::utils::Node3d> m_origin; // Node de l'origine
    std::shared_ptr<biorbd::utils::Node3d> m_insertion; // Node de l'insertion

    std::shared_ptr<biorbd::utils::Node3d> m_originInGlobal; // Position de l'origine dans le repere global
    std::shared_ptr<biorbd::utils::Node3d> m_insertionInGlobal; // Position de l'origine dans le repere global
    std::shared_ptr<std::vector<biorbd::utils::Node3d>> m_pointsInGlobal; // position de tous les points dans le global
    std::shared_ptr<std::vector<biorbd::utils::Node3d>> m_pointsInLocal; // position de tous les points dans le global
    std::shared_ptr<biorbd::utils::Matrix> m_jacobian;
    std::shared_ptr<biorbd::utils::Matrix> m_G;
    std::shared_ptr<biorbd::utils::Matrix> m_jacobianLength; // Incluant la

    std::shared_ptr<double> m_length; // Longueur du muscle
    std::shared_ptr<double> m_muscleTendonLength; // Longueur du muscle et du tendon
    std::shared_ptr<double> m_velocity; // Vitesse de l'élongation musculaire

    std::shared_ptr<bool> m_isGeometryComputed; // Savoir si la geometry a été faite au moins une fois
    std::shared_ptr<bool> m_isVelocityComputed; // Savoir si dans le dernier update, la vitesse a été calculée
    std::shared_ptr<bool> m_posAndJacoWereForced; // On a utilisé la procédure d'override de la position du muscle et de la jacobienne

};

}}

#endif // BIORBD_MUSCLES_GEOMETRY_H
