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
///
/// \brief Class Geometry
///
class BIORBD_API Geometry
{
public:
    ///
    /// \brief Construct geometry
    ///
    Geometry();
    ///
    /// \brief Construct geometry
    /// \param origin The origin node
    /// \param insertion The insertion node
    /// 
    Geometry(
            const biorbd::utils::Node3d &origin,
            const biorbd::utils::Node3d &insertion);

    ///
    /// \brief Deep copy of a geometry
    /// \return A deep copy of a geometry
    ///
    biorbd::muscles::Geometry DeepCopy() const;

    ///
    /// \brief Deep copy of a geometry from another geometry
    /// \param other The geometry to copy
    ///
    void DeepCopy(const biorbd::muscles::Geometry& other);

    // Function to call before calling length/velocity or others!

    ///
    /// \brief Update kinematics
    /// \param model The model
    /// \param Q The position variables
    /// \param Qdot The velocity variables 
    /// \param updateKin TODO (default: 2)
    ///
    void updateKinematics(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr,
            int updateKin = 2);

    ///
    /// \brief Update kinematics
    /// \param model The model
    /// \param c The muscle characteristics
    /// \param o The path changers
    /// \param Q The position variables
    /// \param Qdot The velocity variables 
    /// \param updateKin TODO (default: 2)
    ///
    void updateKinematics(
            biorbd::rigidbody::Joints &model,
            const biorbd::muscles::Characteristics&c,
            biorbd::muscles::PathChangers&o,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr,
            int updateKin = 2);

    ///
    /// \brief Update kinematics
    /// \param musclePointsInGlobal Position of all the points in global
    /// \param jacoPointsInGlobal Position of all the Jacobian points in global
    /// \param Qdot The velocity variables 
    ///
    void updateKinematics(
            std::vector<biorbd::utils::Node3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr);

    ///
    /// \brief Update kinematics
    /// \param musclePointsInGlobal Position of all the points in global
    /// \param jacoPointsInGlobal Position of all the Jacobian points in global
    /// \param characteristics The muscle characteristics
    /// \param Qdot The velocity variables 
    ///
    void updateKinematics(
            std::vector<biorbd::utils::Node3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::rigidbody::GeneralizedCoordinates* Qdot = nullptr);

    ///
    /// \brief Return the origin node
    /// \return The origin node
    ///
    const biorbd::utils::Node3d& originInLocal() const;

    ///
    /// \brief Set the origin node
    /// \param val The origin node to set
    ///
    void setOriginInLocal(
            const biorbd::utils::Node3d &val);

    ///
    /// \brief Return the insertion node
    /// \return The insertion node
    ///
    const biorbd::utils::Node3d& insertionInLocal() const;

    ///
    /// \brief Set the insertion node
    /// \param val THe insertion node value to set
    ///
    void setInsertionInLocal(
            const biorbd::utils::Node3d &val);

    // Position of the muscles in space

    ///
    /// \brief Return the origin node in the global reference
    /// \return The origin node in the global reference 
    ///
    /// Geometry must be computed at least once before calling originInLocal()
    ///
    const biorbd::utils::Node3d& originInGlobal() const;

    ///
    /// \brief Return the insertion node in the global reference
    /// \return The insertion node in the global reference 
    ///
    /// Geometry must be computed at least once before calling
    ///
    const biorbd::utils::Node3d& insertionInGlobal() const; 

    ///
    /// \brief Return the muscles points in the global reference
    /// \return The muscle points in the global reference 
    ///
    /// Geometry must be computed at least once before calling. Return already computed via points
    ///
    const std::vector<biorbd::utils::Node3d>& musclesPointsInGlobal() const;

    // Retour des longueur et vitesse musculaires

    ///
    /// \brief Return the already computed muscle length
    /// \return The already computed muscled lengh
    ///
    double length() const; // Return the already computed muscle length

    ///
    /// \brief Return the already computed muscle-tendon length
    /// \return The already computed muscle-tendon length
    ///
    double musculoTendonLength() const;

    ///
    /// \brief Return the already computed velocity
    /// \return The already computed velocity
    ///
    double velocity() const;

    // Retour des jacobiennes

    ///
    /// \brief Return the last Jacobian
    /// \return The last Jacobian
    ///
    const biorbd::utils::Matrix& jacobian() const;

    ///
    /// \brief Return the origin Jacobian
    /// \return The origin Jacobian
    ///
    biorbd::utils::Matrix jacobianOrigin() const;

    ///
    /// \brief Return the insertion Jacobian
    /// \return the insertion Jacobian
    ///
    biorbd::utils::Matrix jacobianInsertion() const ;

    ///
    /// \brief Return the Jacobian
    /// \param idxMarker The markers identification
    /// \return The Jacobian
    ///
    biorbd::utils::Matrix jacobian(
            unsigned int idxMarker) const;

    /// 
    /// \brief Return the Jacobian length
    /// \return The Jacobian length
    ///
    const biorbd::utils::Matrix& jacobianLength() const;


protected:
    // Update commun de la cinématique
    /// 
    /// \brief Update the kinematics
    /// \param Qdot The velocity variables
    /// \param c The muscle characteristics
    /// \param o Path changers
    ///
    void _updateKinematics(
            const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            const biorbd::muscles::Characteristics* c = nullptr,
            biorbd::muscles::PathChangers* o = nullptr);

    // Calcul de la position des points dans le global

    /// 
    /// \brief Updates the kinematics and return the position of the origin node in space
    /// \param model The model
    /// \param Q The position variables of the model
    /// \return The position of the origin node in space
    ///
    const biorbd::utils::Node3d& originInGlobal(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q);

    /// 
    /// \brief Updates the kinematics and return the position of the insertion node in space
    /// \param model The model
    /// \param Q The position variables of the model
    /// \return The position of the insertion node in space
    ///
    const biorbd::utils::Node3d& insertionInGlobal(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q); 

    /// 
    /// \brief Set the muscle points in global
    /// \param ptsInGlobal The new muscle points in global
    ///
    void musclesPointsInGlobal(
            std::vector<biorbd::utils::Node3d>& ptsInGlobal); 

    ///
    /// \brief Set the muscle points in global
    /// \param model The model
    /// \param Q The position variables of the model
    /// \param pathChangers The path changers
    ///
    void musclesPointsInGlobal(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            biorbd::muscles::PathChangers* pathChangers = nullptr);

  
    ///
    /// \brief Update the kinematics, compute and return the muscle length
    /// \param characteristics The muscle characteristics
    /// \param pathChanger The path changer
    /// \return The muscle length
    ///
    double length(
            const biorbd::muscles::Characteristics* characteristics = nullptr,
            biorbd::muscles::PathChangers* pathChanger = nullptr); // Update the kinematics and compute and return muscle length

    ///
    /// \brief Update the kinematics, compute and return the muscle velocity assuming not via points nor wrapping objects
    /// \param Qdot The velocity variables
    /// \return The muscle velocity
    ///
    double velocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot); 
    // Calcul des jacobiennes des points
    ///
    /// \brief Set the Jacobian dimensions
    /// \param model The model
    ///
    void setJacobianDimension(
            biorbd::rigidbody::Joints &model);

    ///
    /// \brief Force a Jacobian
    /// \param jaco The Jacobian 
    ///
    void jacobian(
            const biorbd::utils::Matrix &jaco); // Forcer une jacobienne
    
    ///
    /// \brief Construct a Jacobian
    /// \param model The model
    /// \param Q The position variables
    ///
    void jacobian(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q);
    ///
    /// \brief Compute the Jacobian length
    ///
    void computeJacobianLength();

    // Position des nodes dans le repere local
    std::shared_ptr<biorbd::utils::Node3d> m_origin; ///< Origin node
    std::shared_ptr<biorbd::utils::Node3d> m_insertion; ///< Insertion node

    std::shared_ptr<biorbd::utils::Node3d> m_originInGlobal; ///< Position of the origin in the global reference
    std::shared_ptr<biorbd::utils::Node3d> m_insertionInGlobal; ///< Position of the insertion node in the global reference
    std::shared_ptr<std::vector<biorbd::utils::Node3d>> m_pointsInGlobal; ///< Position of all the points in the global reference
    std::shared_ptr<std::vector<biorbd::utils::Node3d>> m_pointsInLocal; ///< Position of all the points in local
    std::shared_ptr<biorbd::utils::Matrix> m_jacobian; ///<The Jacobian matrix
    std::shared_ptr<biorbd::utils::Matrix> m_G; ///< TODO
    std::shared_ptr<biorbd::utils::Matrix> m_jacobianLength; ///< The Jacobian length

    std::shared_ptr<double> m_length; ///< Muscle length
    std::shared_ptr<double> m_muscleTendonLength; ///< Muscle tendon length
    std::shared_ptr<double> m_velocity; ///< Velocity of the muscular elongation

    std::shared_ptr<bool> m_isGeometryComputed; ///< To know if the geometry was computed at least once
    std::shared_ptr<bool> m_isVelocityComputed; ///< To know if the velocity was computed in the last update
    std::shared_ptr<bool> m_posAndJacoWereForced; ///< To know if the override was used on the muscle position and the Jacobian

};

}}

#endif // BIORBD_MUSCLES_GEOMETRY_H
