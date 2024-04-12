#ifndef BIORBD_LIGAMENTS_LIGAMENTS_H
#define BIORBD_LIGAMENTS_LIGAMENTS_H

#include <vector>
#include <memory>

#include "biorbdConfig.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
class Matrix;
class Vector;
class Vector3d;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedTorque;
}

namespace internal_forces
{
namespace ligaments
{
class Ligament;

///
/// \brief Ligament holder
///
class BIORBD_API Ligaments
{
public:
    ///
    /// \brief Construct ligaments
    ///
    Ligaments();

    ///
    /// \brief Construct ligaments from other ligaments
    /// \param other The other ligaments
    ///
    Ligaments(const Ligaments& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Ligaments();

    ///
    /// \brief Deep copy of ligaments
    /// \return A deep copy of ligaments
    ///
    Ligaments DeepCopy() const;

    ///
    /// \brief Deep copy of ligaments into another mucles
    /// \param other The ligaments to copy
    ///
    void DeepCopy(
        const Ligaments& other);

    ///
    /// \brief Returns all the ligaments. It sorts the ligaments by group
    /// \return All the ligament
    ///
    std::vector<std::shared_ptr<Ligament>>& ligaments();


    ///
    /// \brief Returns all the ligaments. It sorts the ligaments by group
    /// \return All the ligament
    ///
    const std::vector<std::shared_ptr<Ligament>>& ligaments() const;


    ///
    /// \brief Returns a specific ligament sorted by ligaments()
    /// \param idx The ligament index
    /// \return The ligament
    ///
    ///
    Ligament& ligament(size_t idx);

    ///
    /// \brief Returns a specific ligament sorted by ligaments()
    /// \param idx The ligament index
    /// \return The ligament
    ///
    ///
    const Ligament& ligament(size_t idx) const;

    ///
    /// \brief ligamentNames Return the names for all the ligament ordered by their
    /// respective name
    /// \return All the ligament names
    ///
    std::vector<utils::String> ligamentNames() const;

    ///
    /// \brief Compute the ligament joint torque
    /// \param F The force vector of all the ligaments
    ///
    /// The computation for the ligament joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the ligament lengths jacobian and \f$F\f$ is the force vector of all the ligaments
    ///
    /// Warning: This function assumes that ligaments are already updated (via `updateligaments`)
    ///
    rigidbody::GeneralizedTorque ligamentsJointTorque(
        const utils::Vector& F);

    ///
    /// \brief Compute the ligament joint torque
    /// \param F The force vector of all the ligaments
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateLigamentKinematics If the ligament kinematics parameters should be updated
    ///
    /// This function updates the ligaments and then performs the computation for
    /// the ligament joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the ligament lengths jacobian and \f$F\f$ is the force vector of all the ligaments
    ///
    /// Warning: This function assumes that ligaments are already updated (via `updateligaments`)
    ///
    rigidbody::GeneralizedTorque ligamentsJointTorque(
        const utils::Vector& F,
        rigidbody::Joints &updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot, 
        bool updateLigamentKinematics = true);


    ///
    /// \brief Compute the ligament joint torque
    /// \param F The force vector of all the ligaments
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics should be updated (0 => no update required, 1 => Only ligament parameters, 2 => Joints model and ligaments)
    ///
    /// This function updates the ligaments and then performs the computation for
    /// the ligament joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the ligament lengths jacobian and \f$F\f$ is the force vector of all the ligaments
    ///
    /// Warning: This function assumes that ligaments are already updated (via `updateligaments`)
    ///
    rigidbody::GeneralizedTorque ligamentsJointTorque(
        const utils::Vector& F,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        int updateKin = 2);

    ///
    /// \brief Compute the ligament joint torque
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateLigamentKinematics If the ligament kinematics parameters should be updated
    ///
    /// This function updates the ligaments and then performs the computation for
    /// the ligament joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the ligament lengths jacobian and \f$F\f$ is the force vector of all the ligaments
    ///
    /// Warning: This function assumes that ligaments are already updated (via `updateligaments`)
    ///
    rigidbody::GeneralizedTorque ligamentsJointTorque(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot, 
        bool updateLigamentKinematics = true);

    ///
    /// \brief Compute the ligament joint torque
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics should be updated (0 => no update required, 1 => Only ligament parameters, 2 => Joints model and ligaments)
    ///
    /// This function updates the ligaments and then performs the computation for
    /// the ligament joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the ligament lengths jacobian and \f$F\f$ is the force vector of all the ligaments
    ///
    /// Warning: This function assumes that ligaments are already updated (via `updateligaments`)
    ///
    rigidbody::GeneralizedTorque ligamentsJointTorque(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        int updateKin = 2);

    ///
    /// \brief Return the previously computed ligament length jacobian
    /// \return The ligament length jacobian
    ///
    utils::Matrix ligamentsLengthJacobian() const;

    ///
    /// \brief Compute and return the ligament length Jacobian
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param updateLigamentKinematics If the ligament kinematics parameters should be updated
    /// \return The ligament length Jacobian
    ///
    utils::Matrix ligamentsLengthJacobian(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        bool updateLigamentKinematics = true);

    ///
    /// \brief Compute and return the ligament length Jacobian
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be updated (0 => no update required, 1 => Only ligament parameters, 2 => Joints model and ligaments)
    /// \return The ligament length Jacobian
    ///
    utils::Matrix ligamentsLengthJacobian(
        const rigidbody::GeneralizedCoordinates& Q,
        int updateKin = 2);

    ///
    /// \brief Compute and return the ligament forces
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param updateLigamentKinematics If the kinematics parameters of the ligaments should be updated
    /// \return The ligament forces
    ///
    utils::Vector ligamentForces(
        rigidbody::Joints &updatedModel,
        const rigidbody::GeneralizedCoordinates& Q, 
        bool updateLigamentKinematics = true);

    ///
    /// \brief Compute and return the ligament forces
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be updated (0 => no update required, 1 => Only ligament parameters, 2 => Joints model and ligaments)
    /// \return The ligament forces
    ///
    utils::Vector ligamentForces(
        const rigidbody::GeneralizedCoordinates& Q,
        int updateKin = 2);

    ///
    /// \brief Compute and return the ligament forces
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateLigamentKinematics If the kinematics parameters of the ligaments should be updated
    /// \return The ligament forces
    ///
    utils::Vector ligamentForces(
        rigidbody::Joints& updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        bool updateLigamentKinematics = true);

    ///
    /// \brief Compute and return the ligament forces
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics should be updated (0 => no update required, 1 => Only ligament parameters, 2 => Joints model and ligaments)
    /// \return The ligament forces
    ///
    utils::Vector ligamentForces(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        int updateKin = 2);

    ///
    /// \brief Return the total number of ligament
    /// \return The total number of ligaments
    ///
    size_t nbLigaments() const;

    ///
    /// \brief Return the ligament index
    /// \param name The name of the ligament
    /// \return The ligament index
    ///
    int ligamentID(const utils::String& name);

    ///
    /// \brief Add a ligament to the set of ligaments
    /// \param torque The ligament to add
    ///
    void addLigament(const Ligament &ligament);

    ///
    /// \brief Update all the ligaments (positions, jacobian, etc.)
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    ///
    void updateLigaments(
        rigidbody::Joints &updatedModel,
        const rigidbody::GeneralizedCoordinates& Q);

    ///
    /// \brief Update all the ligaments (positions, jacobian, etc.)
    /// \param Q The generalized coordinates
    /// \param updateKin If the joint model should be updated
    ///
    void updateLigaments(
        const rigidbody::GeneralizedCoordinates& Q, 
        bool updateKin = true);

    ///
    /// \brief Update all the ligaments (positions, jacobian, etc.)
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    ///
    void updateLigaments(
        rigidbody::Joints &updatedModel,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot);

    ///
    /// \brief Update all the ligaments (positions, jacobian, etc.)
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the joint model should be updated
    ///
    void updateLigaments(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot, 
        bool updateKin = true);

    ///
    /// \brief Update by hand all the ligaments (positions, jacobian, velocity, etc.)
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param ligamentPointsInGlobal The ligament points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    /// \param Qdot The generalized velocities
    ///
    void updateLigaments(
        rigidbody::Joints &updatedModel,
        std::vector<std::vector<utils::Vector3d>>& ligamentPointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal,
        const rigidbody::GeneralizedVelocity& Qdot);

    ///
    /// \brief Update by hand all the ligaments (positions, jacobian, velocity, etc.)
    /// \param ligamentPointsInGlobal The ligament points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    ///
    void updateLigaments(
        std::vector<std::vector<utils::Vector3d>>& ligamentPointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal);

protected:
    std::shared_ptr<std::vector<std::shared_ptr<Ligament>>>
            m_ligaments; ///< Holder for ligament groups
};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENTS_H

