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
    const std::vector<std::shared_ptr<Ligament>>& ligaments() const;

    ///
    /// \brief Returns a specific ligament sorted by ligaments()
    /// \param idx The ligament index
    /// \return The ligament
    ///
    ///
    const std::shared_ptr<Ligament>& ligament(unsigned int idx);

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
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
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
        const rigidbody::GeneralizedVelocity& QDot);

    ///
    /// \brief Compute the ligament joint torque
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
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
        const rigidbody::GeneralizedVelocity& QDot);

    ///
    /// \brief Return the previously computed ligament length jacobian
    /// \return The ligament length jacobian
    ///
    utils::Matrix ligamentsLengthJacobian();

    ///
    /// \brief Compute and return the ligament length Jacobian
    /// \param Q The generalized coordinates
    /// \return The ligament length Jacobian
    ///
    utils::Matrix ligamentsLengthJacobian(
        const rigidbody::GeneralizedCoordinates& Q);

    ///
    /// \brief Compute and return the ligament forces
    /// \param Q The generalized coordinates
    /// \return The ligament forces
    ///
    utils::Vector ligamentForces(const rigidbody::GeneralizedCoordinates& Q);

    ///
    /// \brief Compute and return the ligament forces
    /// \param emg The dynamic state
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
    /// \return The ligament forces
    ///
    utils::Vector ligamentForces(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& QDot);

    ///
    /// \brief Return the total number of ligament
    /// \return The total number of ligaments
    ///
    unsigned int nbLigaments() const;

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
    /// \param Q The generalized coordinates
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    ///
    void updateLigaments(
        const rigidbody::GeneralizedCoordinates& Q,
        bool updateKin);

    ///
    /// \brief Update all the ligaments (positions, jacobian, etc.)
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    ///
    void updateLigaments(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& QDot,
        bool updateKin);

    ///
    /// \brief Update by hand all the ligaments (positions, jacobian, velocity, etc.)
    /// \param ligamentPointsInGlobal The ligament points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    ///
    void updateLigaments(
        std::vector<std::vector<utils::Vector3d>>& ligamentPointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal);

    ///
    /// \brief Update by hand all the ligaments (positions, jacobian, velocity, etc.)
    /// \param ligamentPointsInGlobal The ligament points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    /// \param QDot The generalized velocities
    ///
    void updateLigaments(
        std::vector<std::vector<utils::Vector3d>>& ligamentPointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal,
        const rigidbody::GeneralizedVelocity& QDot);


protected:
    std::shared_ptr<std::vector<std::shared_ptr<Ligament>>>
            m_ligaments; ///< Holder for ligament groups
};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENTS_H

