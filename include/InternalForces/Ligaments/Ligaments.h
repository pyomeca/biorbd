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
    /// \brief Construct muscles
    ///
    Ligaments();

    ///
    /// \brief Construct muscles from other muscles
    /// \param other The other muscles
    ///
    Ligaments(const Ligaments& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Ligaments();

    ///
    /// \brief Deep copy of muscles
    /// \return A deep copy of muscles
    ///
    Ligaments DeepCopy() const;

    ///
    /// \brief Deep copy of muscles into another mucles
    /// \param other The muscles to copy
    ///
    void DeepCopy(
        const Ligaments& other);

    ///
    /// \brief Returns all the muscles. It sorts the muscles by group
    /// \return All the muscle
    ///
    const std::vector<std::shared_ptr<Ligament>> ligaments() const;

    ///
    /// \brief Returns a specific muscle sorted by ligaments()
    /// \param idx The ligament index
    /// \return The ligament
    ///
    const Ligament& ligament(
        unsigned int idx) const;

    ///
    /// \brief ligamentNames Return the names for all the ligament ordered by their
    /// respective name
    /// \return All the ligament names
    ///
    std::vector<utils::String> ligamentNames() const;

    ///
    /// \brief Update all the ligaments (positions, jacobian, etc.)
    /// \param Q The generalized coordinates
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and ligaments])
    ///
    void updateLigaments(
        const rigidbody::GeneralizedCoordinates& Q,
        bool updateKin);

    ///
    /// \brief Update all the muscles (positions, jacobian, etc.)
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and muscles])
    ///
    void updateLigaments(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& QDot,
        bool updateKin);

    ///
    /// \brief Update by hand all the muscles (positions, jacobian, velocity, etc.)
    /// \param musclePointsInGlobal The muscle points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    ///
    void updateLigaments(
        std::vector<std::vector<utils::Vector3d>>& ligamentPointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal);

    ///
    /// \brief Update by hand all the muscles (positions, jacobian, velocity, etc.)
    /// \param musclePointsInGlobal The muscle points in global reference frame
    /// \param jacoPointsInGlobal The jacobian points in global reference frame
    /// \param QDot The generalized velocities
    ///
    void updateLigaments(
        std::vector<std::vector<utils::Vector3d>>& LigamentPointsInGlobal,
        std::vector<utils::Matrix>& jacoPointsInGlobal,
        const rigidbody::GeneralizedVelocity& QDot);

    ///
    /// \brief Compute the muscular joint torque
    /// \param F The force vector of all the muscles
    ///
    /// The computation for the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    /// Warning: This function assumes that muscles are already updated (via `updateMuscles`)
    ///
    rigidbody::GeneralizedTorque ligamentJointTorque(
        const utils::Vector& F);

    ///
    /// \brief Compute the muscular joint torque
    /// \param F The force vector of all the muscles
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
    ///
    /// This function updates the muscles and then performs the computation for
    /// the muscular joint torque is done from virtual power:
    ///
    /// i.e. \f$-J \times F\f$
    ///
    /// where \f$J\f$ is the muscle lengths jacobian and \f$F\f$ is the force vector of all the muscles
    ///
    /// Warning: This function assumes that muscles are already updated (via `updateMuscles`)
    ///
    rigidbody::GeneralizedTorque ligamentJointTorque(
        const utils::Vector& F,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& QDot);

    ///
    /// \brief Return the previously computed muscle length jacobian
    /// \return The muscle length jacobian
    ///
    utils::Matrix ligamentLengthJacobian();

    ///
    /// \brief Compute and return the muscle length Jacobian
    /// \param Q The generalized coordinates
    /// \return The muscle length Jacobian
    ///
    utils::Matrix ligamentLengthJacobian(
        const rigidbody::GeneralizedCoordinates& Q);

    ///
    /// \brief Compute and return the muscle forces
    /// \param emg The dynamic state
    /// \return The muscle forces
    ///
    /// Warning: This function assumes that muscles are already updated (via `updateMuscles`)
    ///
    utils::Vector ligamentForces();

    ///
    /// \brief Compute and return the muscle forces
    /// \param emg The dynamic state
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
    /// \return The muscle forces
    ///
    utils::Vector ligamentForces(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& QDot);

    ///
    /// \brief Return the total number of muscle
    /// \return The total number of muscles
    ///
    unsigned int nbLigamentTotal() const;

    ///
    /// \brief Return the total number of muscle
    /// \return The total number of muscles
    ///
    unsigned int nbLigaments() const;

    ///
    /// \brief Add a passive torque to the set of passive torques
    /// \param torque The passive torque to add
    ///
    void addLigament(
        const Ligament &ligament);

protected:
    std::shared_ptr<std::vector<Ligament>>
            m_ligaments; ///< Holder for muscle groups
};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENTS_H

