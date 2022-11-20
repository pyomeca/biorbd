#ifndef BIORBD_LIGAMENTS_LIGAMENT_H
#define BIORBD_LIGAMENTS_LIGAMENT_H

#include "biorbdConfig.h"
#include "InternalForces/Compound.h"
#include "InternalForces/Ligaments/LigamentsEnums.h"
#include "InternalForces/Geometry.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class Matrix;
class Vector3d;
}

namespace internal_forces
{
class Compound;
namespace ligaments
{
class Characteristics;
class Ligaments;

///
/// \brief Base class of all ligament
///
class BIORBD_API Ligament : public Compound
{
    friend Ligaments;

public:
    ///
    /// \brief Construct a ligament
    ///
    Ligament();

    ///
    /// \brief Construct a ligament
    /// \param name Name of the ligament
    /// \param position Position of the origin/insertion
    /// \param characteristics ligament characteristics from an initial state
    ///
    Ligament(
        const utils::String& name,
        const Geometry& position,
        const Characteristics& characteristics);

    ///
    /// \brief Construct a ligament
    /// \param name Name of the ligament
    /// \param position Position of the origin/insertion
    /// \param characteristics ligament characteristics from an initial state
    /// \param pathModifiers The path modifiers
    ///
    Ligament(
        const utils::String& name,
        const Geometry& position,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a ligament from another ligament
    /// \param other The other ligament
    ///
    Ligament(
        const Ligament& other);

    ///
    /// \brief Construct a ligament from another ligament
    /// \param other The other ligament
    ///
    Ligament(
        const std::shared_ptr<Ligament> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Ligament();

    ///
    /// \brief Deep copy of a ligament in new ligament
    /// \param other The ligament to copy
    ///
    void DeepCopy(
        const Ligament& other);

    // Get and set

    ///
    /// \brief Get the length of the ligament
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    /// \return The length of the ligament
    ///
    const utils::Scalar& length(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        int updateKin = 2);

    ///
    /// \brief Return the velocity of the ligament
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    //// \return The velocity of the ligament
    ///
    const utils::Scalar& velocity(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        bool updateKin = true);

    ///
    /// \brief Update the position of the origin and insertion positions of the ligament
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    ///
    void updateOrientations(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates &Q,
        int updateKin = 2);

    ///
    /// \brief Update the position of the origin and insertion nodes of the ligament
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    ///
    void updateOrientations(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        int updateKin = 2);

    ///
    /// \brief Update by hand the position of the origin and insertion nodes of the ligament
    /// \param ligamentPointsInGlobal The ligament points
    /// \param jacoPointsInGlobal The jacobian matrix
    ///
    void updateOrientations(
        std::vector<utils::Vector3d>& ligamentPointsInGlobal,
        utils::Matrix& jacoPointsInGlobal);

    ///
    /// \brief Update by hand the position of the origin and insertion nodes of the ligament
    /// \param ligamentPointsInGlobal The ligament points
    /// \param jacoPointsInGlobal The Jacobian matrix
    /// \param Qdot The genelized velocities
    ///
    void updateOrientations(
        std::vector<utils::Vector3d>& ligamentPointsInGlobal,
        utils::Matrix& jacoPointsInGlobal,
        const rigidbody::GeneralizedVelocity &Qdot);

    ///
    /// \brief Set the position of all the points attached to the ligament (0 being the origin)
    /// \param positions New value of the position
    ///
    void setPosition(
        const Geometry &positions);

    ///
    /// \brief Return the position of all the points attached to the ligament (0 being the origin)
    /// \return The positions
    ///
    const Geometry& position() const;

    ///
    /// \brief Set the ligament characteristics
    /// \param characteristics New value of the ligament characteristics
    ///
    void setCharacteristics(
        const Characteristics &characteristics);

    ///
    /// \brief Return the ligament characteristics
    /// \return The ligament characteristics
    ///
    const Characteristics& characteristics() const;

    ///
    /// \brief Return the ligament points in global reference frame
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \return The ligament points in global reference frame
    ///
    const std::vector<utils::Vector3d>& ligamentsPointsInGlobal(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates &Q);

    ///
    /// \brief Return the previously computed ligament points in global reference frame
    /// \return The ligament points in global reference frame
    ///
    const std::vector<utils::Vector3d>& ligamentsPointsInGlobal() const;

    ///
    /// \brief Return the computed force norm
    /// \param model The joints model
    /// \param Q The generalized coordinates of the model
    /// \param Qdot The generalized velocities of the model
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    /// \return The computed force
    ///
    virtual const utils::Scalar& force(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& Qdot,
        int updateKin = 2);

    ///
    /// \brief Return the computed force norm
    /// \param model The joints model
    /// \param Q The generalized coordinates of the model
    /// \param updateKin Update kinematics (0: don't update, 1:only ligaments, [2: both kinematics and ligaments])
    /// \return The computed force
    ///
    virtual const utils::Scalar& force(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        int updateKin = 2);

    ///
    /// \brief Return the type of the ligament
    /// \return The type of the ligament
    ///
    LIGAMENT_TYPE type() const;

    ///
    /// \brief Return the Force-Length of the passive element
    /// \return The Force-Length of the passive element
    ///
    const utils::Scalar& Fl();

    ///
    /// \brief Return the ligament damping (spring force)
    /// \return The ligament damping
    ///
    const utils::Scalar& damping();

    ///
    /// \brief Compute the ligament damping
    ///
    void computeDamping();

protected:
    ///
    /// \brief Computer the forces from a specific emg
    /// \param emg EMG data
    ///
    virtual void computeForce();

    ///
    /// \brief Set the type of ligament
    ///
    virtual void setType();

    std::shared_ptr<Compound> m_compound; ///< The ligament characteristics
    std::shared_ptr<Geometry> m_position; ///< The position of all the nodes of the ligament (0 being the origin and last being insertion
    std::shared_ptr<LIGAMENT_TYPE> m_type; ///< The type of the ligament
    std::shared_ptr<Characteristics> m_characteristics; ///< The ligament characteristics
    std::shared_ptr<utils::Scalar> m_damping; ///< The ligament damping term

    ///
    /// \brief Function allowing modification of the way the multiplication is done in computeForce(EMG)
    /// \param emg The EMG data
    /// \return The force from activation
    ///
    virtual utils::Scalar getForce();

    ///
    /// \brief Compute the Force-length
    ///
    virtual void computeFl() = 0;

    // Attributs intermédiaires lors du calcul de la force
    std::shared_ptr<utils::Scalar> m_Fl; ///< Force-Length
};

}
}
}

#endif // BIORBD_LIGAMENTS_LIGAMENT_H
