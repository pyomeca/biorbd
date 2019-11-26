#ifndef BIORBD_MUSCLES_H
#define BIORBD_MUSCLES_H

#include "biorbdConfig.h"
#include "Muscles/Compound.h"

namespace biorbd {
namespace utils {
class Matrix;
class Vector3d;
}

namespace muscles {
class Geometry;
class Characteristics;
class State;
///
/// \brief Class Muscle
///
class BIORBD_API Muscle : public biorbd::muscles::Compound
{
public:
    /// 
    /// \brief Construct a muscle
    ///
    Muscle();
    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    ///
    Muscle(
            const biorbd::utils::String& name, 
            const biorbd::muscles::Geometry& position, 
            const biorbd::muscles::Characteristics& characteristics); 
    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    /// \param dynamicState Dynamic state
    ///  
    Muscle(
            const biorbd::utils::String& name, 
            const biorbd::muscles::Geometry& position, 
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::StateDynamics& dynamicState);

    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    /// \param wrap The wrap
    ///  
    Muscle(
            const biorbd::utils::String& name, 
            const biorbd::muscles::Geometry& position, 
            const biorbd::muscles::Characteristics& characteristics, 
            const biorbd::muscles::PathChangers& wrap);
    ///
    /// \brief Construct a muscle
    /// \param name Name of the muscle
    /// \param position Position of the origin/insertion
    /// \param characteristics Muscle characteristics from an initial state
    /// \param wrap The wrap
    /// \param dynamicState The dynamic state
    ///  
    Muscle(
            const biorbd::utils::String& name, 
            const biorbd::muscles::Geometry&position, 
            const biorbd::muscles::Characteristics&characteristics, 
            const biorbd::muscles::PathChangers&wrap, 
            const biorbd::muscles::StateDynamics&dynamicState); 

    ///
    /// \brief Construct a muscle from another muscle
    /// \param muscle The other muscle 
    ///
    Muscle(
            const biorbd::muscles::Muscle& muscle);
    ///
    /// \brief Construct a muscle from another muscle
    /// \param muscle The other muscle (pointer)
    ///
    Muscle(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Muscle();
    
    ///
    /// \brief Deep copy of a muscle in new muscle
    /// \param other The muscle to copy
    ///
    void DeepCopy(const biorbd::muscles::Muscle& other);

    // Get and set

    ///
    /// \brief Get the length of the muscle
    /// \param model The model
    /// \param Q The position variables
    /// \param updateKin Update kinematics (default: 2)
    /// \return The length of the muscle
    ///
    double length(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            int updateKin = 2);

    ///
    /// \brief Return the musculo tendon length
    /// \param model The model
    /// \param Q The position variables
    /// \param updateKin Update kinematics (default: 2)
    /// \return The musculo tendon length
    ///
    double musculoTendonLength(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            int updateKin = 2);

    ///
    /// \brief Return the velocity of the muscle
    /// \param model The model
    /// \param Q The position variables
    /// \param Qdot The velocity variables
    /// \param updateKin Update kinematics (default: true)
    /// \return The velocity of the muscle
    ///
    double velocity(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            bool updateKin = true);

    ///
    /// \brief Update the position of the origin and insertion nodes of the muscle
    /// \param model The model
    /// \param Q The position variables
    /// \param updateKin Update kinematics (default: 2)
    ///
    void updateOrientations(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            int updateKin = 2);

    ///
    /// \brief Update the position of the origin and insertion nodes of the muscle
    /// \param model The model
    /// \param Q The position variables
    /// \param Qdot The velocity variables
    /// \param updateKin Update kinematics (default: 2)
    ///
    void updateOrientations(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            int updateKin = 2);

    ///
    /// \brief Update the position of the origin and insertion nodes of the muscle
    /// \param musclePointsInGlobal The muscle points in space
    /// \param jacoPointsInGlobal The Jacobian points in space
    ///
    void updateOrientations(
            std::vector<biorbd::utils::Vector3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal); 

    ///
    /// \brief Update the position of the origin and insertion nodes of the muscle
    /// \param musclePointsInGlobal The muscle points in space
    /// \param jacoPointsInGlobal The Jacobian points in space
    /// \param Qdot The velocity variables
    ///
    void updateOrientations(
            std::vector<biorbd::utils::Vector3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot); 

    ///
    /// \brief Return the position
    /// \return The position
    ///
    const biorbd::muscles::Geometry& position() const;

    ///
    /// \brief Return the muscle characteristics
    /// \return The muscle characteristics
    ///
    const biorbd::muscles::Characteristics& characteristics() const;

    ///
    /// \brief Set the position
    /// \param val New value of the position
    ///
    void setPosition(const biorbd::muscles::Geometry &val);
    
    ///
    /// \brief Set the muscle characteristics
    /// \param val New value of the muscle characteristics
    ///
    void setCharacteristics(const biorbd::muscles::Characteristics &val);

    ///
    /// \brief Return the muscle points in space
    /// \param j The model
    /// \param Q The position variables
    /// \return The muscle points in space
    ///
    const std::vector<biorbd::utils::Vector3d>& musclesPointsInGlobal(
            biorbd::rigidbody::Joints &j,
            const biorbd::rigidbody::GeneralizedCoordinates &Q);
    ///
    /// \brief Return the muscle points in space
    /// \return The muscle points in space
    ///
    const std::vector<biorbd::utils::Vector3d>& musclesPointsInGlobal() const;

    ///
    /// \brief Set the maximal isometric force
    /// \param val The force to set
    ///
    void forceIsoMax(double val);

    ///
    /// \brief Set the dynamic state
    /// \param s The dynamic state value
    ///
    void setState(const biorbd::muscles::StateDynamics &s);
    ///
    /// \brief Return the dynamic state
    /// \return The dynamic state
    ///
    const biorbd::muscles::StateDynamics& state() const;

    ///
    /// \brief Return the dynamic state
    /// \return The dynamic state
    ///
    biorbd::muscles::StateDynamics& state();

    ///
    /// \brief Return the activation time
    /// \param s The dynamic state
    /// \param already (default: false)
    /// \return The activation time
    ///
    double activationDot(const biorbd::muscles::StateDynamics &s, bool already =false);
protected:
    ///
    /// \brief Computer the forces
    /// \param emg EMG data
    ///
    virtual void computeForce(const biorbd::muscles::State &emg);
    ///
    /// \brief Return the force from activation
    /// \param emg EMG data
    /// \return The force from activation
    ///
    virtual double getForceFromActivation(const biorbd::muscles::State &emg) = 0;

    std::shared_ptr<biorbd::muscles::Geometry> m_position; ///< The position of the origin and insertion nodes of the muscle
    std::shared_ptr<biorbd::muscles::Characteristics> m_characteristics; ///< The muscle characterisitcs
    std::shared_ptr<biorbd::muscles::StateDynamics> m_state; ///< The dynamic state

};

}}

#endif // BIORBD_MUSCLES_H
