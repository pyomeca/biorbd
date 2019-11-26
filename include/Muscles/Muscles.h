#ifndef BIORBD_MUSCLES_MUSCLES_H
#define BIORBD_MUSCLES_MUSCLES_H

#include <vector>
#include <memory>

#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Matrix;
class Vector;
class Vector3d;
}

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
}

namespace muscles {
class MuscleGroup;
class StateDynamics;
class Force;

///
/// \brief Class muscles
///
class BIORBD_API Muscles
{
public:
    ///
    /// \brief Construct muscles
    ///
    Muscles();
    
    ///
    /// \brief Construct muscles from other muscles
    /// \param other The other muscles
    ///
    Muscles(const biorbd::muscles::Muscles& other);
    
    /// 
    /// \brief Destroy class properly
    ///
    virtual ~Muscles();

    ///
    /// \brief Deep copy of muscles
    /// \return A deep copy of muscles
    ///
    biorbd::muscles::Muscles DeepCopy() const;
    ///
    /// \brief Deep copy of muscles into another mucles
    /// \param other The muscles to copy
    ///
    void DeepCopy(const biorbd::muscles::Muscles& other);
    ///
    /// \brief Add a muscle group
    /// \param name The name
    /// \param originName The origin name
    /// \param insertionName THe insertion name
    ///
    void addMuscleGroup(
            const biorbd::utils::String &name,
            const biorbd::utils::String &originName,
            const biorbd::utils::String &insertionName);
    ///
    /// \brief Return the group ID 
    /// \param name The name
    /// \return The group ID (returns -1 if not found)
    ///
    int getGroupId(const biorbd::utils::String &name) const;

    ///
    /// \brief Return the muscle group of specific index
    /// \param idx The index of the muscle group to return
    /// \return A muscle group
    ///
    biorbd::muscles::MuscleGroup& muscleGroup(unsigned int idx); 

    ///
    /// \brief Return the muscle group of specific index
    /// \param idx The index of the muscle group to return
    /// \return A muscle group
    ///
    const biorbd::muscles::MuscleGroup& muscleGroup(unsigned int idx) const; 

    ///
    /// \brief Return the muscle group of specific name
    /// \param name The name of the muscle group to return
    /// \return A muscle group
    ///
    const biorbd::muscles::MuscleGroup& muscleGroup(const biorbd::utils::String&name) const;

    ///
    /// \brief Update the muscles (positions, jacobian, velocity, etc.)
    /// \param Q The position variables
    /// \param updateKin Update kinematics
    ///
    void updateMuscles(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool updateKin); 
    ///
    /// \brief Update the muscles (positions, jacobian, velocity, etc.)
    /// \param Q The position variables
    /// \param QDot The velocity variables
    /// \param updateKin Update kinematics
    ///
    void updateMuscles(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& QDot,
            bool updateKin); 
    ///
    /// \brief Update the muscles (positions, jacobian, velocity, etc.)
    /// \param musclePointsInGlobal The muscle points in space
    /// \param jacoPointsInGlobal The jacobian points in space
    ///
    void updateMuscles(
            std::vector<std::vector<biorbd::utils::Vector3d>>& musclePointsInGlobal,
            std::vector<biorbd::utils::Matrix>& jacoPointsInGlobal); 

    ///
    /// \brief Update the muscles (positions, jacobian, velocity, etc.)
    /// \param musclePointsInGlobal The muscle points in space
    /// \param jacoPointsInGlobal The jacobian points in space
    /// \param QDot The velocity variables
    ///
    void updateMuscles(
            std::vector<std::vector<biorbd::utils::Vector3d>>& musclePointsInGlobal,
            std::vector<biorbd::utils::Matrix>& jacoPointsInGlobal,
            const biorbd::rigidbody::GeneralizedCoordinates& QDot);

    // Calcul des effets musculaires sur les os
    ///
    /// \brief Compute the muscular joint torque
    /// \param F The force? TODO
    /// \param updateKin Update kinematics (default: true)
    /// \param Q The position variables
    /// \param QDot The velocity variables
    /// 
    biorbd::rigidbody::GeneralizedTorque muscularJointTorque(
            const biorbd::utils::Vector& F,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);
    ///
    /// \brief Compute the muscular joint torque
    /// \param state The dynamic state
    /// \param F The force? TODO
    /// \param updateKin Update kinematics (default: true)
    /// \param Q The position variables
    /// \param QDot The velocity variables
    /// 
    biorbd::rigidbody::GeneralizedTorque muscularJointTorque(
            const std::vector<std::shared_ptr<StateDynamics>> &state,
            biorbd::utils::Vector& F,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);
    ///
    /// \brief Compute the muscular joint torque
    /// \param state The dynamic state
    /// \param updateKin Update kinematics (default: true)
    /// \param Q The position variables
    /// \param QDot The velocity variables
    /// 
    biorbd::rigidbody::GeneralizedTorque muscularJointTorque(
            const std::vector<std::shared_ptr<StateDynamics>> &state,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);

    ///
    /// \brief Return the muscle length Jacobian
    /// \return The muscle length Jacobian
    ///
    biorbd::utils::Matrix musclesLengthJacobian();
    ///
    /// \brief Return the muscle length Jacobian
    /// \param Q The position variables
    /// \return The muscle length Jacobian
    ///
    biorbd::utils::Matrix musclesLengthJacobian(const biorbd::rigidbody::GeneralizedCoordinates& Q);

    ///
    /// \brief Return the muscle forces
    /// \param state The dynamic state
    /// \param updateKin Update kinematics (default: True)
    /// \param Q The position variables
    /// \param QDot The velocity variables
    /// \return The muscle forces
    ///
    std::vector<std::vector<std::shared_ptr<Force>>> musclesForces(
            const std::vector<std::shared_ptr<StateDynamics>> &state,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);

    ///
    /// \brief Return the total number of muscle groups
    /// \return The total number of muscle groups
    ///
    unsigned int nbMuscleGroups() const; 
    ///
    /// \brief Return the total number of muscle
    /// \return The total number of muscle 
    ///
    unsigned int nbMuscleTotal() const; 
protected:
    std::shared_ptr<std::vector<biorbd::muscles::MuscleGroup>> m_mus;///< muscle group

};

}}

#endif // BIORBD_MUSCLES_MUSCLES_H

