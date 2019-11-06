#ifndef BIORBD_MUSCLES_HILL_TYPE_H
#define BIORBD_MUSCLES_HILL_TYPE_H

#include "biorbdConfig.h"
#include "Muscles/Muscle.h"

namespace biorbd {
namespace muscles {

    ///
    /// \brief Class HillType
    ///
class BIORBD_API HillType : public biorbd::muscles::Muscle
{
public:
    ///
    /// \brief Contruct a Hill-type muscle
    ///
    HillType();

    ///
    /// \brief Construct a Hill-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics);

    ///
    /// \brief Construct a Hill-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param dynamicState The muscle dynamic state
    ///
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::StateDynamics& dynamicState);

    ///
    /// \brief Construct a Hill-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The path changers
    ///
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers);

    ///
    /// \brief Construct a Hill-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The mupath changers
    /// \param dynamicState The dynamic state
    ///
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState);

    ///
    /// \brief Construct a Hill-type muscle from another muscle
    /// \param muscle The other muscle
    ///
    HillType(
            const biorbd::muscles::Muscle& muscle);

    ///
    /// \brief Construct a Hill-type muscle from another muscle
    /// \param muscle THe other muscle (pointer)
    ///
    HillType(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);

    ///
    /// \brief Deep copy of a Hill-type muscle
    /// \return A deep copy of a Hill-type muscle
    ///
    biorbd::muscles::HillType DeepCopy() const;

    ///
    /// \brief Deep copy of a Hill-type muscle in a new Hill-type muscle
    /// \param other The Hill-type to copy
    ///
    void DeepCopy(const biorbd::muscles::HillType& other);

    ///
    /// \brief Return combined force
    /// \param emg The EMG data
    /// \return The combined force
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            const biorbd::muscles::StateDynamics& emg);
    ///
    /// \brief Return combined force
    /// \param model The model
    /// \param Q The position variables
    /// \param Qdot The velocity variables
    /// \param emg The EMG data
    /// \param updateKin Update kinematics (default: 2)
    /// \return The combined Force
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);

    ///
    /// \brief Return combined force
    /// \param model The model
    /// \param Q The position variables
    /// \param emg The EMG data
    /// \param updateKin Update kinematics (default: 2)
    /// \return The combined force
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);

    // Get individual forces
    ///
    /// \brief Return the Force-Length contractile element
    /// \return The Force-Length contractile element
    ///
    double FlCE(const biorbd::muscles::StateDynamics& EMG);

    ///
    /// \brief Return the Force-Length passive element
    /// \return The Force-Length passive element
    ///
    double FlPE();

    ///
    /// \brief Return the Force-Velocity contractile element
    /// \return The Force-Velocity contractile element
    ///
    double FvCE();

    ///
    /// \brief Return the muscle damping (spring force)
    /// \return The muscle damping
    ///
    double damping();

protected:
    ///
    /// \brief Set type to Hill
    ///
    virtual void setType();

    // Intermediate steps to compute the force

    ///
    /// \brief Compute the muscle damping
    ///
    virtual void computeDamping(); 

    ///
    /// \brief Compute the Force-length contractile element
    ///
    virtual void computeFlCE(const biorbd::muscles::StateDynamics &EMG); 
    ///
    /// \brief Compute the Force-Velocity contractile element 
    ///
    virtual void computeFvCE(); 
    ///
    /// \brief Compute the Force-Length passive element
    ///
    virtual void computeFlPE(); 
    ///
    /// \brief Function allowing modification of the way the multiplication is done in computeForce(EMG)
    /// \param emg The EMG data
    /// \return The force from activation
    virtual double getForceFromActivation(const biorbd::muscles::State &emg); 
    virtual biorbd::muscles::StateDynamics normalizeEMG(const biorbd::muscles::StateDynamics& emg);

    // Attributs intermédiaires lors du calcul de la force
    std::shared_ptr<double> m_damping; ///< Muscle damping (spring force)
    std::shared_ptr<double> m_FlCE; ///<Force-Length contractile element
    std::shared_ptr<double> m_FlPE; /// Force-Length passive element 
    std::shared_ptr<double> m_FvCE; ///<Force-Velocity contractile element 

    // Declaration of multiple constants
    std::shared_ptr<double> m_cste_FlCE_1; ///< constant 1 used in the FlCE
    std::shared_ptr<double> m_cste_FlCE_2; ///< constant 2 used in the FlCE
    std::shared_ptr<double> m_cste_FvCE_1; ///< constant 1 used in the FvCE
    std::shared_ptr<double> m_cste_FvCE_2; ///< constant 2 used in the FvCE
    std::shared_ptr<double> m_cste_FlPE_1; ///< constant 1 used in the FlPE
    std::shared_ptr<double> m_cste_FlPE_2; ///< constant 2 used in the FlPE
    std::shared_ptr<double> m_cste_forceExcentriqueMultiplier; ///< Constant used for ForceVelocity
    std::shared_ptr<double> m_cste_damping; ///< parameters used in damping
    std::shared_ptr<double> m_cste_vitesseRaccourMax; ///< Maximal velocity of shortening

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_H
