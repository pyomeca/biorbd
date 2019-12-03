#ifndef BIORBD_MUSCLES_STATE_DYNAMICS_H
#define BIORBD_MUSCLES_STATE_DYNAMICS_H

#include "biorbdConfig.h"
#include "Muscles/State.h"

namespace biorbd {
namespace muscles {
class Characteristics;
///
/// \brief Class State Dynamics
///
class BIORBD_API StateDynamics : public biorbd::muscles::State
{
public:
    ///
   /// \brief Construct state dynamics
   /// \param excitation The muscle excitation (default: 0)
   /// \param activation The muscle activation (default: 0)
   ///
    StateDynamics(
            double excitation = 0,
            double activation = 0);
    ///
    /// \brief Construct a state dynamics from another state dynamics
    /// \param other The other state dynamics
    ///
    StateDynamics(const biorbd::muscles::StateDynamics& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~StateDynamics();
    ///
    /// \brief Deep copy of state dynamics
    /// \return A deep copy of state dynamics
    ///
    biorbd::muscles::StateDynamics DeepCopy() const;

    ///
    /// \brief Deep copy of state dynamics into another state dynamics
    /// \param other The state dynamics to copy
    ///
    void DeepCopy(const biorbd::muscles::StateDynamics& other);

    ///
    /// \brief Set the muscle excitation
    /// \param val The value of the muscle excitation
    ///
    virtual void setExcitation(double val);
    ///
    /// \brief Set the muscle activation
    /// \param val The value of the muscle activation
    ///
    virtual void setActivation(double val);
    ///
    /// \brief Return the last normalized excitation
    /// \param max The state
    /// \return The last normalized excitation
    ///
    double excitationNorm(const State &max);
    ///
    /// \brief Return the last normalized excitation
    /// \return The last normalized excitation
    ///
    double excitationNorm() const;
    ///
    /// \brief Set the normalized excitation
    /// \param val Value of the normalized excitation to set
    ///
    void setExcitationNorm(double val);

    ///
    /// \brief Return the previous activation
    /// \return The previous activation
    ///
    double previousActivation() const;
    ///
    /// \brief Return the previous activation
    /// \return The previous activation
    ///
    double previousExcitation() const;
    
    ///
    /// \brief Compute and return the activation velocity in function of the excitation and activation
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized (default: false)
    /// \return The activation velocity
    ///
    virtual double timeDerivativeActivation(
            double excitation,
            double activation,
            const Characteristics& characteristics,
            bool alreadyNormalized = false); 
    ///
    /// \brief Compute and return the activation velocity 
    /// \param state The dynamic state
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized (default: false)
    /// \return The activation velocity
    ///
    virtual double timeDerivativeActivation(
            const StateDynamics& state,
            const Characteristics& characteristics,
            bool alreadyNormalized = false); 

    ///
    /// \brief Compute and return the activation velocity 
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized (default: false)
    /// \return The activation velocity
    ///
    virtual double timeDerivativeActivation(
            const Characteristics& characteristics,
            bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    ///
    /// \brief Compute and return the last value of activation velocity 
    /// \return The last value activation velocity
    ///   
    virtual double timeDerivativeActivation();
protected:
    virtual void setType();
    std::shared_ptr<double> m_excitationNorm; ///< The normalized excitation
    std::shared_ptr<double> m_previousExcitation; ///< The previous excitation
    std::shared_ptr<double> m_previousActivation; ///<The previous activation
    std::shared_ptr<double> m_activationDot;///< The activation velocity

};

}}

#endif // BIORBD_MUSCLES_STATE_DYNAMICS_H
