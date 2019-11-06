#ifndef BIORBD_MUSCLES_STATE_ACTUAL_BUCHANAN_H
#define BIORBD_MUSCLES_STATE_ACTUAL_BUCHANAN_H

#include "biorbdConfig.h"
#include "Muscles/StateDynamics.h"

namespace biorbd {
namespace muscles {
    ///
    /// \brief Class State Dynamics Buchanan
    ///
class BIORBD_API StateDynamicsBuchanan : public biorbd::muscles::StateDynamics
{
public:
   ///
   /// \brief Construct state dynamics
   /// \param neuralCommand The muscle neural command (default: 0)
   /// \param activation The muscle activation (default: 0)
   ///
    StateDynamicsBuchanan(
            double neuralCommand = 0,
            double excitation = 0);
    ///
    /// \brief Construct a state dynamics Buchanan from another state dynamics Buchanan
    /// \param other The other state dynamics Buchanan
    ///
    StateDynamicsBuchanan(const biorbd::muscles::StateDynamicsBuchanan& other);
    ///
    /// \brief Destroy class properly
    ///
    ~StateDynamicsBuchanan();
    ///
    /// \brief Deep copy of state dynamics Buchanan
    /// \return A deep copy of state dynamics Buchanan
    ///
    biorbd::muscles::StateDynamicsBuchanan DeepCopy() const;
    ///
    /// \brief Deep copy of state dynamics Buchanan into another state dynamics Buchanan
    /// \parma other The state dynamics Buchanan to copy
    ///
    void DeepCopy(const biorbd::muscles::StateDynamicsBuchanan& other);
    ///
    /// \brief Compute and return the excitation velocity 
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized (default: false)
    /// \return The excitation velocity
    ///
    virtual double timeDerivativeExcitation(
            const Characteristics &c,
            bool alreadyNormalized);
    ///
    /// \brief Set the muscle excitation
    /// \param val Value of the muscle excitation
    ///
    virtual void setExcitation(double val);
    ///
    /// \brief Set the neural command
    /// \param val Value of the neural command
    ///
    virtual void setNeuralCommand(double val);
   ///
   /// \brief Set the shape factor
   /// \param val Value of the shape factor
   ///
    void shapeFactor(double m_shape_factor);
   ///
   /// \brief Return the shape factor
   /// \return The shape factor
   ///
    double shapeFactor() const;
    ///
    /// \brief Set the muscle activation
    /// \param val The value of the muscle activation
    ///
    void setActivation(double notUsed);

protected:
    ///
    /// \brief Set type to Buchanan
    ///
    virtual void setType();

    std::shared_ptr<double> m_neuralCommand; ///< The muscle neural command
    std::shared_ptr<double> m_shapeFactor; ///< The shape factor (Buchanan2004, march 22nd, 2018)
    std::shared_ptr<double> m_excitationDot; ///< The excitation velocity

};

}}

#endif // BIORBD_MUSCLES_STATE_ACTUAL_BUCHANAN_H
